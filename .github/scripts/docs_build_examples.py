#!/usr/bin/env python3
"""
Documentation build examples script for ESP32 Arduino.

This script manages binary preparation and cleanup for documentation examples.
It can build examples with upload-binary configuration, clean up the binaries
directory, and optionally generate diagrams and LaunchPad configurations.

The script requires Arduino CLI and user paths when building examples.
It processes all sketches that have 'upload-binary' configuration in their
ci.json files and builds them for specified targets.

Usage:
    docs_build_examples.py -c                                    # Clean up
    docs_build_examples.py --build -ai /path/cli -au /path/user  # Build all
    docs_build_examples.py --build -ai /path/cli -au /path/user --diagram --launchpad  # Build with extras

Environment Variables:
    ARDUINO_ESP32_PATH: Path to ESP32 Arduino core
    GITHUB_WORKSPACE: GitHub workspace path (fallback)
    DOCS_PROD_URL_BASE: Base URL for documentation deployment
    REPO_URL_PREFIX: Repository URL prefix for LaunchPad configs
"""

import argparse
from argparse import RawDescriptionHelpFormatter
from esp_docs.generic_extensions.docs_embed.tool.wokwi_tool import DiagramSync
import os
import shutil
import sys
from pathlib import Path
import subprocess
import yaml
import platform

SCRIPT_DIR = Path(__file__).resolve().parent

ARDUINO_ESP32_PATH = os.environ.get("ARDUINO_ESP32_PATH")
GITHUB_WORKSPACE = os.environ.get("GITHUB_WORKSPACE")
STORAGE_URL_PREFIX = os.environ.get("STORAGE_URL_PREFIX")
REPO_URL_PREFIX = os.environ.get("REPO_URL_PREFIX")

if ARDUINO_ESP32_PATH and (Path(ARDUINO_ESP32_PATH) / "tools" / "esp32-arduino-libs").is_dir():
    SDKCONFIG_DIR = Path(ARDUINO_ESP32_PATH) / "tools" / "esp32-arduino-libs"
elif GITHUB_WORKSPACE and (Path(GITHUB_WORKSPACE) / "tools" / "esp32-arduino-libs").is_dir():
    SDKCONFIG_DIR = Path(GITHUB_WORKSPACE) / "tools" / "esp32-arduino-libs"
else:
    SDKCONFIG_DIR = Path("tools/esp32-arduino-libs")

KEEP_FILES = [
    "*.merged.bin",
    "ci.yml",
    "launchpad.toml",
    "diagram*.json",
    ".gitignore",
]

SKETCH_UTILS = SCRIPT_DIR / "sketch_utils.sh"
DOCS_BINARIES_DIR = Path("docs/_static/binaries")


def detect_arduino_paths():
    """Get Arduino CLI and user paths from environment variables set by install-arduino-cli.sh

    The function will create the Arduino user directory if it doesn't exist.

    Returns:
        tuple: (arduino_cli_path, arduino_user_path) or (None, None) if not found
    """
    try:
        # Get paths from environment variables exported by install-arduino-cli.sh
        arduino_ide_path = os.environ.get("ARDUINO_IDE_PATH")
        arduino_usr_path = os.environ.get("ARDUINO_USR_PATH")

        if not arduino_ide_path or not arduino_usr_path:
            print("Arduino paths not found in environment variables.")
            print("Make sure to run install-arduino-cli.sh first to set ARDUINO_IDE_PATH and ARDUINO_USR_PATH")
            return None, None

        # Convert to Path objects for validation
        ide_path = Path(arduino_ide_path)
        usr_path = Path(arduino_usr_path)

        # Check if arduino-cli exists
        arduino_cli_exe = ide_path / "arduino-cli"
        if platform.system().lower() == "windows":
            arduino_cli_exe = ide_path / "arduino-cli.exe"

        if arduino_cli_exe.exists():
            # Create user path if it doesn't exist
            if not usr_path.exists():
                try:
                    usr_path.mkdir(parents=True, exist_ok=True)
                    print(f"Created Arduino user directory: {usr_path}")
                except Exception as e:
                    print(f"Failed to create Arduino user directory {usr_path}: {e}")
                    return None, None

            return str(ide_path), str(usr_path)
        else:
            print(f"Arduino CLI or user path not found:")
            print(f"  Arduino CLI: {arduino_cli_exe} {'✓' if arduino_cli_exe.exists() else '✗'}")
            print(f"  User path: {usr_path} {'✓' if usr_path.exists() else '✗'}")
            return None, None

    except Exception as e:
        print(f"Error getting Arduino paths from environment: {e}")
        return None, None
def run_cmd(cmd, check=True, capture_output=False, text=True):
    """Execute a shell command with error handling.

    Args:
        cmd (list): Command and arguments to execute
        check (bool): Whether to raise exception on non-zero exit code
        capture_output (bool): Whether to capture stdout/stderr
        text (bool): Whether to return text output instead of bytes
    """
    try:
        return subprocess.run(cmd, check=check, capture_output=capture_output, text=text)
    except subprocess.CalledProcessError as e:
        # CalledProcessError is raised only when check=True and the command exits non-zero
        print(f"ERROR: Command failed: {' '.join(cmd)}")
        print(f"Exit code: {e.returncode}")
        if hasattr(e, 'stdout') and e.stdout:
            print("--- stdout ---")
            print(e.stdout)
        if hasattr(e, 'stderr') and e.stderr:
            print("--- stderr ---")
            print(e.stderr)
        # Exit the whole script with the same return code to mimic shell behavior
        sys.exit(e.returncode)
    except FileNotFoundError:
        print(f"ERROR: Command not found: {cmd[0]}")
        sys.exit(127)


def check_requirements(sketch_dir, sdkconfig_path):
    """Check if sketch meets requirements for the given SDK config.

    Args:
        sketch_dir (str): Path to the sketch directory
        sdkconfig_path (str): Path to the SDK config file

    Returns:
        bool: True if requirements are met, False otherwise
    """
    cmd = [str(SKETCH_UTILS), "check_requirements", sketch_dir, str(sdkconfig_path)]
    try:
        res = run_cmd(cmd, check=False, capture_output=True)
        return res.returncode == 0
    except Exception:
        return False


def install_libs(*args):
    """Install Arduino libraries using sketch_utils.sh"""
    cmd = [str(SKETCH_UTILS), "install_libs"] + list(args)
    return run_cmd(cmd, check=False)


def build_sketch(args_list):
    """Build a sketch using sketch_utils.sh"""
    cmd = [str(SKETCH_UTILS), "build"] + args_list
    return run_cmd(cmd, check=False)


def parse_args(argv):
    """Parse command line arguments"""
    epilog_text = (
        "Examples:\n"
        "  docs_build_examples.py -c                                    # Clean up binaries directory\n"
        "  docs_build_examples.py --build                               # Build all examples (use env vars)\n"
        "  docs_build_examples.py --build -ai /path/to/cli -au /path/to/user  # Build with explicit paths\n"
        "  docs_build_examples.py --build --diagram --launchpad         # Build with diagrams and LaunchPad\n\n"
        "Path detection:\n"
        "  Arduino paths are read from ARDUINO_IDE_PATH and ARDUINO_USR_PATH environment variables\n"
        "  Set by running install-arduino-cli.sh first, or use -ai/-au to override\n\n"
    )

    p = argparse.ArgumentParser(
        description="Build examples that have ci.yml with upload-binary targets",
        formatter_class=RawDescriptionHelpFormatter,
        epilog=epilog_text,
    )
    p.add_argument(
        "-c", "--cleanup",
        dest="cleanup",
        action="store_true",
        help="Clean up docs binaries directory and exit",
    )
    p.add_argument(
        "-ai", "--arduino-cli-path",
        dest="arduino_cli_path",
        help="Path to Arduino CLI installation directory (overrides ARDUINO_IDE_PATH env var)",
    )
    p.add_argument(
        "-au", "--arduino-user-path",
        dest="user_path",
        help="Path to Arduino user directory (overrides ARDUINO_USR_PATH env var)",
    )
    p.add_argument(
        "-b", "--build",
        dest="build",
        action="store_true",
        help="Build all examples",
    )
    p.add_argument(
        "-d", "--diagram",
        dest="generate_diagrams",
        action="store_true",
        help="Generate diagrams for prepared examples using docs-embed",
    )
    p.add_argument(
        "-l", "--launchpad",
        dest="generate_launchpad_config",
        action="store_true",
        help="Generate LaunchPad config for prepared examples",
    )
    return p.parse_args(argv)


def validate_prerequisites(args):
    """Validate that required prerequisites are available and get paths from env vars if needed."""

    # Get paths from environment variables if not provided via arguments
    if not args.arduino_cli_path or not args.user_path:
        print("Getting Arduino paths from environment variables...")
        detected_cli_path, detected_user_path = detect_arduino_paths()

        if not args.arduino_cli_path:
            if detected_cli_path:
                args.arduino_cli_path = detected_cli_path
                print(f"  Arduino CLI path (ARDUINO_IDE_PATH): {detected_cli_path}")
            else:
                print("ERROR: Arduino CLI path not found in ARDUINO_IDE_PATH env var and not provided (-ai option)")
                print("Run install-arduino-cli.sh first to set environment variables")
                sys.exit(1)

        if not args.user_path:
            if detected_user_path:
                args.user_path = detected_user_path
                print(f"  Arduino user path (ARDUINO_USR_PATH): {detected_user_path}")
            else:
                print("ERROR: Arduino user path not found in ARDUINO_USR_PATH env var and not provided (-au option)")
                print("Run install-arduino-cli.sh first to set environment variables")
                sys.exit(1)

    # Validate the paths
    arduino_cli_exe = Path(args.arduino_cli_path) / "arduino-cli"
    if platform.system().lower() == "windows":
        arduino_cli_exe = Path(args.arduino_cli_path) / "arduino-cli.exe"

    if not arduino_cli_exe.exists():
        print(f"ERROR: arduino-cli not found at {arduino_cli_exe}")
        sys.exit(1)

    # Create Arduino user path if it doesn't exist
    user_path = Path(args.user_path)
    if not user_path.is_dir():
        try:
            user_path.mkdir(parents=True, exist_ok=True)
            print(f"Created Arduino user directory: {user_path}")
        except Exception as e:
            print(f"ERROR: Failed to create Arduino user path {user_path}: {e}")
            sys.exit(1)
def cleanup_binaries():
    """Clean up the binaries directory, keeping only specified files.

    Removes all files except those matching patterns in KEEP_FILES.
    Also removes empty directories after cleanup.
    """
    print(f"Cleaning up binaries directory: {DOCS_BINARIES_DIR}")
    if not DOCS_BINARIES_DIR.exists():
        print("Binaries directory does not exist, nothing to clean")
        return
    for root, dirs, files in os.walk(DOCS_BINARIES_DIR):
        for fname in files:
            fpath = Path(root) / fname
            parent = Path(root).name
            # Always remove sketch/ci.yml
            if parent == "sketch" and fname == "ci.yml":
                fpath.unlink()
                continue
            keep = False
            for pattern in KEEP_FILES:
                if Path(fname).match(pattern):
                    keep = True
                    break
            if not keep:
                print(f"Removing: {fpath}")
                fpath.unlink()
            else:
                print(f"Keeping: {fpath}")
    # remove empty dirs
    for root, dirs, files in os.walk(DOCS_BINARIES_DIR, topdown=False):
        if not os.listdir(root):
            try:
                os.rmdir(root)
            except Exception:
                pass
    print("Cleanup completed")


def find_examples_with_upload_binary():
    """Find all Arduino sketches that have upload-binary configuration.

    Returns:
        list: List of paths to .ino files that have upload-binary in ci.yml
    """
    res = []
    for ino in Path('.').rglob('*.ino'):
        sketch_dir = ino.parent
        sketch_name = ino.stem
        dir_name = sketch_dir.name
        if dir_name != sketch_name:
            continue
        ci_yml = sketch_dir / 'ci.yml'
        if ci_yml.exists():
            try:
                data = yaml.safe_load(ci_yml.read_text())
                if 'upload-binary' in data and data['upload-binary']:
                    res.append(str(ino))
            except Exception:
                continue
    return res


def get_upload_binary_targets(sketch_dir):
    """Get the upload-binary targets from a sketch's ci.yml file.

    Args:
        sketch_dir (str or Path): Path to the sketch directory

    Returns:
        list: List of target names for upload-binary, empty if none found
    """
    ci_yml = Path(sketch_dir) / 'ci.yml'
    try:
        data = yaml.safe_load(ci_yml.read_text())
        targets = data.get('upload-binary', {}).get('targets', [])
        return targets
    except Exception:
        return []


def build_example_for_target(sketch_dir, target, relative_path, args):
    """Build a single example for a specific target.

    Args:
        sketch_dir (Path): Path to the sketch directory
        target (str): Target board/configuration name
        relative_path (str): Relative path for output organization
        args (argparse.Namespace): Parsed command line arguments

    Returns:
        bool: True if build succeeded, False otherwise
    """
    print(f"\n > Building example: {relative_path} for target: {target}")
    output_dir = DOCS_BINARIES_DIR / relative_path / target
    output_dir.mkdir(parents=True, exist_ok=True)

    sdkconfig = SDKCONFIG_DIR / target / 'sdkconfig'
    if not check_requirements(str(sketch_dir), sdkconfig):
        print(f"Target {target} does not meet the requirements for {Path(sketch_dir).name}. Skipping.")
        return True

    # Build the sketch using sketch_utils.sh build - pass args as in shell script
    build_args = [
        "-ai",
        args.arduino_cli_path,
        "-au",
        args.user_path,
        "-s",
        str(sketch_dir),
        "-t",
        target,
        "-b",
        str(output_dir),
        "--first-only",
    ]
    res = build_sketch(build_args)
    if res.returncode == 0:
        print(f"Successfully built {relative_path} for {target}")
        ci_yml = Path(sketch_dir) / 'ci.yml'
        if ci_yml.exists():
            shutil.copy(ci_yml, output_dir / 'ci.yml')
        if args.generate_diagrams:
            print(f"Generating diagram for {relative_path} ({target})...")
            try:
                sync = DiagramSync(output_dir)
                sync.generate_diagram_from_ci(target)
            except Exception as e:
                print(f"WARNING: Failed to generate diagram for {relative_path} ({target}): {e}")
    else:
        print(f"ERROR: Failed to build {relative_path} for {target}")
        return False
    return True


def build_all_examples(args):
    """Build all examples that have upload-binary configuration

    Prerequisites are validated in main() before calling this function
    """
    total_built = 0
    total_failed = 0

    if DOCS_BINARIES_DIR.exists():
        shutil.rmtree(DOCS_BINARIES_DIR)
        print(f"Removed existing build directory: {DOCS_BINARIES_DIR}")

    # add gitignore to binaries dir with * for new files
    DOCS_BINARIES_DIR.mkdir(parents=True, exist_ok=True)
    gitignore_path = DOCS_BINARIES_DIR / '.gitignore'
    gitignore_path.write_text("*\n")

    examples = find_examples_with_upload_binary()
    if not examples:
        print("No examples found with upload-binary configuration")
        return 0

    print('\nExamples to be built:')
    print('====================')
    for i, example in enumerate(examples, start=1):
        sketch_dir = Path(example).parent
        relative_path = str(sketch_dir).lstrip('./')
        targets = get_upload_binary_targets(sketch_dir)
        if targets:
            print(f"{i}. {relative_path} (targets: {' '.join(targets)})")
    print()

    for example in examples:
        sketch_dir = Path(example).parent
        relative_path = str(sketch_dir).lstrip('./')
        targets = get_upload_binary_targets(sketch_dir)
        if not targets:
            print(f"WARNING: No targets found for {relative_path}")
            continue
        print(f"Building {relative_path} for targets: {targets}")
        for target in targets:
            ok = build_example_for_target(sketch_dir, target, relative_path, args)
            if ok is not False:  # Handle None return as success
                total_built += 1
            else:
                total_failed += 1

        output_sketch_dir = DOCS_BINARIES_DIR / relative_path
        output_sketch_dir.mkdir(parents=True, exist_ok=True)

        # copy sketch ci.yml to output dir - parent of target dirs
        ci_yml = sketch_dir / 'ci.yml'
        if ci_yml.exists():
            shutil.copy(ci_yml, output_sketch_dir / 'ci.yml')

        if args.generate_launchpad_config:
            print(f"Generating LaunchPad config for {relative_path}/{target}...")
            try:
                sync = DiagramSync(output_sketch_dir / target)
                sync.generate_launchpad_config(STORAGE_URL_PREFIX, REPO_URL_PREFIX, True, output_sketch_dir)
            except Exception as e:
                print(f"WARNING: Failed to generate LaunchPad config for {relative_path}/{target}: {e}")

    print('\nBuild summary:')
    print(f"  Successfully built: {total_built}")
    print(f"  Failed builds: {total_failed}")
    print(f"  Output directory: {DOCS_BINARIES_DIR}")
    return total_failed


def main(argv):
    """Main entry point for the script"""
    args = parse_args(argv)

    if args.cleanup:
        cleanup_binaries()
        return

    if args.build:
        # Validate prerequisites and auto-detect paths if needed
        validate_prerequisites(args)

        result = build_all_examples(args)
        if result == 0:
            print('\nAll examples built successfully!')
        else:
            print('\nSome builds failed. Check the output above for details.')
            sys.exit(1)
        return

    if args.generate_diagrams or args.generate_launchpad_config:
        print("ERROR: --diagram and --launchpad options are only available when building examples (--build)")
        sys.exit(1)

    # If no specific action is requested, show help
    parse_args(['--help'])


if __name__ == '__main__':
    main(sys.argv[1:])
