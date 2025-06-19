# SPDX-License-Identifier: MIT
# Copyright (C) 2024, CodeMagic LTD

import base64
import json
import asyncio
from typing import List, Optional, Dict, Callable, Any

import websockets

CLIENT_VERSION = "0.0.1"


class WokwiError(Exception):
    """Exception raised for Wokwi API errors."""

    def __init__(self, message):
        self.message = message
        super().__init__(message)

    def __str__(self):
        return self.message


class WokwiClient:
    """Wokwi WebSocket API client."""

    next_id: int
    version = CLIENT_VERSION

    def __init__(self, token: str, server="wss://wokwi.com/api/ws/beta"):
        """Initialize the Wokwi client.

        Args:
            token: Your Wokwi CLI token
            server: WebSocket server URL (default: Wokwi beta endpoint)
        """
        self.token = token
        self.server = server
        self.next_id = 1
        self.event_handlers: Dict[str, List[Callable[[Any], None]]] = {}
        self.pending_responses: Dict[str, asyncio.Future] = {}
        self.message_loop_task: Optional[asyncio.Task] = None
        self.ws = None

    def __str__(self):
        return f"WokwiClient(server={self.server})"

    async def _recv(self):
        """Receive and parse a message from the WebSocket."""
        raw_message = await self.ws.recv()
        try:
            message = json.loads(raw_message)
        except json.JSONDecodeError as e:
            raise WokwiError(f"Failed to parse message: {raw_message}") from e

        if "type" not in message:
            raise WokwiError(f"Invalid message: {message}")

        if message["type"] == "error":
            raise WokwiError(f"Server error: {message['message']}")

        if message["type"] == "response" and message.get("error"):
            result = message["result"]
            code = result.get('code', 'unknown')
            msg = result.get('message', 'unknown error')
            raise WokwiError(f"Server error {code}: {msg}")

        return message

    async def send_command(self, command: str, params: dict):
        """Send a command and wait for the response."""
        cmd_id = self.next_id
        self.next_id += 1

        # Create a future for the response
        response_future = asyncio.Future()
        self.pending_responses[str(cmd_id)] = response_future

        # Send command
        command_msg = {
            "type": "command",
            "command": command,
            "params": params,
            "id": str(cmd_id)
        }
        await self.ws.send(json.dumps(command_msg))

        # Wait for the response
        try:
            return await response_future
        finally:
            # Clean up
            self.pending_responses.pop(str(cmd_id), None)

    async def connect(self):
        """Connect to the Wokwi WebSocket server."""
        headers = {
            "Authorization": f"Bearer {self.token}",
            "User-Agent": f"wokwi-client-py/{CLIENT_VERSION}",
        }

        self.ws = await websockets.connect(
            self.server,
            additional_headers=headers,
        )

        # Wait for hello message
        msg = await self._recv()
        if msg["type"] != "hello":
            raise WokwiError(f"Expected hello message, got {msg['type']}")

        if msg["protocolVersion"] != 1:
            protocol_ver = msg["protocolVersion"]
            raise WokwiError(f"Unsupported protocol version: {protocol_ver}")

        # Start the message dispatcher
        await self._start_message_dispatcher()

        return {"version": msg["appVersion"]}

    async def disconnect(self):
        """Disconnect from the Wokwi server and stop all tasks."""
        await self._stop_message_dispatcher()
        if self.ws:
            await self.ws.close()

    # File operations
    async def upload(self, name: str, content: bytearray):
        """Upload binary content to the simulator."""
        return await self.send_command(
            "file:upload",
            {
                "name": name,
                "binary": base64.b64encode(content).decode("ascii"),
            },
        )

    async def upload_file(self, filename: str,
                          local_path: Optional[str] = None):
        """Upload a file to the simulator."""
        if local_path is None:
            local_path = filename
        with open(local_path, "rb") as f:
            content = f.read()
        return await self.upload(filename, content)

    async def upload_text(self, name: str, content: str):
        """Upload text content to the simulator."""
        return await self.send_command("file:upload", {
            "name": name,
            "text": content
        })

    # Simulation control
    async def start_simulation(self, firmware: str, elf: str,
                               pause=False, chips: List[str] = None):
        """Start the simulation."""
        if chips is None:
            chips = []
        return await self.send_command("sim:start", {
            "firmware": firmware,
            "elf": elf,
            "pause": pause,
            "chips": chips,
        })

    async def pause_simulation(self):
        """Pause the simulation."""
        return await self.send_command("sim:pause", {})

    async def resume_simulation(self, pause_after: Optional[int] = None):
        """Resume the simulation, optionally with a timeout."""
        params = {}
        if pause_after is not None:
            params["pauseAfter"] = pause_after
        return await self.send_command("sim:resume", params)

    async def restart_simulation(self, pause=False):
        """Restart the simulation."""
        return await self.send_command("sim:restart", {"pause": pause})

    async def get_simulation_status(self):
        """Get current simulation status and time."""
        return await self.send_command("sim:status", {})

    # Serial Monitor operations
    async def serial_monitor_listen(self):
        """Start listening to serial monitor data."""
        return await self.send_command("serial-monitor:listen", {})

    async def serial_monitor_write(self, data: str):
        """Write text data to the serial monitor."""
        bytes_data = data.encode('utf-8')
        return await self.send_command("serial-monitor:write", {
            "bytes": list(bytes_data)
        })

    async def serial_monitor_write_bytes(self, bytes_data: List[int]):
        """Write raw bytes to the serial monitor."""
        return await self.send_command("serial-monitor:write", {
            "bytes": bytes_data
        })

    # Pin/GPIO operations
    async def pin_read(self, part: str, pin: str):
        """Read the current state of a pin."""
        return await self.send_command("pin:read", {
            "part": part,
            "pin": pin
        })

    async def gpio_write(self, pin: str, value: bool):
        """Set a GPIO pin to a given value."""
        return await self.send_command("gpio:write", {
            "pin": pin,
            "value": value
        })

    async def pin_listen(self, part: str, pin: str, listen: bool = True):
        """Listen for pin value/direction/configuration changes."""
        return await self.send_command("pin:listen", {
            "part": part,
            "pin": pin,
            "listen": listen
        })

    # Control operations
    async def control_set(self, part_id: str, control: str, value: float):
        """Set a control value on a part."""
        return await self.send_command("control:set", {
            "part": part_id,
            "control": control,
            "value": value
        })

    # Framebuffer operations
    async def framebuffer_read(self, device_id: str):
        """Read the current content of a device's framebuffer."""
        return await self.send_command("framebuffer:read", {
            "id": device_id
        })

    # Event handling
    def add_event_handler(self, event_name: str,
                          handler: Callable[[Any], None]):
        """Add an event handler for a specific event type."""
        if event_name not in self.event_handlers:
            self.event_handlers[event_name] = []
        self.event_handlers[event_name].append(handler)

    def remove_event_handler(self, event_name: str,
                             handler: Callable[[Any], None]):
        """Remove an event handler for a specific event type."""
        if event_name in self.event_handlers:
            self.event_handlers[event_name].remove(handler)
            if not self.event_handlers[event_name]:
                del self.event_handlers[event_name]

    # Message dispatcher (internal)
    async def _start_message_dispatcher(self):
        """Start the central message dispatcher."""
        if self.message_loop_task is not None:
            return  # Already running

        async def message_dispatcher():
            try:
                while True:
                    msg = await self._recv()
                    await self._dispatch_message(msg)
            except asyncio.CancelledError:
                pass
            except Exception as e:
                print(f"Message dispatcher error: {e}")

        self.message_loop_task = asyncio.create_task(message_dispatcher())

    async def _dispatch_message(self, message: dict):
        """Dispatch incoming messages to appropriate handlers."""
        if message["type"] == "response":
            # Handle command responses
            msg_id = message.get("id")
            if msg_id and msg_id in self.pending_responses:
                future = self.pending_responses[msg_id]
                if not future.done():
                    if message.get("error"):
                        # Create an exception for error responses
                        result = message["result"]
                        code = result.get('code', 'unknown')
                        msg = result.get('message', 'unknown error')
                        error = WokwiError(f"Server error {code}: {msg}")
                        future.set_exception(error)
                    else:
                        future.set_result(message)
        elif message["type"] == "event":
            # Handle events
            await self._handle_event(message)

    async def _handle_event(self, message: dict):
        """Handle incoming event messages."""
        event_name = message.get("event")
        if event_name and event_name in self.event_handlers:
            for handler in self.event_handlers[event_name]:
                try:
                    handler(message)
                except Exception as e:
                    print(f"Error in event handler for {event_name}: {e}")

    async def _stop_message_dispatcher(self):
        """Stop the message dispatcher."""
        if self.message_loop_task is not None:
            self.message_loop_task.cancel()
            try:
                await self.message_loop_task
            except asyncio.CancelledError:
                pass
            self.message_loop_task = None

    async def serial_monitor_cat(self):
        """Legacy method for monitoring serial output (blocking)."""
        await self.send_command("serial-monitor:listen", {})
        while True:
            msg = await self._recv()
            if (msg["type"] == "event"
                    and msg["event"] == "serial-monitor:data"):
                payload = msg["payload"]
                data = bytearray(payload["bytes"]).decode("ascii")
                print(data, end="")
