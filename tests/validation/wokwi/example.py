# SPDX-License-Identifier: MIT
# Copyright (C) 2024, CodeMagic LTD

import os
import asyncio
from wokwi import WokwiClient


async def main():
    client = WokwiClient(os.getenv("WOKWI_CLI_TOKEN"))
    print(f"Wokwi client library version: {client.version}")
    result = await client.connect()
    print("Connected to Wokwi Simulator, server version:", result["version"])

    # Upload files
    await client.upload_file("diagram.json", "diagram.esp32.json")
    await client.upload_file("wokwi.bin", "../../../../../../../../.arduino/tests/esp32/wokwi/build.tmp/wokwi.ino.merged.bin")
    await client.upload_file("wokwi.elf", "../../../../../../../../.arduino/tests/esp32/wokwi/build.tmp/wokwi.ino.elf")

    # Set up event handler for serial data
    def handle_serial_data(event):
        """Handle incoming serial data events."""
        payload = event["payload"]
        data = bytearray(payload["bytes"]).decode("ascii", errors="ignore")
        print(f"[SERIAL] {data}", end="")

    def handle_pin_change(event):
        """Handle pin change events."""
        payload = event["payload"]
        print(f"\n[PIN] {payload['part']}.{payload['pin']}: {payload['value']}")

    # Register event handlers
    # client.add_event_handler("serial-monitor:data", handle_serial_data)
    client.add_event_handler("pin:change", handle_pin_change)

    # Start simulation
    await client.start_simulation("wokwi.bin", "wokwi.elf")
    print("Simulation started as paused")

    # Start listening to serial monitor
    await client.serial_monitor_listen()

    # Try to set up pin monitoring
    await client.pin_listen("esp32", "2")
    print("Monitoring pin 2 for changes")
    await asyncio.sleep(1)

    delay_time = 3

    for i in range(2):
        await client.control_set("btn1", "pressed", 1)
        print("\nSent message to serial monitor: '1' (turn ON LED)")
        await asyncio.sleep(delay_time)
        print("pin_read: ", (await client.pin_read("esp32", "2"))["result"]["value"])

        await client.control_set("btn1", "pressed", 0)
        print("\nSent message to serial monitor: '0' (turn OFF LED)")
        await asyncio.sleep(delay_time)
        print("pin_read: ", (await client.pin_read("esp32", "2"))["result"]["value"])

    await client.disconnect()
    print("Disconnected")


if __name__ == "__main__":
    asyncio.run(main())
