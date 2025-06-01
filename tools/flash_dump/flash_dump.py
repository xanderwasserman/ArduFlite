#!/usr/bin/env python3
# dump_flash_logs.py

import argparse
import serial
import serial.tools.list_ports
import time
import sys

def find_candidate_ports():
    """
    Return a list of serial ports that look like ESP32/USB-serial adapters.
    - Windows: COMx
    - macOS: /dev/cu.* or /dev/tty.*
    - Linux: /dev/ttyUSB* or /dev/ttyACM* or description containing "usbser"
    """
    ports = serial.tools.list_ports.comports()
    candidates = []

    for p in ports:
        dev = p.device.lower()
        desc = p.description.lower()

        if sys.platform.startswith("win"):
            if dev.startswith("com"):
                candidates.append(p.device)
        elif sys.platform.startswith("darwin"):
            if dev.startswith("/dev/cu.") or dev.startswith("/dev/tty."):
                candidates.append(p.device)
        else:
            if dev.startswith("/dev/ttyusb") or dev.startswith("/dev/ttyacm") or "usbser" in desc:
                candidates.append(p.device)

    # Remove duplicates while preserving order
    seen = set()
    unique = []
    for c in candidates:
        if c not in seen:
            unique.append(c)
            seen.add(c)

    return unique

def main():
    parser = argparse.ArgumentParser(
        description="Dump ArduFlite flash log over serial into a CSV file"
    )
    parser.add_argument(
        "-p", "--port",
        help="Serial port (e.g. COM3, /dev/cu.SLAB_USBtoUART). If omitted, tries to autodetect."
    )
    parser.add_argument(
        "-b", "--baud",
        type=int, default=115200,
        help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "-i", "--index",
        type=int, default=0,
        help="Log index to dump (default: 0)"
    )
    parser.add_argument(
        "-o", "--output",
        default=None,
        help="Output CSV filename (default: log_XXX.csv)"
    )
    args = parser.parse_args()

    # 1) Determine serial port
    port = args.port
    if not port:
        candidates = find_candidate_ports()
        if len(candidates) == 1:
            port = candidates[0]
            print(f"→ Autodetected port: {port}")
        elif len(candidates) == 0:
            print("→ No ESP32-like serial ports found.")
            print("Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  {p.device} ({p.description})")
            print("Please specify a port with -p <port>.")
            sys.exit(1)
        else:
            print("→ Multiple candidate ports found:")
            for idx, dev in enumerate(candidates):
                print(f"  [{idx}] {dev}")
            choice = input("Enter the index of the port you want to use: ").strip()
            if not choice.isdigit():
                print("Invalid input; exiting.")
                sys.exit(1)
            idx = int(choice)
            if idx < 0 or idx >= len(candidates):
                print("Index out of range; exiting.")
                sys.exit(1)
            port = candidates[idx]
            print(f"→ Using port: {port}")

    # 2) Prepare output filename
    if args.output is None:
        args.output = f"log_{args.index:03d}.csv"

    # 3) Open serial port without causing auto-reset on close
    try:
        ser = serial.Serial()
        ser.port     = port
        ser.baudrate = args.baud
        ser.timeout  = 1
        ser.dsrdtr   = False  # disable DSR/DTR handshake
        ser.rtscts   = False  # disable RTS/CTS handshake

        # Force DTR low *before* opening to avoid reset
        ser.dtr = False
        ser.rts = False

        ser.open()
    except serial.SerialException as e:
        print(f"Error opening serial port {port}: {e}")
        sys.exit(1)

    # 4) Give the ESP32 a moment to reset and settle
    time.sleep(2)
    ser.reset_input_buffer()

    # 5) Send the dump command
    cmd = f"flash dump {args.index}\n"
    ser.write(cmd.encode("utf-8"))

    recording = False
    with open(args.output, "w", newline="") as fout:
        print(f"→ Listening for log #{args.index}, writing to {args.output} …")
        while True:
            line = ser.readline()
            if not line:
                # timeout without data
                continue
            text = line.decode("utf-8", errors="ignore").rstrip("\r\n")

            # 6a) Look for the BEGIN marker
            if not recording:
                if text.startswith("--- BEGIN"):
                    recording = True
                continue

            # 6b) Look for the END marker
            if text.startswith("--- END"):
                print("→ End of log reached.")
                break

            # 6c) Otherwise, write this CSV row
            fout.write(text + "\n")

    ser.close()
    print("→ Done.")

if __name__ == "__main__":
    main()
