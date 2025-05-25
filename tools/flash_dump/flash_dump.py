#!/usr/bin/env python3
# dump_flash_logs.py

import argparse
import serial
import time
import sys

def main():
    parser = argparse.ArgumentParser(description="Dump ArduFlite flash log over serial into a CSV file")
    parser.add_argument("-p", "--port", required=True, help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("-i", "--index", type=int, default=0, help="Log index to dump (default: 0)")
    parser.add_argument("-o", "--output", default=None, help="Output CSV filename (default: log_XXX.csv)")
    args = parser.parse_args()

    # Default output filename if none given
    if args.output is None:
        args.output = f"log_{args.index:03d}.csv"

    # Open serial port
    try:
        ser = serial.Serial()
        ser.port      = args.port
        ser.baudrate  = args.baud
        ser.timeout   = 1
        ser.dsrdtr    = False    # disable DSR/DTR handshake
        ser.rtscts    = False    # disable RTS/CTS handshake

        # 2) Force DTR low *before* open
        ser.dtr = False
        ser.rts = False

        # 3) Now open the port — it will inherit DTR=False
        ser.open()

    except serial.SerialException as e:
        print(f"Error opening serial port {args.port}: {e}")
        sys.exit(1)

    # Give the ESP32 a moment to reset/settle
    time.sleep(2)
    ser.reset_input_buffer()

    # Send the dump command
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
            # Start marker
            if not recording:
                if text.startswith("--- BEGIN"):
                    recording = True
                continue
            # End marker
            if text.startswith("--- END"):
                print("→ End of log reached.")
                break
            # Otherwise write to file
            fout.write(text + "\n")

    ser.close()
    print("→ Done.")

if __name__ == "__main__":
    main()
