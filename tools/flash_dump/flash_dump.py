#!/usr/bin/env python3
# dump_flash_logs.py

import argparse
import serial
import serial.tools.list_ports
import time
import sys
import re
from pathlib import Path

def find_candidate_ports():
    ports = serial.tools.list_ports.comports()
    cands = []
    for p in ports:
        dev = p.device.lower()
        desc = p.description.lower()
        if sys.platform.startswith("win"):
            if dev.startswith("com"):
                cands.append(p.device)
        elif sys.platform.startswith("darwin"):
            if dev.startswith("/dev/cu.") or dev.startswith("/dev/tty."):
                cands.append(p.device)
        else:
            if dev.startswith("/dev/ttyusb") or dev.startswith("/dev/ttyacm") or "usbser" in desc:
                cands.append(p.device)
    # dedupe
    seen = set(); unique = []
    for c in cands:
        if c not in seen:
            unique.append(c); seen.add(c)
    return unique

def list_logs(ser):
    """Send 'flash list' and return a list of indices (ints) found."""
    ser.reset_input_buffer()
    ser.write(b"flash list\n")
    pattern = re.compile(r"^log_(\d{3})\.csv$")
    found = []
    t0 = time.time()
    while time.time() - t0 < 5:
        line = ser.readline().decode("utf-8", "ignore").strip()
        if not line:
            continue
        m = pattern.match(line)
        if m:
            found.append(int(m.group(1)))
        elif found:
            # once we've started seeing logs, any non-log line ends the list
            break
    return sorted(set(found))

def dump_one(ser, idx, out_dir):
    """Dump a single log index to out_dir/log_###.csv"""
    out_path = Path(out_dir) / f"log_{idx:03d}.csv"
    ser.reset_input_buffer()
    ser.write(f"flash dump {idx}\n".encode())
    recording = False

    with open(out_path, "w", newline="") as fout:
        print(f"→ Dumping log {idx:03d} → {out_path}")
        while True:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", "ignore").rstrip("\r\n")
            if not recording:
                if text.startswith("--- BEGIN"):
                    recording = True
                continue
            if text.startswith("--- END"):
                break
            fout.write(text + "\n")

def open_port(port, baud):
    ser = serial.Serial()
    ser.port     = port
    ser.baudrate = baud
    ser.timeout  = 1
    ser.dsrdtr   = False
    ser.rtscts   = False
    # try to suppress resets
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(2)
    ser.reset_input_buffer()
    return ser

def main():
    p = argparse.ArgumentParser(description="Dump ArduFlite flash log(s) to CSV")
    p.add_argument("-p","--port", help="Serial port; if omitted, will try to autodetect")
    p.add_argument("-b","--baud", type=int, default=115200, help="Baud rate (default 115200)")
    p.add_argument("-i","--index", type=int, help="Single log index to dump (e.g. 0).")
    p.add_argument("-o","--output", default=".", help="Output directory (default: current).")
    p.add_argument("--all", action="store_true", help="Dump *all* logs returned by `flash list`")
    args = p.parse_args()

    # pick port
    port = args.port
    if not port:
        cands = find_candidate_ports()
        if len(cands)==1:
            port = cands[0]; print(f"→ Autodetected port: {port}")
        elif not cands:
            print("No serial ports found. Use -p to specify one."); sys.exit(1)
        else:
            print("Multiple ports found:")
            for i,dev in enumerate(cands): print(f"  [{i}] {dev}")
            sel = input("Choose index: ").strip()
            if not sel.isdigit() or int(sel) not in range(len(cands)):
                print("Invalid selection"); sys.exit(1)
            port = cands[int(sel)]

    out_dir = args.output
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    try:
        ser = open_port(port, args.baud)
    except Exception as e:
        print("Failed to open port:", e); sys.exit(1)

    indices = []
    if args.all:
        indices = list_logs(ser)
        if not indices:
            print("No logs to dump."); sys.exit(0)
    else:
        if args.index is None:
            print("Specify either --index or --all"); sys.exit(1)
        indices = [args.index]

    for idx in indices:
        dump_one(ser, idx, out_dir)

    ser.close()
    print("→ All done.")

if __name__=="__main__":
    main()
