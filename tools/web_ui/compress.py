#!/usr/bin/env python3
"""
compress.py - Compress web assets for ArduFlite WebUI

Reads HTML, CSS, JS from src/ and generates a C header with gzip-compressed
byte arrays suitable for PROGMEM storage.

Usage:
    cd tools/web_ui
    python3 compress.py > ../../src/web/WebUI.h

Author: Alexander Wasserman
Date: 09 February 2026
"""

import gzip
import os
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
SRC_DIR = SCRIPT_DIR / "src"

def read_file(name: str) -> bytes:
    """Read a source file and return as bytes."""
    path = SRC_DIR / name
    with open(path, "r", encoding="utf-8") as f:
        return f.read().encode("utf-8")

def compress(data: bytes) -> bytes:
    """Gzip compress data with maximum compression."""
    return gzip.compress(data, compresslevel=9)

def to_c_array(data: bytes, name: str, per_line: int = 16) -> str:
    """Convert bytes to C array declaration."""
    lines = []
    for i in range(0, len(data), per_line):
        chunk = data[i:i+per_line]
        hex_values = ", ".join(f"0x{b:02x}" for b in chunk)
        lines.append(f"    {hex_values},")
    
    array_content = "\n".join(lines)
    return f"""static const uint8_t {name}[] PROGMEM = {{
{array_content}
}};

static const size_t {name}_LEN = {len(data)};
"""

def main():
    # Read source files
    html = read_file("index.html")
    css = read_file("styles.css")
    js = read_file("app.js")
    
    # Compress
    html_gz = compress(html)
    css_gz = compress(css)
    js_gz = compress(js)
    
    # Report compression stats
    import sys
    print(f"// Compression stats:", file=sys.stderr)
    print(f"//   HTML: {len(html)} -> {len(html_gz)} bytes ({100*len(html_gz)/len(html):.1f}%)", file=sys.stderr)
    print(f"//   CSS:  {len(css)} -> {len(css_gz)} bytes ({100*len(css_gz)/len(css):.1f}%)", file=sys.stderr)
    print(f"//   JS:   {len(js)} -> {len(js_gz)} bytes ({100*len(js_gz)/len(js):.1f}%)", file=sys.stderr)
    print(f"//   Total: {len(html)+len(css)+len(js)} -> {len(html_gz)+len(css_gz)+len(js_gz)} bytes", file=sys.stderr)
    
    # Generate header
    header = f'''/**
 * WebUI.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 09 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Embedded web UI for configuration (gzip-compressed in PROGMEM).
 *
 * This file is auto-generated. To regenerate:
 *   cd tools/web_ui && python3 compress.py > ../../src/web/WebUI.h
 *
 * For development, raw assets are in tools/web_ui/src/
 *
 * Compression stats:
 *   HTML: {len(html)} -> {len(html_gz)} bytes ({100*len(html_gz)/len(html):.1f}%)
 *   CSS:  {len(css)} -> {len(css_gz)} bytes ({100*len(css_gz)/len(css):.1f}%)
 *   JS:   {len(js)} -> {len(js_gz)} bytes ({100*len(js_gz)/len(js):.1f}%)
 *   Total: {len(html)+len(css)+len(js)} -> {len(html_gz)+len(css_gz)+len(js_gz)} bytes
 */
#ifndef WEB_UI_H
#define WEB_UI_H

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════
// HTML (gzip compressed)
// ═══════════════════════════════════════════════════════════════════════════

{to_c_array(html_gz, "WEB_UI_HTML_GZ")}

// ═══════════════════════════════════════════════════════════════════════════
// CSS (gzip compressed)
// ═══════════════════════════════════════════════════════════════════════════

{to_c_array(css_gz, "WEB_UI_CSS_GZ")}

// ═══════════════════════════════════════════════════════════════════════════
// JavaScript (gzip compressed)
// ═══════════════════════════════════════════════════════════════════════════

{to_c_array(js_gz, "WEB_UI_JS_GZ")}

#endif // WEB_UI_H
'''
    
    print(header)

if __name__ == "__main__":
    main()
