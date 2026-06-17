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

def inline_assets(html: bytes, css: bytes, js: bytes) -> bytes:
    """Inline CSS and JS into the HTML used by firmware builds."""
    html_text = html.decode("utf-8")
    css_text = css.decode("utf-8")
    js_text = js.decode("utf-8")

    css_tag = '<link rel="stylesheet" href="/styles.css">'
    js_tag = '<script src="/app.js"></script>'

    if css_tag not in html_text:
        raise RuntimeError(f"Missing CSS tag in index.html: {css_tag}")
    if js_tag not in html_text:
        raise RuntimeError(f"Missing JS tag in index.html: {js_tag}")

    html_text = html_text.replace(css_tag, f"<style>\n{css_text}\n</style>", 1)
    html_text = html_text.replace(js_tag, f"<script>\n{js_text}\n</script>", 1)
    return html_text.encode("utf-8")

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
    html_shell = read_file("index.html")
    css = read_file("styles.css")
    js = read_file("app.js")
    html = inline_assets(html_shell, css, js)

    # Compress
    html_gz = compress(html)

    # Report compression stats
    import sys
    print(f"// Compression stats:", file=sys.stderr)
    print(f"//   HTML shell: {len(html_shell)} bytes", file=sys.stderr)
    print(f"//   CSS:        {len(css)} bytes", file=sys.stderr)
    print(f"//   JS:         {len(js)} bytes", file=sys.stderr)
    print(f"//   Inlined:    {len(html)} -> {len(html_gz)} bytes ({100*len(html_gz)/len(html):.1f}%)", file=sys.stderr)
    
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
 *   HTML shell: {len(html_shell)} bytes
 *   CSS:        {len(css)} bytes
 *   JS:         {len(js)} bytes
 *   Inlined:    {len(html)} -> {len(html_gz)} bytes ({100*len(html_gz)/len(html):.1f}%)
 */
#ifndef WEB_UI_H
#define WEB_UI_H

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════
// HTML (gzip compressed)
// ═══════════════════════════════════════════════════════════════════════════

{to_c_array(html_gz, "WEB_UI_HTML_GZ")}

#endif // WEB_UI_H
'''
    
    print(header)

if __name__ == "__main__":
    main()
