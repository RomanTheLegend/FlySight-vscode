#!/usr/bin/env python3
"""
Convert TTF fonts to ActiveLook goggle binary font format.

Characters included: '.' (ASCII 46) and '0'-'9' (ASCII 48-57).
'/' (ASCII 47) is a dummy zero-width glyph to fill the contiguous range.

ActiveLook font binary layout (format 0x02, section 5.9):
  Byte  0     : 0x02  (format ID)
  Byte  1     : font_height (pixels)
  Bytes 2-3   : first_char  (u16 BE)
  Bytes 4-5   : last_char   (u16 BE)
  Bytes 6...  : offset table – num_chars × u16 BE, offset into char-data section
  Then        : char-data blocks, one per char in [first_char, last_char]

Each char-data block:
  Byte 0        : total block size (= 2 + len(rle)), max 255
  Byte 1        : char width in pixels
  Bytes 2...    : RLE-encoded pixels (rows, top-to-bottom, left-to-right)

RLE encoding:
  Non-zero byte : [7:4] = off-pixel count (0-15), [3:0] = on-pixel count (0-15)
  0x00 + next   : [7]=1 → on pixels, [7]=0 → off pixels; [6:0] × 8 pixels

Format limits (ActiveLook API section 5.9):
  Max block per glyph : 256 bytes  (size byte is u8, so max value = 255)
  Max total font size : 8192 bytes
"""

import os
import struct
from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
THRESHOLD   = 128
FIRST_CHAR  = ord(' ')   # 32
LAST_CHAR   = ord('9')   # 57
REAL_CHARS  = set(' .0123456789')

MAX_GLYPH_BYTES = 255   # total block including size + width bytes
MAX_FONT_BYTES  = 8192

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

FONTS = {
    'inconsolata_bold': os.path.join(BASE_DIR, 'Inconsolata/static/Inconsolata/Inconsolata-Bold.ttf'),
    'noto_sans'       : os.path.join(BASE_DIR, 'Noto_Sans/NotoSans-Regular.ttf'),
}

# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
ANCHOR_Y = 300   # baseline anchor on the scratch canvas

def compute_cell(font):
    """
    Find the vertical cell (ascent, descent in pixels) from actual rendered glyphs.
    Uses anchor='ls' so ANCHOR_Y is the baseline position.
    """
    max_above = 0
    max_below = 0

    for code in range(FIRST_CHAR, LAST_CHAR + 1):
        ch = chr(code)
        if ch not in REAL_CHARS:
            continue
        img = Image.new('L', (800, 600), 0)
        draw = ImageDraw.Draw(img)
        draw.text((100, ANCHOR_Y), ch, fill=255, font=font, anchor='ls')
        bb = img.getbbox()
        if bb:
            max_above = max(max_above, ANCHOR_Y - bb[1])
            max_below = max(max_below, bb[3] - ANCHOR_Y)

    return max_above, max_below  # ascent, descent


def render_glyph(font, ch, ascent, descent):
    """
    Render character as a binary pixel list (1=on, 0=off).
    Height = ascent + descent, width = tight bounding box of the glyph.
    Baseline is at row index `ascent` within the cell.
    Returns (width, pixels).
    """
    cell_h = ascent + descent

    img = Image.new('L', (800, 600), 0)
    draw = ImageDraw.Draw(img)
    draw.text((100, ANCHOR_Y), ch, fill=255, font=font, anchor='ls')

    bb = img.getbbox()
    if bb is None:
        # Invisible glyph (e.g. space) — use advance width from font metrics
        try:
            adv = round(font.getlength(ch))
        except AttributeError:
            adv = 4
        w = max(1, adv)
        return w, [0] * (w * cell_h)

    w = bb[2] - bb[0]
    crop = img.crop((bb[0], ANCHOR_Y - ascent, bb[2], ANCHOR_Y - ascent + cell_h))
    pixels = [1 if p >= THRESHOLD else 0 for p in crop.getdata()]
    return w, pixels


# ---------------------------------------------------------------------------
# RLE encoder
# ---------------------------------------------------------------------------
def rle_encode(pixels):
    """Encode a flat (off=0, on=1) pixel list to ActiveLook RLE bytes."""
    out = bytearray()
    i, n = 0, len(pixels)

    while i < n:
        off = 0
        while i + off < n and pixels[i + off] == 0:
            off += 1
        on = 0
        j = i + off
        while j + on < n and pixels[j + on] == 1:
            on += 1

        off_rem, on_rem = off, on

        # Extended encoding for long off runs (multiples of 8)
        while off_rem >= 8:
            blocks = min(off_rem // 8, 127)
            out.append(0x00)
            out.append(blocks & 0x7F)       # bit7=0 → off pixels
            off_rem -= blocks * 8

        # Extended encoding for long on runs (only when no pending off remainder)
        if off_rem == 0:
            while on_rem >= 8:
                blocks = min(on_rem // 8, 127)
                out.append(0x00)
                out.append(0x80 | blocks)   # bit7=1 → on pixels
                on_rem -= blocks * 8

        # Emit remaining small counts with nibble pairs
        while off_rem > 0 or on_rem > 0:
            oc = min(off_rem, 15)
            nc = min(on_rem,  15)
            out.append((oc << 4) | nc)
            off_rem -= oc
            on_rem  -= nc

        i += off + on

    return bytes(out)


# ---------------------------------------------------------------------------
# Font builder
# ---------------------------------------------------------------------------
def build_font(font, ascent, descent):
    """
    Assemble the complete ActiveLook binary font blob.
    Raises ValueError if any glyph exceeds MAX_GLYPH_BYTES or
    the total font exceeds MAX_FONT_BYTES.
    """
    cell_h = ascent + descent

    blocks = {}
    for code in range(FIRST_CHAR, LAST_CHAR + 1):
        ch = chr(code)
        if ch in REAL_CHARS:
            w, px = render_glyph(font, ch, ascent, descent)
        else:
            w, px = 1, [0] * cell_h

        rle  = rle_encode(px)
        size = 2 + len(rle)
        if size > MAX_GLYPH_BYTES:
            raise ValueError(
                f"glyph '{chr(code)}' needs {size} bytes (max {MAX_GLYPH_BYTES})"
            )
        blocks[code] = bytes([size, w]) + rle

    offsets   = []
    char_data = bytearray()
    offset    = 0
    for code in range(FIRST_CHAR, LAST_CHAR + 1):
        offsets.append(offset)
        char_data.extend(blocks[code])
        offset += len(blocks[code])

    header       = struct.pack('>BBHH', 0x02, cell_h, FIRST_CHAR, LAST_CHAR)
    offset_table = b''.join(struct.pack('>H', o) for o in offsets)
    data         = header + offset_table + bytes(char_data)

    if len(data) > MAX_FONT_BYTES:
        raise ValueError(
            f"font is {len(data)} bytes (max {MAX_FONT_BYTES})"
        )

    return data


# ---------------------------------------------------------------------------
# Max-size finder
# ---------------------------------------------------------------------------
def find_max_size(font_path, lo=10, hi=250):
    """
    Binary-search the largest point size at which all glyphs fit within
    MAX_GLYPH_BYTES and the total font fits within MAX_FONT_BYTES.
    """
    best = lo
    while lo <= hi:
        mid = (lo + hi) // 2
        try:
            font = ImageFont.truetype(font_path, mid)
            ascent, descent = compute_cell(font)
            build_font(font, ascent, descent)
            best = mid
            lo   = mid + 1
        except ValueError:
            hi = mid - 1
    return best


# ---------------------------------------------------------------------------
# C header writer
# ---------------------------------------------------------------------------
def write_header(data, var_name, path, font_label, font_size, cell_h):
    guard = os.path.basename(path).upper().replace('.', '_').replace('-', '_')
    with open(path, 'w') as f:
        f.write(f'#ifndef {guard}\n')
        f.write(f'#define {guard}\n\n')
        f.write(f'#include <stdint.h>\n\n')
        f.write(
            f'/*\n'
            f' * ActiveLook font  : {font_label}\n'
            f' * Point size       : {font_size}pt  →  {cell_h}px cell height\n'
            f' * Characters       : . 0-9  (ASCII {FIRST_CHAR}–{LAST_CHAR})\n'
            f' * Total size       : {len(data)} bytes\n'
            f' *\n'
            f' * Send with fontSave command (opcode 0x51):\n'
            f' *   1st frame  : 0xFF 0x51 0x00 0x08 <font_id> <size_hi> <size_lo> 0xAA\n'
            f' *   data frames: 0xFF 0x51 0x00 <len> <bytes...> 0xAA  (max 242 data bytes each)\n'
            f' */\n\n'
        )
        f.write(f'#define {var_name.upper()}_SIZE {len(data)}u\n\n')
        f.write(f'static const uint8_t {var_name}[{len(data)}] = {{\n')
        for i in range(0, len(data), 16):
            chunk = data[i:i+16]
            row   = ', '.join(f'0x{b:02X}' for b in chunk)
            comma = ',' if i + 16 < len(data) else ''
            f.write(f'    {row}{comma}\n')
        f.write(f'}};\n\n')
        f.write(f'#endif /* {guard} */\n')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    for base_name, font_path in FONTS.items():
        print(f'\n{base_name}')
        print(f'  Font  : {font_path}')
        print(f'  Searching for maximum point size...')

        pt = find_max_size(font_path)
        var_name = f'{base_name}_{pt}'

        font = ImageFont.truetype(font_path, pt)
        ascent, descent = compute_cell(font)
        cell_h = ascent + descent
        print(f'  Max pt: {pt}pt  →  cell {ascent}+{descent}={cell_h}px')

        data = build_font(font, ascent, descent)
        print(f'  Size  : {len(data)} bytes')

        out_path   = os.path.join(BASE_DIR, f'{var_name}.h')
        font_label = os.path.basename(font_path)
        write_header(data, var_name, out_path, font_label, pt, cell_h)
        print(f'  → {out_path}')


if __name__ == '__main__':
    main()
