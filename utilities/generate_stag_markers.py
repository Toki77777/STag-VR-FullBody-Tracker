#!/usr/bin/env python3
"""Generate printable STag marker images and print sheets.

This is a Python port of the official STag marker generator
(a C# WinForms tool, ``ref/marker_generator/Form1.cs`` in
https://github.com/ManfredStoiber/stag). It reproduces that tool's
rendering algorithm using numpy/OpenCV/Pillow so markers can be generated
on any platform without .NET, and adds two conveniences the original tool
did not have: a plain CLI and a "sheet" mode that lays markers out on a
multi-page, physically-scaled PDF ready to print.

Marker anatomy (all ratios below are relative to the marker edge = 1):

* a black square border (``border`` = 0.125 of the marker edge)
* a white circle (``outerCircleRadius`` = 0.4)
* 48 code bit locations arranged in 4 rotationally-symmetric groups of 12,
  inside an inner circle (``innerCircleRadius`` = 0.35). Each bit is
  rendered as a small black (1) or white (0) circle; "filler" circles are
  added between adjacent 1-bits to make the code more robust to blur.

The bit layout, and the constants used below (border/circle ratios, code
bit locations, filler rule, smoothing algorithm, text placement) are taken
directly from Form1.cs so the printed markers match what the reference
tool would produce, and so they decode to the requested id with the
``stag`` detector.

Usage examples
--------------

Render markers 0-134 (HD11, the default) as individual 1000 px PNGs::

    python3 generate_stag_markers.py --range 0 135 --out out_dir

Render a specific set of ids::

    python3 generate_stag_markers.py --hd 15 --ids 0 1 2 --out out_dir

Build a print-ready A4 sheet with an explicit id layout (2 columns x 3
rows per page -- the default grid shape -- new page every 6 ids, ``-``
for an empty slot)::

    python3 generate_stag_markers.py --sheet a4 --marker-size-mm 93.0 \\
        --sheet-ids 0 1 45 46 90 91 --sheet-out sheet.pdf

The sheet grid shape is configurable with ``--sheet-cols``/``--sheet-rows``
(page capacity = cols * rows ids, new page started every cols * rows ids).
For example, a 1 column x 2 row per page layout with explicit tile
centers::

    python3 generate_stag_markers.py --sheet a4 --marker-size-mm 93.0 \\
        --sheet-cols 1 --sheet-rows 2 \\
        --centers-x-mm 105 --centers-y-mm 74.25 222.75 \\
        --sheet-ids 0 1 45 46 90 91 --sheet-out sheet.pdf
"""

import argparse
import math
import os
import sys

import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------------------
# Constants ported from Form1.cs (values are ratios where the marker edge
# length is 1, unless noted otherwise).
# ---------------------------------------------------------------------------

NO_OF_BITS = 48

BORDER_RATIO = 0.125                       # border
OUTER_CIRCLE_RADIUS_RATIO = 0.4            # outerCircleRadius
INNER_CIRCLE_RADIUS_RATIO = 0.35           # innerCircleRadius
CODE_RADIUS_RATIO = 0.062482177287080      # codeRadius (ratio to innerCircleRadius)
FILLER_RADIUS_RATIO = 0.7                  # fillerCodeRadius

FILE_SIZE = 1000                           # fileSize, in pixels

MARKER_SIZE = FILE_SIZE / (1 + BORDER_RATIO * 2)          # 800  (the black square edge)
BORDER_SIZE = MARKER_SIZE * BORDER_RATIO                  # 100

OUTER_CIRCLE_DIAM = 2 * MARKER_SIZE * OUTER_CIRCLE_RADIUS_RATIO   # 640
INNER_CIRCLE_DIAM = 2 * MARKER_SIZE * INNER_CIRCLE_RADIUS_RATIO   # 560
OUTER_CIRCLE_TOPLEFT = (FILE_SIZE - OUTER_CIRCLE_DIAM) / 2        # 180
INNER_CIRCLE_TOPLEFT = (FILE_SIZE - INNER_CIRCLE_DIAM) / 2        # 220
CODE_CIRCLE_DIAM = 2 * INNER_CIRCLE_DIAM * CODE_RADIUS_RATIO      # ~69.98
FILLER_CIRCLE_DIAM = CODE_CIRCLE_DIAM * FILLER_RADIUS_RATIO

DEFAULT_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
LARGE_FONT_SIZE = 72
SMALL_FONT_SIZE = 20

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CODEBOOK_DIR = os.path.join(SCRIPT_DIR, "stag-codebooks")


# ---------------------------------------------------------------------------
# Code bit geometry (Form1.cs: fillLocs()).
# ---------------------------------------------------------------------------

def _polar_to_cart(radius, radians):
    """Port of Form1.cs polarToCart(): returns normalized (x, y) in [0, 1]."""
    return (0.5 + math.cos(radians) * radius, 0.5 - math.sin(radians) * radius)


def _fill_locs():
    """Port of Form1.cs fillLocs(): the 48 code bit locations.

    Locations are normalized to the inner circle's bounding box (i.e. in
    units where innerCircleDiameterSize == 1), 12 locations per rotation
    group, 4 groups spaced 90 degrees apart.
    """
    locs = []
    for i in range(4):
        off = i * (math.pi / 2)
        locs.append(_polar_to_cart(0.088363142525988, 0.785398163397448 + off))
        locs.append(_polar_to_cart(0.206935928182607, 0.459275804122858 + off))
        locs.append(_polar_to_cart(0.206935928182607, (math.pi / 2) - 0.459275804122858 + off))
        locs.append(_polar_to_cart(0.313672146827381, 0.200579720495241 + off))
        locs.append(_polar_to_cart(0.327493143484516, 0.591687617505840 + off))
        locs.append(_polar_to_cart(0.327493143484516, (math.pi / 2) - 0.591687617505840 + off))
        locs.append(_polar_to_cart(0.313672146827381, (math.pi / 2) - 0.200579720495241 + off))
        locs.append(_polar_to_cart(0.437421957035861, 0.145724938287167 + off))
        locs.append(_polar_to_cart(0.437226762361658, 0.433363129825345 + off))
        locs.append(_polar_to_cart(0.430628029742607, 0.785398163397448 + off))
        locs.append(_polar_to_cart(0.437226762361658, (math.pi / 2) - 0.433363129825345 + off))
        locs.append(_polar_to_cart(0.437421957035861, (math.pi / 2) - 0.145724938287167 + off))
    return locs


def _compute_nearby(locs, code_radius):
    """Port of Form1.cs fillLocs() nearbyCodes computation.

    Two code locations are "nearby" if the (normalized) distance between
    them is less than 4x the code circle radius; nearby 1-bit pairs get a
    filler circle drawn at their midpoint.
    """
    n = len(locs)
    nearby = [set() for _ in range(n)]
    thresh = code_radius * 4
    for i in range(n):
        xi, yi = locs[i]
        for j in range(n):
            if i == j:
                continue
            xj, yj = locs[j]
            if math.hypot(xi - xj, yi - yj) < thresh:
                nearby[i].add(j)
    return nearby


CODE_LOCS = _fill_locs()
NEARBY_CODES = _compute_nearby(CODE_LOCS, CODE_RADIUS_RATIO)


def _code_pixel_center(bit_index):
    x, y = CODE_LOCS[bit_index]
    px = INNER_CIRCLE_TOPLEFT + INNER_CIRCLE_DIAM * x
    py = INNER_CIRCLE_TOPLEFT + INNER_CIRCLE_DIAM * y
    return px, py


# ---------------------------------------------------------------------------
# Codebook loading.
# ---------------------------------------------------------------------------

def load_codebook(codebook_dir, hd):
    """Read HD<hd>.txt: one 48-char bitstring per line, line N = marker id N."""
    path = os.path.join(codebook_dir, "HD{}.txt".format(hd))
    codes = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if len(line) != NO_OF_BITS:
                raise ValueError(
                    "{}: expected {}-bit lines, got {} bits: {!r}".format(
                        path, NO_OF_BITS, len(line), line))
            codes.append([int(c) for c in line])
    if not codes:
        raise ValueError("{}: no codes found".format(path))
    return codes


# ---------------------------------------------------------------------------
# Rendering.
# ---------------------------------------------------------------------------

def _make_ball_kernel(radius):
    """Port of Form1.cs generateBallMask(): a circular neighborhood, minus
    the center pixel, as a 0/1 kernel usable with cv2.filter2D."""
    yy, xx = np.mgrid[-radius:radius + 1, -radius:radius + 1]
    mask = (xx * xx + yy * yy) <= radius * radius
    mask[radius, radius] = False  # exclude the center point, like the C# mask
    kernel = mask.astype(np.float32)
    return kernel, float(kernel.sum())


_BALL_KERNEL, _BALL_COUNT = _make_ball_kernel(12)


def _majority_filter_5x(is_black, region_lo, region_hi):
    """Port of Form1.cs dilateBitmap()/erodeBitmap(), called 5 times each
    in alternation ("erode, dilate" .. drawMarkers loop calls dilate then
    erode, 5 times).

    Operates on a 0/1 float array (1 = black). Each pass is a majority
    vote over a circular neighborhood of radius 12px (excluding the
    center pixel), restricted to the given square region -- this mirrors
    doing the vote with cv2.filter2D on a 0/1 image and thresholding at
    0.5 of the neighborhood size, which is algorithmically equivalent to
    the original per-pixel neighbor count and compare.
    """
    half = _BALL_COUNT / 2.0
    region_mask = np.zeros_like(is_black, dtype=bool)
    region_mask[region_lo:region_hi, region_lo:region_hi] = True

    black = is_black.astype(np.float32)
    for _ in range(5):
        # dilate: white pixels surrounded mostly by black become black
        neighbor_black = cv2.filter2D(black, -1, _BALL_KERNEL, borderType=cv2.BORDER_CONSTANT)
        cond = (black == 0) & (neighbor_black > half) & region_mask
        black = np.where(cond, np.float32(1.0), black)

        # erode: black pixels surrounded mostly by non-black become white
        neighbor_black2 = cv2.filter2D(black, -1, _BALL_KERNEL, borderType=cv2.BORDER_CONSTANT)
        white_neighbor = _BALL_COUNT - neighbor_black2
        cond2 = (black == 1) & (white_neighbor > half) & region_mask
        black = np.where(cond2, np.float32(0.0), black)

    return black


def _draw_circle_aa(img, center, radius, color, supersample=4):
    """Draw a single filled circle with (supersampled) anti-aliased edges,
    in place, on a uint8 grayscale image."""
    h, w = img.shape
    up = cv2.resize(img, (w * supersample, h * supersample), interpolation=cv2.INTER_NEAREST)
    cx, cy = center
    cv2.circle(
        up,
        (int(round(cx * supersample)), int(round(cy * supersample))),
        int(round(radius * supersample)),
        color, -1, lineType=cv2.LINE_8)
    down = cv2.resize(up, (w, h), interpolation=cv2.INTER_AREA)
    img[:, :] = down


def _redraw_code_circles_aa(img, bits, supersample=4):
    """Port of the final crisp code-circle redraw in Form1.cs drawMarkers()
    (after the majority-filter smoothing pass): every code bit location is
    redrawn as a precise, anti-aliased black (1) or white (0) circle,
    overwriting whatever the majority filter produced there. Filler
    circles are intentionally NOT redrawn here, matching the original."""
    h, w = img.shape
    up = cv2.resize(img, (w * supersample, h * supersample), interpolation=cv2.INTER_NEAREST)
    r_px = int(round((CODE_CIRCLE_DIAM / 2) * supersample))
    for j, b in enumerate(bits):
        cx, cy = _code_pixel_center(j)
        center = (int(round(cx * supersample)), int(round(cy * supersample)))
        color = 0 if b == 1 else 255
        cv2.circle(up, center, r_px, color, -1, lineType=cv2.LINE_8)
    down = cv2.resize(up, (w, h), interpolation=cv2.INTER_AREA)
    return down


def _apply_ring_wipe(img, supersample=4):
    """Port of the Form1.cs drawMarkers() ringImg logic.

    The reference builds a helper bitmap: a black rectangle, with a white
    outer circle drawn (anti-aliased) and then a *slightly enlarged*
    (+1px radius) black inner circle drawn with hard edges on top; the
    black is then made fully transparent and the result is composited
    over the marker. Net effect: pixels outside the outer circle are left
    untouched (already black there); the ring between the inner and outer
    circle is painted solid white (cleaning up any leftover smoothing
    artifacts there); and everything inside the (slightly enlarged) inner
    circle -- i.e. the actual code area -- is left untouched.

    Because GDI+'s MakeTransparent(Color.Black) only makes *exactly*
    black pixels transparent, the anti-aliased boundary of the outer
    circle (which blends to non-pure-black shades against the black
    background) stays opaque and gets composited too, softening that
    edge. We reproduce this by only ever touching pixels where our ring
    layer is non-zero.
    """
    h, w = img.shape
    ring = np.zeros((h, w), dtype=np.uint8)
    up = cv2.resize(ring, (w * supersample, h * supersample), interpolation=cv2.INTER_NEAREST)
    cv2.circle(
        up,
        (int(round(w / 2 * supersample)), int(round(h / 2 * supersample))),
        int(round((OUTER_CIRCLE_DIAM / 2) * supersample)),
        255, -1)
    ring = cv2.resize(up, (w, h), interpolation=cv2.INTER_AREA)

    # hard-edged (no AA), enlarged by 1px radius (+2px diameter), like C#'s
    # innerCircleTopLeft - 1, innerCircleDiameterSize + 2
    inner_r = INNER_CIRCLE_DIAM / 2 + 1
    cv2.circle(ring, (int(round(w / 2)), int(round(h / 2))), int(round(inner_r)), 0, -1)

    mask = ring != 0
    img[mask] = ring[mask]
    return img


def _create_index_string(index, total_count):
    """Port of Form1.cs createIndexString(): zero-pad `index` to the width
    of the codebook's total entry count (e.g. HD11 has 22308 entries, so
    ids are zero-padded to 5 digits: "00000", "00001", ...)."""
    width = len(str(total_count))
    return str(index).zfill(width)


def _load_font(font_path, font_size):
    """Load the label font, falling back to fonts commonly available on
    Linux (DejaVu) and Windows (Arial) when `font_path` does not exist."""
    for candidate in (font_path, "DejaVuSans-Bold.ttf", "arialbd.ttf", "arial.ttf"):
        if not candidate:
            continue
        try:
            return ImageFont.truetype(candidate, font_size)
        except OSError:
            continue
    raise SystemExit("could not load a .ttf font for the marker id labels, "
                     "pass an existing font file with --font")


def _text_layer(text, font_path, font_size):
    """Render `text` in white on a transparent-black layer, sized exactly
    to the text, for later rotation/compositing."""
    font = _load_font(font_path, font_size)
    tmp = Image.new("L", (4, 4))
    bbox = ImageDraw.Draw(tmp).textbbox((0, 0), text, font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    layer = Image.new("L", (max(tw, 1), max(th, 1)), 0)
    ImageDraw.Draw(layer).text((-bbox[0], -bbox[1]), text, font=font, fill=255)
    return layer


def _draw_rotated_text(img, text, center_xy, font_path, font_size, rotation_deg=45):
    """Draw `text` in white, centered at `center_xy` (absolute pixel
    coordinates), rotated `rotation_deg` counterclockwise about its own
    center -- ports the g.TranslateTransform/RotateTransform(315)/
    DrawString sequence in Form1.cs drawMarkers()."""
    layer = _text_layer(text, font_path, font_size)
    rotated = layer.rotate(rotation_deg, resample=Image.BICUBIC, expand=True)
    rw, rh = rotated.size
    cx, cy = center_xy
    left = int(round(cx - rw / 2))
    top = int(round(cy - rh / 2))

    img_pil = Image.fromarray(img)
    white_layer = Image.new("L", (rw, rh), 255)
    img_pil.paste(white_layer, (left, top), rotated)
    return np.array(img_pil)


def render_marker(bits, hd, total_count, index, font_path=DEFAULT_FONT):
    """Render one STag marker to a 1000x1000 uint8 grayscale image.

    `bits` is a list of 48 ints (0/1), `hd` is the codebook's Hamming
    distance (used only for the "HD<n>" label text), `total_count` is the
    number of entries in that codebook (used for zero-padding the id
    label), and `index` is this marker's id.
    """
    img = np.full((FILE_SIZE, FILE_SIZE), 255, dtype=np.uint8)

    # black square (hard edge, no AA -- matches SmoothingMode.None)
    x0 = int(round(BORDER_SIZE))
    edge = int(round(MARKER_SIZE))
    cv2.rectangle(img, (x0, x0), (x0 + edge - 1, x0 + edge - 1), 0, -1)

    # white outer circle (anti-aliased -- matches SmoothingMode.AntiAlias)
    _draw_circle_aa(img, (FILE_SIZE / 2, FILE_SIZE / 2), OUTER_CIRCLE_DIAM / 2, 255)

    # initial code circles + filler circles (hard edge, no AA)
    r_code = int(round(CODE_CIRCLE_DIAM / 2))
    for j, b in enumerate(bits):
        if b == 1:
            cx, cy = _code_pixel_center(j)
            cv2.circle(img, (int(round(cx)), int(round(cy))), r_code, 0, -1)

    r_filler = int(round(FILLER_CIRCLE_DIAM / 2))
    for j in range(NO_OF_BITS):
        if bits[j] != 1:
            continue
        for k in NEARBY_CODES[j]:
            if k <= j or bits[k] != 1:
                continue
            mx = (CODE_LOCS[j][0] + CODE_LOCS[k][0]) / 2
            my = (CODE_LOCS[j][1] + CODE_LOCS[k][1]) / 2
            px = INNER_CIRCLE_TOPLEFT + INNER_CIRCLE_DIAM * mx
            py = INNER_CIRCLE_TOPLEFT + INNER_CIRCLE_DIAM * my
            cv2.circle(img, (int(round(px)), int(round(py))), r_filler, 0, -1)

    for j, b in enumerate(bits):
        if b == 0:
            cx, cy = _code_pixel_center(j)
            cv2.circle(img, (int(round(cx)), int(round(cy))), r_code, 255, -1)

    # 5x (dilate, erode) majority-vote smoothing pass
    is_black = (img < 128).astype(np.float32)
    black = _majority_filter_5x(is_black, int(OUTER_CIRCLE_TOPLEFT), FILE_SIZE - int(OUTER_CIRCLE_TOPLEFT))
    img = np.where(black > 0.5, np.uint8(0), np.uint8(255))

    # redraw crisp, anti-aliased code circles on top
    img = _redraw_code_circles_aa(img, bits)

    # clean up the ring between the inner and outer circle
    img = _apply_ring_wipe(img)

    # id + "HD<n>" text, white, rotated 45 degrees CCW, in the top-left border
    id_str = _create_index_string(index, total_count)
    img = _draw_rotated_text(img, id_str, (2.375 * BORDER_SIZE, 2.375 * BORDER_SIZE), font_path, LARGE_FONT_SIZE)
    img = _draw_rotated_text(img, "HD{}".format(hd), (1.875 * BORDER_SIZE, 1.875 * BORDER_SIZE), font_path, SMALL_FONT_SIZE)

    return img


# ---------------------------------------------------------------------------
# Sheet (print) layout.
# ---------------------------------------------------------------------------

MM_PER_INCH = 25.4

PAGE_SIZES_MM = {
    "a4": (210.0, 297.0),
}

# Default sheet grid shape (used when --sheet-cols/--sheet-rows are not given).
SHEET_COLS_DEFAULT = 2
SHEET_ROWS_DEFAULT = 3


def _mm_to_px(mm, dpi):
    return int(round(mm / MM_PER_INCH * dpi))


def _default_centers(page_w_mm, page_h_mm, tile_mm, cols, rows):
    """A simple, symmetric fallback grid: evenly spaced tiles centered on
    the page, used when explicit --centers-x-mm/--centers-y-mm are not
    given."""
    xs = [(c + 0.5) * page_w_mm / cols for c in range(cols)]
    ys = [(r + 0.5) * page_h_mm / rows for r in range(rows)]
    return xs, ys


def build_sheet_pages(ids, marker_size_mm, page_w_mm, page_h_mm, dpi,
                       centers_x_mm=None, centers_y_mm=None,
                       codebook_dir=DEFAULT_CODEBOOK_DIR, hd=11,
                       font_path=DEFAULT_FONT,
                       sheet_cols=SHEET_COLS_DEFAULT, sheet_rows=SHEET_ROWS_DEFAULT):
    """Render `ids` (a flat list where the string "-" means "leave this
    slot blank") onto one or more sheet pages, `sheet_cols` columns x
    `sheet_rows` rows per page (row-major), a new page started every
    ``sheet_cols * sheet_rows`` ids. Returns a list of PIL Images, one per
    page."""
    codes = load_codebook(codebook_dir, hd)
    total_count = len(codes)

    per_page = sheet_cols * sheet_rows

    tile_mm = marker_size_mm * (1 + 2 * BORDER_RATIO)  # full tile incl. white margin
    tile_px = _mm_to_px(tile_mm, dpi)

    # Sanity-check that the white margin ratio (12.5% of the black square,
    # each side) survives the mm -> px rounding used for the sheet tile.
    marker_px = _mm_to_px(marker_size_mm, dpi)
    border_px = (tile_px - marker_px) / 2.0
    if abs(border_px / marker_px - BORDER_RATIO) > 0.01:
        raise AssertionError(
            "sheet tile white margin ratio drifted: expected {:.4f}, got {:.4f}".format(
                BORDER_RATIO, border_px / marker_px))

    if centers_x_mm is None:
        centers_x_mm, centers_y_mm = _default_centers(page_w_mm, page_h_mm, tile_mm, sheet_cols, sheet_rows)
    if len(centers_x_mm) != sheet_cols:
        raise ValueError("expected {} x-centers, got {}".format(sheet_cols, len(centers_x_mm)))
    if len(centers_y_mm) != sheet_rows:
        raise ValueError("expected {} y-centers, got {}".format(sheet_rows, len(centers_y_mm)))

    page_w_px = _mm_to_px(page_w_mm, dpi)
    page_h_px = _mm_to_px(page_h_mm, dpi)

    # pre-render each distinct marker used, once
    cache = {}

    def get_tile(marker_id):
        if marker_id not in cache:
            bits = codes[marker_id]
            img = render_marker(bits, hd, total_count, marker_id, font_path=font_path)
            # The base render is FILE_SIZE (1000px) square; resample to the
            # physical tile size. Shrinking (smaller sheets/lower DPI) wants
            # an area-average to avoid aliasing, while enlarging (e.g. large
            # markers at 300 DPI) wants a smooth upscale instead of the
            # blocky/aliased result INTER_AREA gives when upsampling.
            if tile_px < FILE_SIZE:
                interp = cv2.INTER_AREA
            elif tile_px > FILE_SIZE:
                interp = cv2.INTER_CUBIC
            else:
                interp = cv2.INTER_AREA
            tile = cv2.resize(img, (tile_px, tile_px), interpolation=interp)
            cache[marker_id] = tile
        return cache[marker_id]

    pages = []
    for page_start in range(0, len(ids), per_page):
        page_ids = ids[page_start:page_start + per_page]
        page_ids = page_ids + ["-"] * (per_page - len(page_ids))

        page = np.full((page_h_px, page_w_px), 255, dtype=np.uint8)
        for slot, marker_id in enumerate(page_ids):
            if marker_id in ("-", None):
                continue
            row, col = divmod(slot, sheet_cols)
            tile = get_tile(int(marker_id))
            cx_px = _mm_to_px(centers_x_mm[col], dpi)
            cy_px = _mm_to_px(centers_y_mm[row], dpi)
            half = tile_px // 2
            y0, x0 = cy_px - half, cx_px - half
            y1, x1 = y0 + tile_px, x0 + tile_px
            # clip to page bounds, just in case
            py0, px0 = max(y0, 0), max(x0, 0)
            py1, px1 = min(y1, page_h_px), min(x1, page_w_px)
            if py1 <= py0 or px1 <= px0:
                continue
            page[py0:py1, px0:px1] = tile[py0 - y0:py1 - y0, px0 - x0:px1 - x0]

        pages.append(Image.fromarray(page, mode="L"))

    return pages


def save_pdf(pages, out_path, dpi):
    if not pages:
        raise ValueError("no pages to save")
    pages[0].save(out_path, save_all=True, append_images=pages[1:], resolution=float(dpi))


# ---------------------------------------------------------------------------
# CLI.
# ---------------------------------------------------------------------------

def _parse_ids_arg(values):
    ids = []
    for v in values:
        if v == "-":
            ids.append("-")
        else:
            ids.append(int(v))
    return ids


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--hd", type=int, default=11, choices=[11, 13, 15, 17, 19, 21, 23],
                    help="STag codebook Hamming distance to use (default: 11)")
    p.add_argument("--codebook-dir", default=DEFAULT_CODEBOOK_DIR,
                    help="directory containing HD<n>.txt codebooks (default: %(default)s)")
    p.add_argument("--font", default=DEFAULT_FONT, help="path to a .ttf font for the id/HD labels")

    g = p.add_mutually_exclusive_group()
    g.add_argument("--ids", nargs="+", metavar="ID", help="explicit list of marker ids to render as individual PNGs")
    g.add_argument("--range", nargs=2, type=int, metavar=("START", "END"),
                   help="render ids in [START, END) as individual PNGs")

    p.add_argument("--out", metavar="DIR", help="output directory for individual marker PNGs")

    p.add_argument("--sheet", choices=sorted(PAGE_SIZES_MM), help="build a multi-marker print PDF for this page size")
    p.add_argument("--marker-size-mm", type=float, help="black square edge length, in mm (sheet mode)")
    p.add_argument("--sheet-ids", nargs="+", metavar="ID",
                    help="ids to lay out row-major, --sheet-cols x --sheet-rows per page, new page "
                         "every (cols * rows) ids ('-' leaves a slot blank)")
    p.add_argument("--sheet-out", metavar="FILE", help="output PDF path (sheet mode)")
    p.add_argument("--dpi", type=float, default=300.0, help="sheet render DPI (default: %(default)s)")
    p.add_argument("--sheet-cols", type=int, default=SHEET_COLS_DEFAULT,
                    help="sheet grid columns per page (default: %(default)s)")
    p.add_argument("--sheet-rows", type=int, default=SHEET_ROWS_DEFAULT,
                    help="sheet grid rows per page (default: %(default)s)")
    p.add_argument("--centers-x-mm", nargs="+", type=float, metavar="X",
                    help="explicit tile-center x positions in mm, one per column, "
                         "i.e. --sheet-cols values (default: evenly spaced)")
    p.add_argument("--centers-y-mm", nargs="+", type=float, metavar="Y",
                    help="explicit tile-center y positions in mm, one per row, "
                         "i.e. --sheet-rows values (default: evenly spaced)")

    args = p.parse_args(argv)

    if args.sheet:
        if not args.marker_size_mm or not args.sheet_ids or not args.sheet_out:
            p.error("--sheet requires --marker-size-mm, --sheet-ids and --sheet-out")
        if args.sheet_cols < 1 or args.sheet_rows < 1:
            p.error("--sheet-cols/--sheet-rows must be >= 1")
        if args.centers_x_mm is not None and len(args.centers_x_mm) != args.sheet_cols:
            p.error("--centers-x-mm expects {} value(s) (one per --sheet-cols), got {}".format(
                args.sheet_cols, len(args.centers_x_mm)))
        if args.centers_y_mm is not None and len(args.centers_y_mm) != args.sheet_rows:
            p.error("--centers-y-mm expects {} value(s) (one per --sheet-rows), got {}".format(
                args.sheet_rows, len(args.centers_y_mm)))
        page_w_mm, page_h_mm = PAGE_SIZES_MM[args.sheet]
        ids = _parse_ids_arg(args.sheet_ids)
        pages = build_sheet_pages(
            ids, args.marker_size_mm, page_w_mm, page_h_mm, args.dpi,
            centers_x_mm=args.centers_x_mm, centers_y_mm=args.centers_y_mm,
            codebook_dir=args.codebook_dir, hd=args.hd, font_path=args.font,
            sheet_cols=args.sheet_cols, sheet_rows=args.sheet_rows)
        save_pdf(pages, args.sheet_out, args.dpi)
        print("wrote {} page(s) to {}".format(len(pages), args.sheet_out))
        return 0

    if not args.out:
        p.error("--out DIR is required (unless using --sheet)")
    if args.ids:
        ids = [int(v) for v in args.ids]
    elif args.range:
        ids = list(range(args.range[0], args.range[1]))
    else:
        p.error("one of --ids or --range is required (unless using --sheet)")

    codes = load_codebook(args.codebook_dir, args.hd)
    total_count = len(codes)
    os.makedirs(args.out, exist_ok=True)

    for marker_id in ids:
        if marker_id < 0 or marker_id >= total_count:
            raise ValueError("id {} out of range for HD{} (0..{})".format(marker_id, args.hd, total_count - 1))
        bits = codes[marker_id]
        img = render_marker(bits, args.hd, total_count, marker_id, font_path=args.font)
        id_str = _create_index_string(marker_id, total_count)
        out_path = os.path.join(args.out, "{}.png".format(id_str))
        cv2.imwrite(out_path, img)
        print("wrote {}".format(out_path))

    return 0


if __name__ == "__main__":
    sys.exit(main())
