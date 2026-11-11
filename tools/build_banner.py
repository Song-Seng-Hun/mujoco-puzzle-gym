#!/usr/bin/env python3
"""Draw the animated pixel-art banner at the top of the READMEs.

    python3 tools/build_banner.py

writes assets/banner-dark.svg and assets/banner-light.svg: Claude types a room.yaml at a terminal,
and each line adds a piece to a top-view puzzle room (walls with an opening, a floor switch, a
door, a goal and the link between them). After the check passes, a small duck robot walks to the
switch, the door slides open and the duck walks through to the goal; the terminal prints the
success rate and Claude nods. The pixel helpers, palettes, font and the Claude sprite are copied
from the llm_image_lens and llm_music_lens banners (which take them from the profile map,
Song-Seng-Hun/Song-Seng-Hun tools/build_worldmap.py), so the banners look alike; the digits 7-9
are added to the font. 160x90 grid, 4 px per cell, SMIL animation (GitHub plays it in README
images). Standard library only.
"""

import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "assets"

P = 4                 # px per grid cell
W, H = 160, 90        # grid size

# ---------------------------------------------------------------- palettes (from build_worldmap.py)

COMMON = {
    "skin": "#F1C9A5", "hair": "#3B2A22", "glass": "#1A1A2A",
    "coat": "#4A6FA5", "coat_d": "#38598A", "pants": "#2E3A55", "scarf": "#E8C468",
    "orange": "#D97757", "yellow": "#E3B341", "teal": "#4FA3A5",
    "green": "#6DBE73", "blue": "#5E8FD6", "red": "#E5566B",
    "white": "#F7F5EE", "paper": "#FFFDF6", "ink": "#4A5070",
    "flower1": "#E8A0B4", "flower2": "#E8D27A",
}

PALETTES = {
    "dark": {
        **COMMON,
        "K": "#0B0E1C", "frame": "#B89B5E", "frame_d": "#5E4C2C",
        "sky0": "#0C1026", "sky1": "#111735", "sky2": "#171F45", "sky3": "#1D2855", "sky4": "#253366",
        "star": "#F4EBD9", "star_d": "#7F88B5", "cloud": "#2A3566", "cloud_d": "#222C58",
        "mtn_far": "#1A2350", "g0": "#1F3B35", "g1": "#264840", "g2": "#19312C", "tuft": "#2F5A4D",
        "path": "#5C4B3F", "path_e": "#3F332B", "path_h": "#6E5A4B",
        "wall": "#8B8FA8", "wall_m": "#6C7192", "wall_d": "#4C5172",
        "roof": "#7B4A57", "roof_d": "#5A3442", "wood": "#6B4E3A", "wood_d": "#4A3527",
        "metal": "#9AA3B5", "metal_d": "#5F6880", "win": "#262E4D",
        "lit": "#FFD27A", "glow": ("#FFB85C", 0.32), "smoke": ("#9AA3B5", 0.7),
        "text": "#F4EBD9", "sub": "#8E96C2", "shadow": "#0B0E1C",
    },
    "light": {
        **COMMON,
        "K": "#2B2F45", "frame": "#C9A86A", "frame_d": "#8A6E3E",
        "sky0": "#B9D8EA", "sky1": "#C6E0EE", "sky2": "#D2E7F0", "sky3": "#DDEDF1", "sky4": "#E8F2EF",
        "star": "#FFFFFF", "star_d": "#FFFFFF", "cloud": "#FFFFFF", "cloud_d": "#E3ECF2",
        "mtn_far": "#A9C2D4", "g0": "#B4CE98", "g1": "#C1D8A6", "g2": "#A2BE87", "tuft": "#8CB072",
        "path": "#E2D2AE", "path_e": "#C9B48C", "path_h": "#EEE2C6",
        "wall": "#F2EEE6", "wall_m": "#DAD3C6", "wall_d": "#B9B0A1",
        "roof": "#C97E6C", "roof_d": "#A8655A", "wood": "#A47C5B", "wood_d": "#86624A",
        "metal": "#C6CDD6", "metal_d": "#8E97A6", "win": "#7F95AA",
        "lit": "#FFE08A", "glow": ("#FFC94A", 0.38), "smoke": ("#8E97A6", 0.55),
        "text": "#2B2F45", "sub": "#6A7192", "shadow": "#F7F3E8",
    },
}

# sprite characters -> palette keys
CHARS = {
    "K": "K", "h": "hair", "f": "skin", "G": "glass", "c": "coat", "C": "coat_d",
    "n": "pants", "s": "scarf", "o": "orange", "w": "white", "p": "paper", "i": "ink",
    "y": "yellow", "m": "metal", "M": "metal_d", "l": "lit", "r": "red", "g": "green",
    "W": "wood_d", "v": "wood", "R": "roof", "x": "text", "u": "sub",
}

# ---------------------------------------------------------------- canvas (from build_worldmap.py)


class Canvas:
    def __init__(self):
        self.px = {}

    def set(self, x, y, col):
        self.px[(x, y)] = col

    def rect(self, x, y, w, h, col):
        for j in range(y, y + h):
            for i in range(x, x + w):
                self.px[(i, j)] = col

    def box(self, x, y, w, h, fill, edge="K"):
        self.rect(x, y, w, h, edge)
        if w > 2 and h > 2:
            self.rect(x + 1, y + 1, w - 2, h - 2, fill)

    def sprite(self, rows, x, y):
        for j, row in enumerate(rows):
            for i, ch in enumerate(row):
                if ch != ".":
                    self.px[(x + i, y + j)] = CHARS[ch]

    def merge(self, other):
        self.px.update(other.px)


def outlined(c, col="K"):
    out = Canvas()
    for (x, y) in c.px:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if (x + dx, y + dy) not in c.px:
                out.px[(x + dx, y + dy)] = col
    out.px.update(c.px)
    return out


def rects(cells):
    """Cover a set of cells with few rectangles (row runs merged downwards)."""
    rows = {}
    for x, y in cells:
        rows.setdefault(y, []).append(x)
    out, open_ = [], {}
    for y in sorted(rows):
        xs = sorted(rows[y])
        runs, start, prev = [], xs[0], xs[0]
        for x in xs[1:]:
            if x != prev + 1:
                runs.append((start, prev - start + 1))
                start = x
            prev = x
        runs.append((start, prev - start + 1))
        nxt = {}
        for key in runs:
            if key in open_ and sum(open_[key]) == y:
                y0, h = open_.pop(key)
                nxt[key] = (y0, h + 1)
            else:
                nxt[key] = (y, 1)
        out += [(k[0], y0, k[1], h) for k, (y0, h) in open_.items()]
        open_ = nxt
    out += [(k[0], y0, k[1], h) for k, (y0, h) in open_.items()]
    return out


def svg_paths(c, pal, unit=P, ox=0, oy=0):
    by_col = {}
    for xy, col in c.px.items():
        by_col.setdefault(col, set()).add(xy)
    out = []
    for col in sorted(by_col):
        d = "".join(f"M{x*unit+ox} {y*unit+oy}h{w*unit}v{h*unit}h{-w*unit}z"
                    for x, y, w, h in sorted(rects(by_col[col]), key=lambda r: (r[1], r[0])))
        fill = pal[col]
        if isinstance(fill, tuple):
            out.append(f'<path fill="{fill[0]}" fill-opacity="{fill[1]}" d="{d}"/>')
        else:
            out.append(f'<path fill="{fill}" d="{d}"/>')
    return "".join(out)

# ---------------------------------------------------------------- animation helpers (from build_worldmap.py)

T = 12.0              # loop length in seconds
DT = 1 / 16           # timeline step in seconds
TICKS = [k * DT for k in range(int(T / DT))]
FADE = T - 1.0        # lights go out and everything resets from here


def _kt(t):
    return f"{t / T:.4f}".rstrip("0").rstrip(".")


def _steps(changes):
    ts, vs = [], []
    for t, v in changes:
        if vs and vs[-1] == v:
            continue
        if ts and abs(t - ts[-1]) < 1e-9:
            vs[-1] = v
            continue
        ts.append(t)
        vs.append(v)
    ts[0] = 0.0
    return ";".join(_kt(t) or "0" for t in ts), ";".join(str(v) for v in vs)


def timeline(attr, changes):
    """Discrete animation on the shared T-second loop."""
    k, v = _steps(changes)
    if attr == "translate":
        return (f'<animateTransform attributeName="transform" type="translate" dur="{T}s" '
                f'repeatCount="indefinite" calcMode="discrete" keyTimes="{k}" values="{v}"/>')
    return (f'<animate attributeName="{attr}" dur="{T}s" repeatCount="indefinite" '
            f'calcMode="discrete" keyTimes="{k}" values="{v}"/>')


def loop(attr, dur, values, keytimes=None, begin=0.0, discrete=True, translate=False):
    """Free-running ambient animation with its own period."""
    kt = f' keyTimes="{keytimes}"' if keytimes else ""
    mode = ' calcMode="discrete"' if discrete else ""
    beg = f' begin="{begin:.2f}s"' if begin else ""
    if translate:
        return (f'<animateTransform attributeName="transform" type="translate" dur="{dur}s"'
                f'{beg} repeatCount="indefinite"{mode}{kt} values="{values}"/>')
    return f'<animate attributeName="{attr}" dur="{dur}s"{beg} repeatCount="indefinite"{mode}{kt} values="{values}"/>'


def group(inner, anim="", opacity=None, transform=None):
    attrs = ""
    if opacity is not None:
        attrs += f' opacity="{opacity}"'
    if transform:
        attrs += f' transform="{transform}"'
    return f"<g{attrs}>{inner}{anim}</g>"


def shown(inner, t_on, t_off):
    """Visible from t_on to t_off on the loop."""
    return group(inner, timeline("opacity", [(0, 0), (t_on, 1), (t_off, 0)]), opacity=0)


def sparkle(pal, x, y, t_on):
    small, big = Canvas(), Canvas()
    for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):
        small.set(x + dx, y + dy, "lit")
    for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (2, 0), (-2, 0), (0, 2), (0, -2)):
        big.set(x + dx, y + dy, "lit" if abs(dx) + abs(dy) < 2 else "white")
    return (group(svg_paths(small, pal), timeline("opacity", [(0, 0), (t_on, 1), (t_on + 0.2, 0),
                                                            (t_on + 0.45, 1), (t_on + 0.65, 0)]), opacity=0)
            + group(svg_paths(big, pal), timeline("opacity", [(0, 0), (t_on + 0.2, 1), (t_on + 0.45, 0)]),
                    opacity=0))

# ---------------------------------------------------------------- font 5x7 (from build_worldmap.py)


FONT_SRC = """
A .###. #...# #...# ##### #...# #...# #...#
B ####. #...# #...# ####. #...# #...# ####.
C .###. #...# #.... #.... #.... #...# .###.
D ####. #...# #...# #...# #...# #...# ####.
E ##### #.... #.... ####. #.... #.... #####
F ##### #.... #.... ####. #.... #.... #....
G .###. #...# #.... #.### #...# #...# .####
H #...# #...# #...# ##### #...# #...# #...#
I ### .#. .#. .#. .#. .#. ###
J ..### ...#. ...#. ...#. ...#. #..#. .##..
K #...# #..#. #.#.. ##... #.#.. #..#. #...#
L #.... #.... #.... #.... #.... #.... #####
M #...# ##.## #.#.# #.#.# #...# #...# #...#
N #...# #...# ##..# #.#.# #..## #...# #...#
O .###. #...# #...# #...# #...# #...# .###.
P ####. #...# #...# ####. #.... #.... #....
Q .###. #...# #...# #...# #.#.# #..#. .##.#
R ####. #...# #...# ####. #.#.. #..#. #...#
S .#### #.... #.... .###. ....# ....# ####.
T ##### ..#.. ..#.. ..#.. ..#.. ..#.. ..#..
U #...# #...# #...# #...# #...# #...# .###.
V #...# #...# #...# #...# #...# .#.#. ..#..
W #...# #...# #...# #.#.# #.#.# #.#.# .#.#.
X #...# #...# .#.#. ..#.. .#.#. #...# #...#
Y #...# #...# .#.#. ..#.. ..#.. ..#.. ..#..
Z ##### ....# ...#. ..#.. .#... #.... #####
0 .###. #...# #..## #.#.# ##..# #...# .###.
1 .#. ##. .#. .#. .#. .#. ###
2 .###. #...# ....# ...#. ..#.. .#... #####
3 ####. ....# ....# .###. ....# ....# ####.
4 ...#. ..##. .#.#. #..#. ##### ...#. ...#.
5 ##### #.... ####. ....# ....# #...# .###.
6 .###. #.... #.... ####. #...# #...# .###.
7 ##### ....# ...#. ..#.. .#... .#... .#...
8 .###. #...# #...# .###. #...# #...# .###.
9 .###. #...# #...# .#### ....# ....# .###.
. . . . . . . #
· . . . # . . .
- ... ... ... ### ... ... ...
"""

FONT = {" ": ["..."] * 7}
for line in FONT_SRC.strip().splitlines():
    ch, *rows = line.split()
    FONT[ch] = rows


def text_width(s):
    return sum(len(FONT[ch][0]) + 1 for ch in s) - 1


def draw_text(c, s, x, y, col, shadow="shadow"):
    for pass_col, dy in ((shadow, 1), (col, 0)):
        if pass_col is None:
            continue
        cx = x
        for ch in s:
            for j, row in enumerate(FONT[ch]):
                for i, v in enumerate(row):
                    if v == "#":
                        c.set(cx + i, y + j + dy, pass_col)
            cx += len(FONT[ch][0]) + 1
    return x + text_width(s)

# ---------------------------------------------------------------- sprites (Claude from build_worldmap.py)


# Claude: body = cols 1-11 x rows 0-7 from the body's top-left, arms beside it, legs on rows 8-9.
CLAUDE_EYES = {
    "happy": [(2, 2), (3, 3), (2, 4), (10, 2), (9, 3), (10, 4)],   # > <
    "dot": [(3, 2), (3, 3), (9, 2), (9, 3)],
    "blink": [(2, 3), (3, 3), (9, 3), (10, 3)],
    "right": [(4, 2), (4, 3), (10, 2), (10, 3)],
}
_ARM_L, _ARM_R = [(0, 3), (0, 4), (0, 5)], [(12, 3), (12, 4), (12, 5)]
_UP_L, _UP_R = [(0, -2), (0, -1), (0, 0)], [(12, -2), (12, -1), (12, 0)]
CLAUDE_ARMS = {
    "down": _ARM_L + _ARM_R,
    "up": _UP_L + _UP_R,
    "wave1": _UP_L + _ARM_R,
    "wave2": [(0, 1), (-1, 0), (-1, -1)] + _ARM_R,
}
CLAUDE_LEGS = {
    "stand": [(x, y) for x in (2, 4, 8, 10) for y in (8, 9)],
    "stepA": [(2, 8), (2, 9), (8, 8), (8, 9), (4, 8), (10, 8)],
    "stepB": [(4, 8), (4, 9), (10, 8), (10, 9), (2, 8), (8, 8)],
}


def claude_frame(eyes, arms, legs, bob):
    c = Canvas()
    ox, oy = 1, 3 - bob          # two spare rows on top for raised arms
    c.rect(ox + 1, oy, 11, 8, "orange")
    for x, y in CLAUDE_ARMS[arms] + CLAUDE_LEGS[legs]:
        c.set(ox + x, oy + y, "orange")
    for x, y in CLAUDE_EYES[eyes]:
        c.set(ox + x, oy + y, "K")
    return outlined(c)




# typing at the keyboard: the right arm reaches forward, alternating rows
CLAUDE_ARMS["typeA"] = _ARM_L + [(12, 4), (13, 4), (14, 4)]
CLAUDE_ARMS["typeB"] = _ARM_L + [(12, 5), (13, 5), (14, 5)]

# ---------------------------------------------------------------- scene

THEME = {
    "dark": {"screen": "#101528", "termtx": "#DDE3F2", "bezel": "#3B4266",
             "fl0": "#252C4D", "fl1": "#2A3255", "rwall": "#A8AFC4", "rwall_d": "#6C7192",
             "pad": "#E3B341", "pad_d": "#9A7420", "pad_on": "#6DBE73", "pad_on_d": "#3E8547",
             "goal": "#5E8FD6", "goal_l": "#9DBEF0", "link": "#D97757", "door": "#B07F55", "door_d": "#6B4A30"},
    "light": {"screen": "#1C2136", "termtx": "#E8ECF6", "bezel": "#5A5F78",
              "fl0": "#F3EEE2", "fl1": "#E9E2D2", "rwall": "#6A7192", "rwall_d": "#454B63",
              "pad": "#E3B341", "pad_d": "#A57B1C", "pad_on": "#6DBE73", "pad_on_d": "#3E8547",
              "goal": "#4F82D0", "goal_l": "#A8C6F2", "link": "#D97757", "door": "#A47C5B", "door_d": "#6B4A30"},
}
FLOOR = 80                               # floor row of the room behind everything
FADE = T - 0.4                           # the scene goes dark from FADE_START and resets here
FADE_START = FADE - 0.6

CLAUDE_AT = (12, 65)                     # Claude stands on a crate at the desk
TERM = (22, 12, 45, 43)                  # terminal x, y, w, h (cells)
BOARD = (70, 12, 84, 62)                 # room board x, y, w, h (cells)
DIV_X, DIV_W = 110, 3                    # divider wall
OPEN_Y, OPEN_H = 37, 12                  # opening in the divider, where the door sits
SWITCH = (82, 24, 12, 9)                 # floor switch x, y, w, h
GOAL_C, GOAL_R = (136, 57), 9.5          # goal circle
START_C = (82, 60)                       # where the duck starts (sprite centre)

# terminal lines: (text, colour, time typing starts); 1/16 s per character
TX = (TERM[0] + 3) * 2                   # text left edge, in 2 px units
LINES = [("ROOM.YAML", "lit", 0.3), ("- WALLS", "termtx", 1.0), ("- SWITCH", "termtx", 1.8),
         ("- DOOR", "termtx", 2.6), ("- GOAL", "termtx", 3.4), ("- LINK", "termtx", 4.2),
         ("CHECK", "green", 5.0), ("SUCCESS 0.99", "green", 9.6)]


def line_y(k):
    return (TERM[1] + 3) * 2 + 9 * k


def typed(k):
    """Time the line k finishes typing."""
    return LINES[k][2] + len(LINES[k][0]) * DT


T_WALLS, T_SWITCH, T_DOOR, T_GOAL, T_LINK = (typed(k) for k in range(1, 6))
T_TICK = typed(6) + 0.25
T_DUCK = T_TICK + 0.3
SW_ON = 7.0                              # duck reaches the switch
DOOR_OPEN = (SW_ON + 0.2, SW_ON + 0.7)
ARRIVE = 9.3                             # duck reaches the goal
# duck path: (time, centre)
PATH = [(T_DUCK + 0.2, START_C), (SW_ON, (88, 23)), (DOOR_OPEN[1] + 0.1, (88, 23)),
        (DOOR_OPEN[1] + 0.6, (100, 43)), (DOOR_OPEN[1] + 1.2, (122, 43)), (ARRIVE, GOAL_C)]


def snap(x, y):
    return f"{round(x) * P} {round(y) * P}"


def draw_room(c):
    bands = ["sky0", "sky1", "sky2", "sky3", "sky4"]
    for y in range(2, FLOOR):
        k = min(4, (y - 2) * 5 // (FLOOR - 2))
        for x in range(2, W - 2):
            c.set(x, y, bands[k])
    for y in range(FLOOR, H - 2):
        for x in range(2, W - 2):
            c.set(x, y, "wood_d" if (y - FLOOR) % 3 == 0 or (x + 7 * ((y - FLOOR) // 3)) % 23 == 0 else "wood")


def draw_desk(c):
    x, y, w, h = TERM
    c.box(x, y, w, h, "bezel")
    c.rect(x + 2, y + 2, w - 4, h - 4, "screen")
    c.rect(x + w // 2 - 2, y + h, 5, 61 - y - h, "metal_d")      # monitor stand
    c.rect(19, 61, 50, 2, "wood_d")                                # desk top
    c.rect(19, 61, 50, 1, "wood")
    for lx in (21, 64):
        c.rect(lx, 63, 2, FLOOR - 63, "wood_d")
    c.box(20, 58, 16, 3, "metal")                                  # keyboard
    c.box(4, 66, 15, FLOOR - 66, "wood")                           # crate under Claude
    c.rect(5, 72, 13, 1, "wood_d")


def draw_board(c):
    x, y, w, h = BOARD
    c.box(x, y, w, h, "fl0")
    for j in range(y + 1, y + h - 1):
        for i in range(x + 1, x + w - 1):
            if ((i - x) // 6 + (j - y) // 6) % 2:
                c.set(i, j, "fl1")
    c.rect(x + 8, y + h, 2, FLOOR - y - h, "wood_d")              # easel legs
    c.rect(x + w - 10, y + h, 2, FLOOR - y - h, "wood_d")


def walls_canvas():
    x, y, w, h = BOARD
    c = Canvas()
    ix, iy, iw, ih = x + 1, y + 1, w - 2, h - 2
    for (rx, ry, rw, rh) in ((ix, iy, iw, 2), (ix, iy + ih - 2, iw, 2), (ix, iy, 2, ih), (ix + iw - 2, iy, 2, ih),
                             (DIV_X, iy, DIV_W, OPEN_Y - iy), (DIV_X, OPEN_Y + OPEN_H, DIV_W, iy + ih - OPEN_Y - OPEN_H)):
        c.rect(rx, ry, rw, rh, "rwall")
        c.rect(rx, ry + rh - 1, rw, 1, "rwall_d")
    return c


def switch_canvas(on):
    x, y, w, h = SWITCH
    c = Canvas()
    c.rect(x, y, w, h, "pad_on_d" if on else "pad_d")
    c.rect(x + 1, y + 1, w - 2, h - 2, "pad_on" if on else "pad")
    c.rect(x + 4, y + 3, w - 8, h - 6, "pad_on_d" if on else "pad_d")
    return c


def goal_canvas():
    c = Canvas()
    gx, gy = GOAL_C
    for dy in range(-10, 11):
        for dx in range(-10, 11):
            r = math.hypot(dx, dy)
            if r < GOAL_R:
                c.set(gx + dx, gy + dy, "goal_l" if 5 <= r < 6.5 else "goal")
    return c


def link_canvas(col):
    """Signal wire from the switch to the door, drawn as a dotted line."""
    c = Canvas()
    sx, sy = SWITCH[0] + SWITCH[2], SWITCH[1] + SWITCH[3] // 2
    pts = [(i, sy) for i in range(sx + 1, DIV_X - 3)] + [(DIV_X - 3, j) for j in range(sy, OPEN_Y - 1)]
    for k, (i, j) in enumerate(pts):
        if k % 2 == 0:
            c.set(i, j, col)
    return c


def door_canvas():
    c = Canvas()
    c.rect(DIV_X, OPEN_Y, DIV_W, OPEN_H, "door_d")
    c.rect(DIV_X + 1, OPEN_Y + 1, DIV_W - 2, OPEN_H - 2, "door")
    return c

# ---------------------------------------------------------------- duck robot (original sprite)


DUCK_BODY = [
    "....www...",
    "...wwwww..",
    "...wwKwwoo",
    "...mmmmm..",
    "....www...",
    ".wwwwwwww.",
    "wwwmmmwwww",
    "wwwwmwwww.",
    ".wwwwwwww.",
]
DUCK_LEGS = {
    "A": ["...M..M...", "..oo...oo."],
    "B": ["...M.M....", "...oooo..."],
}


def duck_canvas(step):
    c = Canvas()
    c.sprite(DUCK_BODY + DUCK_LEGS[step], 0, 0)
    return outlined(c)


def duck_pos(t):
    if t <= PATH[0][0]:
        return PATH[0][1]
    for (t0, a), (t1, b) in zip(PATH, PATH[1:]):
        if t < t1:
            u = (t - t0) / (t1 - t0)
            return (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u)
    return PATH[-1][1]


def duck_layer(pal):
    pos, frames = [], {"A": [], "B": []}
    for k, t in enumerate(TICKS):
        cx, cy = duck_pos(t)
        pos.append((t, snap(cx - 5, cy - 5.5)))
        moving = any(t0 <= t < t1 and a != b for (t0, a), (t1, b) in zip(PATH, PATH[1:]))
        step = "B" if moving and (k // 2) % 2 else "A"
        for name in frames:
            frames[name].append((t, 1 if T_DUCK <= t < FADE and step == name else 0))
    body = "".join(group(svg_paths(duck_canvas(name), pal), timeline("opacity", vis), opacity=0)
                   for name, vis in frames.items())
    return group(body, timeline("translate", pos))

# ---------------------------------------------------------------- terminal


def terminal_layer(pal):
    out = ""
    for k, (s, col, t0) in enumerate(LINES):
        c = Canvas()
        draw_text(c, s, TX, line_y(k), col, shadow=None)
        width = text_width(s)
        # a screen-coloured mask over the line shrinks one character per tick: the line types itself
        xs, ws, adv = [(0, TX * 2)], [(0, width * 2)], 0
        for i, ch in enumerate(s):
            adv += len(FONT[ch][0]) + 1
            xs.append((t0 + (i + 1) * DT, (TX + adv) * 2))
            ws.append((t0 + (i + 1) * DT, max(0, width - adv) * 2 + 2))
        mask = (f'<rect x="{TX * 2}" y="{line_y(k) * 2}" width="{width * 2 + 2}" height="14" '
                f'fill="{pal["screen"]}">{timeline("x", xs)}{timeline("width", ws)}</rect>')
        out += shown(svg_paths(c, pal, unit=2) + mask, t0, FADE)
    tick = Canvas()                                                # green tick after CHECK
    tx = TX + text_width("CHECK") + 5
    for i, dy in enumerate((3, 4, 5, 4, 3, 2, 1, 0)):
        tick.rect(tx + i, line_y(6) + dy, 1, 2, "green")
    out += shown(svg_paths(tick, pal, unit=2), T_TICK, FADE)
    # cursor: sits after the text typed so far, blinking
    pos = []
    for t in TICKS:
        k = max((i for i, ln in enumerate(LINES) if ln[2] <= t), default=0)
        s, _, t0 = LINES[k]
        n = min(len(s), int((t - t0) / DT + 1e-9)) if t >= t0 else 0
        if n == len(s) and k + 1 < len(LINES):              # line done: the cursor waits on the next one
            k, n = k + 1, 0
        x = TX + (text_width(s[:n]) + 2 if n else 0)
        pos.append((t, f"{x * 2} {line_y(k) * 2}"))
    cur = f'<rect width="8" height="14" fill="{pal["lit"]}">{loop("opacity", 0.5, "1;0")}</rect>'
    out += group(shown(cur, LINES[0][2] - 0.2, FADE), timeline("translate", pos))
    return out

# ---------------------------------------------------------------- board pieces


def board_layer(pal):
    gx, gy = GOAL_C
    door = group(svg_paths(door_canvas(), pal),
                 timeline("translate", [(0, "0 0"), (DOOR_OPEN[0], f"0 {-3 * P}"),
                                        (DOOR_OPEN[0] + 0.125, f"0 {-6 * P}"), (DOOR_OPEN[0] + 0.25, f"0 {-9 * P}"),
                                        (DOOR_OPEN[0] + 0.375, f"0 {-12 * P}"), (FADE, "0 0")]))
    out = (shown(svg_paths(switch_canvas(False), pal), T_SWITCH, FADE)
           + shown(svg_paths(switch_canvas(True), pal), SW_ON, FADE)
           + shown(svg_paths(goal_canvas(), pal), T_GOAL, FADE)
           + shown(svg_paths(link_canvas("link"), pal), T_LINK, FADE)
           + shown(svg_paths(link_canvas("pad_on"), pal), SW_ON, FADE)
           + shown(door, T_DOOR, FADE)
           + shown(svg_paths(walls_canvas(), pal), T_WALLS, FADE))
    sx, sy, sw, sh = SWITCH
    for (x, y), t in (((BOARD[0] + 6, BOARD[1] + 4), T_WALLS), ((DIV_X + 1, BOARD[1] + 10), T_WALLS + 0.1),
                      ((sx + sw // 2, sy + sh // 2), T_SWITCH), ((DIV_X + 1, OPEN_Y + OPEN_H // 2), T_DOOR),
                      ((gx, gy), T_GOAL), ((DIV_X - 3, SWITCH[1] + 4), T_LINK),
                      (START_C, T_DUCK), ((sx + sw // 2, sy - 2), SW_ON),
                      ((gx - 11, gy - 8), ARRIVE + 0.1), ((gx + 11, gy - 6), ARRIVE + 0.3),
                      ((gx, gy - 13), ARRIVE + 0.5), ((gx - 10, gy + 9), ARRIVE + 0.7), ((gx + 10, gy + 9), ARRIVE + 0.9)):
        out += sparkle(pal, x, y, t)
    return out

# ---------------------------------------------------------------- Claude


def claude_pose(t, k):
    if LINES[-1][2] + 0.1 <= t < FADE_START:
        into = (t - LINES[-1][2] - 0.1) % 0.5
        return ("happy", "down", 1 if into < 0.25 else 0)          # nods at the result
    typing = any(t0 <= t < typed(i) for i, (_, _, t0) in enumerate(LINES[:-1]))
    if typing:
        return ("right", "typeA" if (k // 2) % 2 else "typeB", 0)
    blink = any(a <= t < a + 0.125 for a in (0.1, 6.2, 8.4))
    return ("blink" if blink else "right", "down", 0)


def claude_layer(pal):
    seq = [(t, claude_pose(t, k)) for k, t in enumerate(TICKS)]
    frames = {pose: claude_frame(eyes, arms, "stand", bob)
              for pose in dict.fromkeys(p for _, p in seq) for eyes, arms, bob in [pose]}
    ox, oy = (CLAUDE_AT[0] - 7) * P, (CLAUDE_AT[1] - 13) * P
    return "".join(group(svg_paths(fc, pal, ox=ox, oy=oy),
                         timeline("opacity", [(t, 1 if p == pose else 0) for t, p in seq]),
                         opacity=1 if pose == seq[0][1] else 0)
                   for pose, fc in frames.items())


def frame_border():
    f = Canvas()
    f.rect(0, 0, W, H, "frame_d")
    f.rect(1, 1, W - 2, H - 2, "frame")
    for y in range(2, H - 2):
        for x in range(2, W - 2):
            f.px.pop((x, y), None)
    for x, y in ((0, 0), (W - 1, 0), (0, H - 1), (W - 1, H - 1)):
        f.px.pop((x, y), None)
    for x, y in ((1, 1), (W - 2, 1), (1, H - 2), (W - 2, H - 2)):
        f.set(x, y, "frame_d")
    return f


def fade_layer(pal):
    """The scene darkens in three steps before the reset."""
    vis = [(0, 0), (FADE_START, 0.25), (FADE_START + 0.2, 0.5), (FADE_START + 0.4, 0.75), (FADE, 0)]
    return (f'<rect x="{2 * P}" y="{2 * P}" width="{(W - 4) * P}" height="{(H - 4) * P}" fill="{pal["K"]}" '
            f'opacity="0">{timeline("opacity", vis)}</rect>')


def build(theme):
    pal = {**PALETTES[theme], **THEME[theme]}
    bg = Canvas()
    draw_room(bg)
    draw_desk(bg)
    draw_board(bg)
    title = Canvas()                                          # unit 3 px
    draw_text(title, "MUJOCO PUZZLE GYM", W * P // 6 - text_width("MUJOCO PUZZLE GYM") // 2, 3, "text")
    desc = ("Pixel-art banner: Claude types a room.yaml at a terminal and each line adds a piece to a "
            "top-view puzzle room (walls with an opening, a yellow floor switch, a door, a blue goal and "
            "the link between them); after a green check, a small duck robot walks onto the switch, which "
            "turns green and slides the door open, then walks through to the goal while the terminal "
            "prints success 0.99 and Claude nods.")
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W * P} {H * P}" width="{W * P}" '
        f'height="{H * P}" shape-rendering="crispEdges" role="img" aria-labelledby="t d">'
        f'<title id="t">mujoco-puzzle-gym</title><desc id="d">{desc}</desc>'
        + svg_paths(bg, pal) + terminal_layer(pal) + board_layer(pal) + duck_layer(pal)
        + claude_layer(pal) + fade_layer(pal)
        + svg_paths(title, pal, unit=3) + svg_paths(frame_border(), pal)
        + "</svg>\n"
    )


def main():
    OUT.mkdir(exist_ok=True)
    for theme in PALETTES:
        path = OUT / f"banner-{theme}.svg"
        path.write_text(build(theme), encoding="utf-8")
        print(f"wrote {path.relative_to(ROOT)} ({path.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
