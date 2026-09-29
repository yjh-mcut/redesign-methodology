"""Annotate the three-generation render with the changes each generation carried.

    python docs/annotate_history.py

Reads docs/images/fig6_history.png and writes docs/images/fig_history_annotated.png,
which is Figure 2 of the paper.

Marker positions are measured from the image, not placed by eye.  The external
spur pairs are lavender against an otherwise black-and-grey render, so they can
be found by colour; the leg centres come from a column-density scan.  Running
this on a different render will move the markers with it.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse, FancyArrowPatch
from PIL import Image

SRC = 'docs/images/fig6_history.png'
DST = 'docs/images/fig_history_annotated.png'
GEAR = '#c0392b'
IDLER = '#1f6fb4'

# The idler-bearing race sits above the servo plate and is the one thing the
# third generation has that the second does not.  Measured extent x 554-647,
# y 7-18; the servo plate below starts at y = 19.
IDLER_RING = dict(centre=(600.5, 12.5), semi=(50.0, 8.5))

GENERATIONS = [  # x window, label, first contested, stance
    ((0, 200), 'Generation 1', 2017, '$W$ = 26.0 cm'),
    ((260, 410), 'Generation 2', 2018, '$W$ = 17.9 cm'),
    ((520, 670), 'Generation 3', 2019, '$W$ = 17.9 cm'),
]


def find_gears(rgb):
    """Centres of the lavender spur pairs, by colour and connected component."""
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    mask = (b - r > 18) & (b - g > 12) & (b > 150) & (r > 110)
    seen = np.zeros_like(mask, bool)
    h, w = mask.shape
    out = []
    from collections import deque
    for y0, x0 in zip(*np.nonzero(mask)):
        if seen[y0, x0]:
            continue
        queue, pts = deque([(y0, x0)]), []
        seen[y0, x0] = True
        while queue:
            y, x = queue.popleft()
            pts.append((y, x))
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        queue.append((ny, nx))
        if len(pts) >= 40:                    # ignore stray lavender pixels
            a = np.array(pts)
            out.append((a[:, 1].mean(), a[:, 0].mean()))
    return sorted(out)


def find_legs(grey, x0, x1, y0=120, y1=260):
    """Centre of each leg, from how much of each column is dark."""
    col = (grey[y0:y1, x0:x1] < 160).sum(axis=0)
    on = col > (y1 - y0) * 0.25
    runs, start = [], None
    for i, v in enumerate(on):
        if v and start is None:
            start = i
        if not v and start is not None:
            if i - start > 3:
                runs.append((x0 + start, x0 + i - 1))
            start = None
    if start is not None and len(on) - start > 3:
        runs.append((x0 + start, x0 + len(on) - 1))
    return [(a + b) / 2 for a, b in runs]


def main():
    im = Image.open(SRC).convert('RGB')
    rgb = np.array(im).astype(int)
    grey = np.array(im.convert('L'))
    width, height = im.size

    gears = find_gears(rgb)
    if len(gears) != 8:
        raise SystemExit(f'expected 8 spur pairs, found {len(gears)} -- check the source image')

    band = 175
    fig = plt.figure(figsize=(width / 100 * 1.35, (height + band) / 100 * 1.35), dpi=200)
    ax = fig.add_axes([0, band / (height + band), 1, height / (height + band)])
    ax.imshow(im)
    ax.axis('off')
    ax.set_xlim(-6, width + 6)
    ax.set_ylim(height, -10)          # headroom so the idler ellipse is not clipped

    for cx, cy in gears:
        ax.add_patch(Circle((cx, cy), 12, fill=False, ec=GEAR, lw=1.7))
    ax.add_patch(Ellipse(IDLER_RING['centre'],
                         2 * IDLER_RING['semi'][0], 2 * IDLER_RING['semi'][1],
                         fill=False, ec=IDLER, lw=1.8))

    lab = fig.add_axes([0, 0, 1, band / (height + band)])
    lab.axis('off')
    lab.set_xlim(0, width)
    lab.set_ylim(0, band)
    for (x0, x1), name, year, stance in GENERATIONS:
        legs = find_legs(grey, x0, x1)
        centre = sum(legs) / len(legs)
        lab.text(centre, 150, name, ha='center', fontsize=9.4, weight='bold')
        lab.text(centre, 131, f'first contested {year}', ha='center', fontsize=7.6, color='0.3')
        lab.text(centre, 111, stance, ha='center', fontsize=8.4)
        lab.add_patch(FancyArrowPatch((legs[0], 98), (legs[-1], 98), arrowstyle='<->',
                                      mutation_scale=9, lw=1.0, color='0.25'))

    lab.add_patch(Circle((40, 56), 10, fill=False, ec=GEAR, lw=1.7))
    lab.text(60, 56, 'external spur pair added at the hip-roll and ankle-roll axes (Gen 2 onward)',
             fontsize=8.0, color=GEAR, va='center')
    lab.add_patch(Ellipse((40, 22), 26, 15, fill=False, ec=IDLER, lw=1.8))
    lab.text(60, 22, 'hip-yaw bolt replaced by an idler-bearing assembly (Gen 3)',
             fontsize=8.0, color=IDLER, va='center')

    fig.savefig(DST, dpi=200, facecolor='white', bbox_inches='tight', pad_inches=0.03)
    plt.close(fig)
    print(f'{len(gears)} spur pairs marked at ' +
          ', '.join(f'({x:.0f},{y:.0f})' for x, y in gears))
    print(f'wrote {DST}')


if __name__ == '__main__':
    main()
