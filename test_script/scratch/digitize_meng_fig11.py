# PURPOSE (2026-09-07): digitise the three measured normalized-gain curves (plane / spherical / cell surface, lambda_z/lambda_N vs
#   h_d/R) of IEEE article 11072733 Fig. 11 (Meng; the gif the user supplied, not stored in the repo) into
#   reference/eq17_analysis/data/meng_11072733_fig11_three_surfaces.csv. Axes frame detected from the black box (x 1..6, y 0..1),
#   curves by colour masks (blue / red / green), one median row per pixel column. Companion fit: fit_meng_fig11_walls.m.
import sys, csv
from PIL import Image
import numpy as np
gif = sys.argv[1] if len(sys.argv) > 1 else '/Users/kevin/Downloads/11072733-fig-11-source-large.gif'
out = sys.argv[2] if len(sys.argv) > 2 else 'reference/eq17_analysis/data/meng_11072733_fig11_three_surfaces.csv'
a = np.array(Image.open(gif).convert('RGB')).astype(int); r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
masks = {'plane': (b > 150) & (r < 120) & (g < 120), 'sphere': (r > 150) & (g < 120) & (b < 120), 'cell': (g > 150) & (r < 120) & (b < 120)}
black = (r < 60) & (g < 60) & (b < 60); H, W = black.shape
rows = np.where(black.sum(1) > 0.5 * W)[0]; cols = np.where(black.sum(0) > 0.5 * H)[0]
frame_rows = [x for x in rows if x > 80]; y1, y0 = min(frame_rows) + 0.5, max(frame_rows) + 0.5; x0, x1 = min(cols) + 0.5, max(cols) + 0.5
with open(out, 'w') as f:
    w = csv.writer(f); w.writerow(['surface', 'hd_R', 'lambda_norm'])
    for k, m in masks.items():
        for c in range(int(x0) + 2, int(x1) - 1):
            rr = np.where(m[int(y1) + 2:int(y0) - 1, c])[0]
            if len(rr) == 0: continue
            y = np.median(rr) + int(y1) + 2
            w.writerow([k, '%.4f' % (1 + 5 * (c - x0) / (x1 - x0)), '%.4f' % ((y0 - y) / (y0 - y1))])
print('saved', out)
