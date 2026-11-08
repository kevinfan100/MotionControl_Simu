# STATUS: SCRATCH -- 2026-11-01 (user): cross-section of wall options. A = fillet rho 4 um; B = bare bead on glass (sharp junction). Probe radius 2.25 um, centre gap 2.51 um, bead radius 5 um.
# Top: wall, probe, probe-centre path. Bottom: wall-normal tilt along the path, from the bead top to the glass.
# Output test_results/temp_3d_wsonly/fig_1101_wall_ab.png
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path

rb, g, Rp, rho = 5.0, 2.51, 2.25, 4.0
cE, cT, cW = (0, 0.25, 0.85), (0.85, 0, 0), (0.35, 0.35, 0.35)
fig, ax = plt.subplots(2, 2, figsize=(14, 9.5), gridspec_kw={"height_ratios": [1.5, 1]})

# A: fillet
Rt = 2 * np.sqrt(rb * rho)
a_t = np.arctan2(Rt, rho - rb)                     # tilt of the tangency seen from the bead centre
b = np.linspace(0, a_t, 400)
wx, wz = rb * np.sin(b), rb + rb * np.cos(b)
c = np.linspace(a_t, 0, 400)                       # along the fillet, normal tilt falls from a_t to 0
fx, fz = Rt - rho * np.sin(c), rho - rho * np.cos(c)
px1, pz1 = (rb + g) * np.sin(b), rb + (rb + g) * np.cos(b)
px2, pz2 = Rt - (rho - g) * np.sin(c), rho - (rho - g) * np.cos(c)
gx = np.linspace(Rt, 15, 50)
ax[0, 0].fill(np.r_[wx, fx, 15, 15, 0], np.r_[wz, fz, 0, -1, -1], color=(0.88, 0.88, 0.88))
ax[0, 0].plot(np.r_[wx, fx, 15], np.r_[wz, fz, 0], color=cW, lw=2.2, label="wall")
ax[0, 0].plot(np.r_[px1, px2, gx], np.r_[pz1, pz2, g + 0 * gx], color=cE, lw=2.2, label="probe centre path")
s1 = (rb + g) * b
s2 = s1[-1] + (rho - g) * (a_t - c)
s3 = s2[-1] + (gx - Rt)
sA = np.r_[s1, s2, s3]
tA = np.r_[b, c, 0 * gx] * 180 / np.pi
k = 200
pc = (px2[k], pz2[k])

# B: bare bead on glass
ak = np.arccos((g - rb) / (rb + g))                # tilt at the kink: equal gap to bead and glass
b2 = np.linspace(0, np.pi, 500)
wx2, wz2 = rb * np.sin(b2), rb + rb * np.cos(b2)
b3 = np.linspace(0, ak, 400)
qx, qz = (rb + g) * np.sin(b3), rb + (rb + g) * np.cos(b3)
kx = qx[-1]
hx = np.linspace(kx, 15, 50)
ax[0, 1].fill(np.r_[wx2, 15, 15, 0], np.r_[wz2, 0, -1, -1], color=(0.88, 0.88, 0.88))
ax[0, 1].plot(np.r_[wx2, 15], np.r_[wz2, 0], color=cW, lw=2.2, label="wall")
ax[0, 1].plot(np.r_[qx, hx], np.r_[qz, g + 0 * hx], color=cE, lw=2.2, label="probe centre path")
ax[0, 1].plot(kx, g, "o", color=cE, ms=8)
sB = np.r_[(rb + g) * b3, (rb + g) * ak + (hx - kx)]
tB = np.r_[b3, 0 * hx] * 180 / np.pi

th = np.linspace(0, 2 * np.pi, 200)
for a, (x0, z0) in zip(ax[0], [pc, (kx, g)]):
    a.plot(x0 + Rp * np.cos(th), z0 + Rp * np.sin(th), color=cT, lw=1.8, label="probe (radius 2.25 µm)")
    a.plot(x0, z0, ".", color=cT, ms=9)
    a.set_xlim(0, 15); a.set_ylim(-0.6, 11); a.set_aspect("equal")
    a.set_xlabel("radial position  [µm]", fontweight="bold"); a.set_ylabel("z  [µm]", fontweight="bold")
    a.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=3, frameon=True)
ax[1, 0].plot(sA, tA, color=cE, lw=2.2, label=f"A: fillet ρ = {rho:.0f} µm")
ax[1, 1].plot(sB, tB, color=cE, lw=2.2, label="B: bead on glass")
for a in ax[1]:
    a.set_ylim(-5, 125); a.set_xlim(0, 22)
    a.set_xlabel("path length from the bead top  [µm]", fontweight="bold"); a.set_ylabel("wall normal tilt  [deg]", fontweight="bold")
    a.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0))
for a in ax.ravel():
    for t in a.get_xticklabels() + a.get_yticklabels():
        t.set_fontweight("bold")
fig.tight_layout()
out = Path(__file__).resolve().parents[2] / "test_results" / "temp_3d_wsonly" / "fig_1101_wall_ab.png"
fig.savefig(out, dpi=130)
print(f"A: ring {Rt:.2f} um, tilt at tangency {a_t*180/np.pi:.1f} deg, concave path radius {rho-g:.2f} um, length {(rho-g)*a_t:.2f} um, at 40 um/s {(rho-g)*a_t/40*1e3:.0f} ms, {a_t*180/np.pi/((rho-g)*a_t/40*1612):.2f} deg/step")
print(f"B: kink at r {kx:.2f} um, normal jumps {ak*180/np.pi:.1f} deg to 0 at one point; bead is reached down to tilt {ak*180/np.pi:.1f} deg (height {rb+rb*np.cos(ak):.2f} um); unseen under it")
print(out)
