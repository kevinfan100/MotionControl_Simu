# STATUS: SCRATCH -- 2026-11-01 (user): overlay, no fill, no legend: bead outline and glass with the fillet wall (dotted) and probe-centre path for rho 4 um, and the probe-centre path of the bare bead on glass (red dashed), a_d 0.1, centre gap 2.51 um, bead radius 5 um. 2026-11-02: rho 10 curves removed.
# Output test_results/temp_3d_wsonly/fig_1101_wall_overlay.png
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path

rb, g = 5.0, 2.51


def path(rho):
    Rt = 2 * np.sqrt(rb * rho)
    a_t = np.arctan2(Rt, rho - rb)
    b = np.linspace(0, a_t, 400)
    c = np.linspace(a_t, 0, 400)
    return np.r_[(rb + g) * np.sin(b), Rt - (rho - g) * np.sin(c), Rt, 20], np.r_[rb + (rb + g) * np.cos(b), rho - (rho - g) * np.cos(c), g, g], Rt, a_t


def wall(rho):
    Rt = 2 * np.sqrt(rb * rho)
    a_t = np.arctan2(Rt, rho - rb)
    c = np.linspace(a_t, 0, 300)
    return Rt - rho * np.sin(c), rho - rho * np.cos(c)


ak = np.arccos((g - rb) / (rb + g))
b3 = np.linspace(0, ak, 400)
bx = np.r_[(rb + g) * np.sin(b3), 20]
bz = np.r_[rb + (rb + g) * np.cos(b3), g]
th = np.linspace(0, np.pi, 400)
fig, a = plt.subplots(figsize=(13, 8.2))
a.plot(rb * np.sin(th), rb + rb * np.cos(th), color=(0.35, 0.35, 0.35), lw=2.2, label="bead (radius 5 µm)")
a.plot([0, 20], [0, 0], color="k", lw=2.2, label="glass")
for rho, cl in [(4.0, (0, 0.25, 0.85))]:
    x, z, Rt, at = path(rho)
    wx, wz = wall(rho)
    a.plot(wx, wz, color=cl, lw=1.2, ls=":", label=f"fillet wall, ρ = {rho:.0f} µm")
    a.plot(x, z, color=cl, lw=2.2, label=f"probe centre path, ρ = {rho:.0f} µm")
    print(f"rho {rho:.0f}: ring {Rt:.2f} um, concave path radius {rho-g:.2f} um, tilt {at*180/np.pi:.1f} deg, deg/step at 40 um/s {0.0248/(rho-g)*180/np.pi:.2f}")
a.plot(bx, bz, color=(0.85, 0, 0), lw=2.2, ls="--", label="probe centre path, bead on glass")
a.set_xlim(0, 20); a.set_ylim(-0.5, 13); a.set_aspect("equal")
a.set_xlabel("radial position  [µm]", fontweight="bold"); a.set_ylabel("z  [µm]", fontweight="bold")
for t in a.get_xticklabels() + a.get_yticklabels():
    t.set_fontweight("bold")
fig.tight_layout()
out = Path(__file__).resolve().parents[2] / "test_results" / "temp_3d_wsonly" / "fig_1101_wall_overlay.png"
fig.savefig(out, dpi=130)
