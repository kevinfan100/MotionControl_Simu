# reference/eq6_analysis/

Paper 2025 Eq.6 controller (per-axis 7-state EKF, Stage 4 on-the-fly C_dpmr_eff)
— design, verification, and investigation log.

## Layout

```
eq6_analysis/
├── README.md                            (this file)
├── qr_theory_complete_final.md          canonical Q/R theory + 4-stage history
├── qr_known_limitations.md              open issues + accepted limitations
│
├── verification/   h=50 paper-level + P2 + axm + R(2,2) self-consistent
│   ├── qr_verification_h50_final_2026-04-27.md    canonical h=50 result
│   ├── task_p2_positioning_verification.md
│   ├── task_p2_linearization_quantitative.md
│   ├── r22_self_consistent_report.md
│   ├── phase1_axm_audit.md                a_xm derivation audit
│   ├── fig_qr_h50_ahat_std.png
│   ├── fig_qr_h50_tracking_std.png
│   ├── fig_qr_positioning_summary.png
│   ├── fig_qr_verification.png
│   ├── fig_r22_convergence.png
│   └── fig_qr_verification_h50/         subdir of detail figs
│
├── investigations/   V5/V7 finding + Stage 1 outputs + KF param notes
│   ├── q66_value_dominance.md           Stage 1 finding (Q(6,6) physics dominates)
│   ├── transitions_report.md            V1-V7 full study record
│   ├── 0427_kf_paras.md                 KF parameter snapshot 2026-04-27
│   └── fig_transitions/                 8 PNG V5/V7 evidence
│
├── figures/   lookup-table generation outputs (not scenario-specific)
│   ├── fig_baseline_empirical_zoom.png
│   ├── fig_bias_factor_lookup.png
│   ├── fig_cdpmr_eff_lookup.png
│   └── fig_q77_trajectory.png
│
├── theory_pdfs/   external paper / appendix material
│   ├── Observer-1.pdf
│   └── Tracking Error Variance -1.pdf
│
└── archive/   sigma 2026-04-28 intentional snapshot (don't read first)
    ├── figs/                             32 historical PNG
    ├── historical_notes/                 15 historical .md
    │                                      (merged sessions/+theory/)
    ├── verification/                     12 historical verify reports
    ├── legacy_controllers/               4 archived motion_control_law_*.m
    └── V5_V7_study/                      motion_control_law_olmode.m
                                          (V5/V7 isolation harness)
```

## Where to start

- **What is this controller?** → See `agent_docs/eq6_or_23state/ekf-matrix-guide.md`
  + paper source `reference/controller_paper_source/`
- **What's the canonical Q/R theory?** → `qr_theory_complete_final.md`
- **Does it work at h=50?** → `verification/qr_verification_h50_final_2026-04-27.md`
- **Stage 1 V5/V7 finding (Q(6,6) physics dominates)?** → `investigations/q66_value_dominance.md`
- **What limitations are known?** → `qr_known_limitations.md`
- **History: what was sigma's exploration?** → `../branch-stories/sigma.md`
  + `archive/` for verbatim originals
