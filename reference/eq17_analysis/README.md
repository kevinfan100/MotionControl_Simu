# reference/eq17_analysis/

Paper 2023 Eq.17 d-step delay-compensated controller — design, derivation,
verification, and investigation log.

## Layout

```
eq17_analysis/
├── README.md                          (this file)
├── design_v2.md                       canonical design spec (paper Eq.17 with
│                                       Σ f_d retained + x_D additive)
├── task01_math_observability_report.md  observability rank/structure result
│
├── derivation/   Phase 1-7 step-by-step derivation chain
│   ├── phase1_Fe_derivation.md           F_e Row 3 + structural columns
│   ├── phase2_C_dpmr_C_n_derivation.md   C_dpmr, C_n closed-form
│   ├── phase2_IF_var_dpr_derivation.md   IF_var, dpr correlation
│   ├── phase3_algebraic_verification.md  cross-check of phases 1+2
│   ├── phase4_observability_rank.md      rank-test design + theory
│   ├── phase5_Q_matrix_derivation.md     Q assembly (Q33, Q66, Q77 physics)
│   ├── phase6_R_matrix_derivation.md     R assembly + 3-guard logic
│   └── phase7_lyapunov_bench.md          closed-loop variance oracle spec
│
├── verification/   Phase 8 (Wave 1-3 + Stage 10) + Phase 9
│   ├── phase8_e2e_h50_results_v4.md      canonical 5-seed h=50 results
│   ├── phase9_validation_report.md       Phase 9 Layer 1 R(2,2) validation
│   ├── development_log_phase8.md         synthesis of 7 Phase 8 reports
│   ├── phase9_predictions.mat            production (verify scripts load)
│   └── phase9_stageI_acf_diagnosis.mat   production (calc_ctrl_params X2a)
│
├── investigations/   Specific finding documents
│   ├── xD_suppression_finding.md         x_D not acting as thermal compensator
│   ├── Q66_physical_test_report.md
│   ├── cdpmr_closed_form_verify_5seed.md
│   ├── near_wall_gap.md                  synthesis: h_bar<2 divergence + K_h cap fix
│   └── development_log_iir_prefill.md    synthesis: 5-round IIR prefill validation
│
├── figures/   Active figure dumps per scenario
│   ├── positioning_h50/                  canonical h=50
│   ├── motion_z_ramp_50to{5,10}/         ramp scenarios
│   ├── motion_z_osc/                     oscillation
│   ├── paper_compare/                    cross-paper compare
│   ├── suppress_xD_ramp_acov005/         Stage 2 finding (+diagnostic/)
│   ├── iir_prefill/                      iir prefill validation
│   └── theory/                           design / theory figures
│
└── archive/   Historical originals (don't read first; use synthesis above)
    └── sessions/                         design.md v1, phase8 wave reports,
                                          near-wall report originals, iir prefill
                                          source reports, plus suppress_xD_study/
                                          and figures_pre_phase9/ subdir tree
```

## Where to start

- **What is this controller?** → `design_v2.md`
- **How was Q/R derived?** → `derivation/phase5` (Q) + `derivation/phase6` (R)
- **Does it work at h=50?** → `verification/phase8_e2e_h50_results_v4.md`
- **Phase 9 R(2,2) story?** → `verification/phase9_validation_report.md`
- **Why does it diverge near wall?** → `investigations/near_wall_gap.md`
- **Why IIR prefill?** → `investigations/development_log_iir_prefill.md`
- **What did Phase 8 actually go through?** → `verification/development_log_phase8.md`

For original source documents (verbatim) see `archive/sessions/`.
