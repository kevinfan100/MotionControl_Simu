# Q/R Theoretization — Progress Record

## 2026-04-16 (Session 2) — Q/R Coupled Stability Analysis

### Completed Parts
- ✅ Step (b): Q/R entry-by-entry derivation (8/9 from physics)
- ✅ Step (a): Paper Q/R simulation + seed instability discovery
- ✅ 5 near-wall problem points identified and severity-ranked
- ✅ Self-consistent C_dpmr_eff lookup rebuilt (lc x a/a_nom, 48 DARE points)
- ✅ Warmup drift quantification: z 1sig=+-22%, x 1sig=+-11%
- ✅ h=50um free-space verification: paper Q/R ALSO unstable (NOT wall-effect specific)
- ✅ Q(6,6)-R(2,2) coupled stability mechanism identified and documented
- ✅ Forgetting factor sweep (lf=0.99/0.98/0.95): lf=0.99 stabilizes z (spread 6pp) but not x
- ⏸️ Adaptive Q(3,3)/C_dpmr_eff implementation attempted, blocked by Stateflow limitations
- ⏸️ Combined Q(6,6)+R(2,2) coupled fix not yet tested

### File Changes
**Modified Files:**
- `test_script/build_cdpmr_eff_lookup.m` (+94/-72 lines)
  Changed: f0 axis -> aratio axis, self-consistent Q/R per grid point
- `model/controller/calc_ctrl_params.m` (+2/-2 lines)
  Changed: read last column (free-space) instead of first column
- `reference/for_test/fig_cdpmr_eff_lookup.png` (regenerated for aratio lookup)
- `reference/for_test/fig_bias_factor_lookup.png` (regenerated)

### Testing Status
⏸️ Extensive testing completed, key findings documented
- Tested: paper Q/R multi-seed (h=2.5, h=50) ✅
- Tested: forgetting factor lf=0.99/0.98/0.95 ✅
- Tested: adaptive Q(3,3) + C_dpmr_eff (Stateflow blocked) ✅
- Pending: Q(6,6)+R(2,2) coupled sweep ⬜
- Pending: lf=0.99 + R(2,2)=1.0 combined test ⬜

### Next Steps
- [ ] Test lf=0.99 + R(2,2)=1.0 (coupled fix, config-only, no code change)
- [ ] Resolve Stateflow compatibility for adaptive Q(3,3)/C_dpmr_eff
- [ ] Update qr_theoretical_values.md with Q(6,6)-R(2,2) coupling analysis
- [ ] Update task_qr_reference_report.md with all test results
- [ ] Decide writeup narrative: physical derivation + coupled stability story

### Issues & Notes
⚠️ **Critical finding**: Q(6,6)=0 is structurally fragile at ANY h (not wall-specific)
- P(6,6) collapses in ~100 steps -> L(6,2)->0 -> a_hat permanently frozen
- Combined with small R(2,2)=0.176 -> L(6,2) approx 1 during critical window -> a_hat slammed by noise

⚠️ **Stateflow limitation**: if/else branches, evalin, load, local functions all fail compilation
- Workaround: analytic C_dpmr approximation passed but overall still unstable

💡 **Key insight**: Q(6,6) and R(2,2) are a COUPLED stability pair
- Physical values (both small) -> fragile estimator
- Code values (both inflated) -> robust but conservative
- Forgetting factor lf<1 can replace Q(6,6) but NOT R(2,2)

💡 **Writeup value**: physical Q/R derivation is correct but insufficient without stability mechanisms

### Git Commit
`c3214c2` - WIP(test/qr): Q/R coupled stability analysis + forgetting factor + free-space verification

---

## 2026-04-16 (Session 1) — Step (b) derivation + Step (a) simulation

### Git Commit
`8a212eb` - WIP(test/qr): Q/R theoretical derivation + simulation verification + adaptive Q plan

(See previous progress entry for details)

---
