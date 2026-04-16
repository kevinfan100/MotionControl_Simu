# Q/R Theoretization — Progress Record

## 2026-04-16 — Step (b) derivation + Step (a) simulation + Problem identification

### Completed Parts
- ✅ Step (b): Paper Q/R entry-by-entry theoretical derivation (9 entries, 8 derivable)
- ✅ Step (a): Simulation with paper Q/R vs baseline (noise OFF + noise ON)
- ✅ C_dpmr_eff sensitivity to Q(3,3) and Q(6,6) — confirmed numerically
- ✅ Warmup drift quantification: z-axis 1σ=±21.8%, 3σ=±65% at h=2.5um
- ✅ Seed variance verification: baseline config + seed 12345 = normal, paper Q/R + seed 12345 = +67% z bias
- ✅ 5 near-wall problem points identified and severity-ranked
- ✅ Adaptive Q(3,3) = 4kBT·a_hat derivation (from paper Eq.21 + wall effect)
- ✅ Self-consistent Q/R lookup verification: C_dpmr_eff(lc, a/a_nom) smooth, Q/R ratio increases near wall
- ⏸️ Phase 2-5: Adaptive Q/R implementation (plan approved, not started)

### File Changes
**New Files:**
- `reference/for_test/qr_theoretical_values.md` (440 lines)
  Purpose: Entry-by-entry Q/R derivation from physics, comparison table
- `reference/for_test/task_qr_reference_report.md` (124 lines)
  Purpose: Step (a) simulation results, 3 configs comparison, key findings

**Modified Files:**
- `.claude/settings.local.json` (+4 lines) — settings change
- `reference/for_test/fig_*.png` — lookup figures regenerated (restored to original)

**Data Files (gitignored):**
- `test_results/verify/paper_qr_paper_qr_noiseOFF.mat`
- `test_results/verify/paper_qr_paper_qr_noiseON.mat`

### Testing Status
⏸️ Phase 1 verification complete, Phase 2-5 pending
- Verified: C_dpmr_eff sensitivity, warmup drift physics, seed robustness ✅
- Pending: adaptive Q/R implementation + multi-seed MC verification ⬜

### Next Steps
- [ ] Phase 2: Build new lookup (lc x a/a_nom) with self-consistent Q/R
- [ ] Phase 3: Implement adaptive Q(3,3), R(2,2), C_dpmr_eff, warmup re-init
- [ ] Phase 4: Multi-seed verification (5 seeds, h=2.5 + h=20)
- [ ] Phase 5: Update documentation

### Issues & Notes
⚠️ **Critical finding**: Paper Q/R with fixed Q is seed-dependent (a_z: +9% to +67% across seeds)
⚠️ **C_dpmr_eff dependency**: With paper R, Q(3,3)=0.096->1 causes 12.75% C_dpmr_eff change -> must co-adapt
💡 **Key insight**: Self-consistent Q/R ratio INCREASES near wall (proportional to 1/a) -> EKF more responsive near wall -> physically correct
💡 **Q(4,4)**: Set to 0 initially; add back empirically if needed after verification

### Git Commit
`8a212eb` - WIP(test/qr): Q/R theoretical derivation + simulation verification + adaptive Q plan

---
