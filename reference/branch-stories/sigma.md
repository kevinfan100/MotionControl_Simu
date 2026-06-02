# feat/sigma-ratio-filter — branch story

> 故事索引, 給日後翻閱用。完整 commit history 在 archive tag `archive/sigma-pre-cleanup` (origin remote + 離線 bundle `D:/archives/MotionControl_Simu/sigma-pre-cleanup.bundle`)。

## 主軸

從 main HEAD (`8cc32ee`, 7-state EKF 引入點) fork 出去後, 獨立累積 79 commits, 圍繞 **「Paper 2025 Eq.6 控制律」+「per-axis preset Q/R」+「Sigma_e ratio filter 理論探索」+ qr 規範化文件** 四條工作線。

最後 merge 了 `test/qr-paper-reference` (`c33111c`), 將 paper-reference 分支累積的 writeup + apply_qr_preset + 4-stage Q/R fix 全部納入。

## 里程碑

| 階段 | 關鍵 commit | 結論 |
|---|---|---|
| Sigma_e ratio filter L0-L4 | `07d5ab2`, `55bd65c`, `0f79c5f`, `5e492a6` | Oracle ratio filter 69% 較 IIR 為佳; 但 5-state KF 整合 fail (正反饋); 未塞進 runtime, 留 ablation |
| Time-varying Σ_e recursion | `8637c7d`, `c765b38` | 變異數遞推驗證; figures 記錄 |
| C_dpmr augmented Lyapunov | `cad4a57` | 7-state EKF 用 augmented Lyapunov 推導 C_dpmr_eff |
| IIR finite-sample bias | `df68dce`, `53cf857` | Task 1b-1c: IIR 有 finite-sample bias, 已 integrate into 7-state EKF |
| 4-stage Q/R 完工 (h=50) | `29d74fb`, `105611b`, `d80829b` | Stage 1: per-axis R(1,1); Stage 2: per-axis R(2,2); Stage 3: per-axis Q(7,7)=0; Stage 4: on-the-fly C_dpmr_eff; 最終 5-seed CI a_hat std ratio x=0.92/z=0.84 |
| Wall-aware a_hat init | `9a0e8db` | EKF 用 c_para/c_perp 在 h_bar_init 初始化, 避開 slow convergence |
| qr-paper-reference merge | `c33111c` | 帶入 writeup_architecture, P2 positioning verification, qr_analysis canonical |
| V5/V7 cross-branch 研究 (Stage 1 pre-cleanup) | (uncommitted) | 證明 Q(6,6) 值從 1e-8 (empirical) → 1.89e-10 (physics) 讓 a_hat std 5-7% → 1-1.4%; **controller 架構非 a_hat std 主因** |

## V5/V7 結論的影響

V5/V7 是這次整理前的最重要發現 — 在進 new main 設計時, 兩 controller 都改採 on-the-fly 物理 Q/R 的依據, 就是 V7 已經實證 eq6 控制律配 eq17 物理 Q(6,6) 跑得比原 empirical preset 更好。詳見 `reference/eq6_analysis/q66_value_dominance.md` 與 `reference/eq6_analysis/transitions_report.md`。

## 為何選此 (進 main) / 不選此 (留 archive)

進 main:
- `motion_control_law_eq6.m` (= 整理後的 _7state.m, 帶 wall-aware init + 4-stage Q/R + on-the-fly C_dpmr_eff)
- `calc_ctrl_params.m`, `apply_qr_preset.m`, `compute_*.m` (per-axis Q/R + Stage 4 on-the-fly 全套)
- `reference/shared/writeup_architecture.tex/.pdf` (兩 controller 共通推導)
- `reference/eq6_analysis/` 整套 canonical 文件 + figures + archive snapshot
- `q66_value_dominance.md` + `fig_transitions/` + `transitions_report.md` (V5/V7 study)

留 archive (不進 main):
- `motion_control_law_olmode.m` (V5/V7 isolation harness, new main 有雙 controller 後過時) — 在 `archive/sessions/V5_V7_study/`
- `motion_control_law_{1,2,4,5state}.m` 4 個 legacy 變體 — 在 `archive/legacy_controllers/`
- `docs/superpowers/specs/2026-04-28-qr-cleanup-design.md` 一次性整理 spec — 在 `archive/sessions/`
- `feat_sigma-ratio-filter.md` 進度筆記 — 在 `.claude/progress/` (gitignored 本來就不進 main)
- 與 ratio filter L0-L4 相關所有 .mat sweep results — 因架構未 deploy, 不放 main 浪費空間

## Key commits to remember

```
05b699e fix(test): verify_qr_derivation variant shape for per-axis Bus  ← archive tag 凍住的 HEAD
c33111c merge: test/qr-paper-reference into feat/sigma-ratio-filter
105611b feat(controller): per-axis on-the-fly C_dpmr/Cn/beta
29d74fb feat(controller): per-axis Q/R Bus extension
d80829b feat(config): apply_qr_preset + frozen_correct preset
cad4a57 feat(analysis): augmented Lyapunov C_dpmr_eff for 7-state EKF
9a0e8db feat(controller): wall-aware initial a_hat for EKF
```

## 復原指令 (若需要)

```bash
# 從 origin 拉 archive tag
git fetch origin tag archive/sigma-pre-cleanup

# 把整個 sigma branch 還原至 working tree
git checkout archive/sigma-pre-cleanup

# 或從 bundle 重建一個 fresh clone
git clone D:/archives/MotionControl_Simu/sigma-pre-cleanup.bundle recovered_sigma
```
