# 通用軌跡規約（2026-08-19 起）

**SSOT = `model/config/canonical_scenario.m`。** 五支 gain-law driver
(`run_formC_b` / `run_formC_dist` / `run_formC_state` / `run_formC_ugap` /
`run_formB_ws`) 各自的 `local_canonical_config` 現在只是它的薄殼。
**軌跡只能改那一個檔**；在 driver 裡再開一份是 2026-08-19 之前的狀態，
當時五份 byte-identical 的拷貝讓「改了四支漏一支」變成可能。

## 兩個名字

| name | 谷底 | 帶 | 谷底拖曳 | 狀態 |
|---|---|---|---|---|
| `deep` | w̄ = 1.10 | [1.10, 3.32] | 11.45× | drivers 的預設（未改） |
| `shallow` | w̄ = 2.00 | [2.00, 4.22] | 2.12× | 2026-08-19 之前 |
| `ramp_tri` | w̄ = 1.10 | [1.10, 2.00] | 11.45× | **一階線與 c-only 線的測試預設（2026-10-04）** |

`ramp_tri`（使用者 10-04 裁示）：**沒有 hold**；22.2 → 1.10 等速下坡 1 s；接三角波 1.10 ↔ 2.00、
從谷底往上、固定 2 s；T_sim = 3 s。唯一旋鈕 = 三角波頻率 f：`canonical_scenario(a_cov, h_min, 'ramp_tri', f)`，
或只吃名字的 driver 用 `config_override` 設 `frequency = f`、`n_cycles = 2f`。軌跡型 `osc_tri`（`trajectory_generator`
phase 2 等速、phase 3 三角）。起點與谷底同 deep ⇒ 包絡導出的 prior 全部不變。判準逐點：谷底（1、2、3 s）、
頂端（1.5、2.5 s）、三角段 |e_ā| 均值；沒有 hold 窗（driver 回 NaN，不捏造）。
落地時四件事（10-04）：deep 逐位相同（`run_formC_b` 1 seed 改前改後 `isequaln`）、shallow 可重複；五支 driver 都跑得動
（`run_formB_ws`／`_dist`／`_ugap`／`_state` 原本斷言 hold 窗非空，改成只斷言下坡窗，`_dist`／`_ugap` 的 hold 漂移量在無 hold 時給 NaN）；
健康（c-only plane／cell min h 1.088／1.067、NaN 0）；包絡不變的理由如上。一階線 `run_pure_simulation`（5state_noap）也跑得動。

`deep`／`shallow` 的形狀都是 hold → descend → oscillate → hold，**帶寬都是 2.22**：
深度是唯一的變數，帶是**平移**不是拉伸。

任何 driver 都吃 `opts.scenario`。要重現 2026-08-19 之前量的數字，
傳 `'shallow'`（已驗逐位相同）。

## 為什麼是 1.10，不能更深也不該更淺

形狀資訊在近壁：`∂ā/∂b` 在遠場趨零，所以停在 w̄ = 2 等於要濾波器從曲線最平的
那一段去辨識它。實測（b 當 state 的臂）：深帶讓 `b̂` 的走行從 0.037 變 0.213，
真值本身的變化 `b_true` 從 0.0094 變 0.0608。

不能更深，因為兩個下限在 1.10 交會，而且**都不是 house value**：
two-sphere 級數只在其上可信，而 `h_min` 也在那裡，所以 prior 包絡不得往下讀。
命令谷底放 1.10 時追蹤誤差已經把粒子帶到 1.085，離接觸還有 190 nm。

## 換帶之後不能做的事

1. **不准跨帶比百分比。** 谷底的 ā 從 0.47 掉到 0.087，分母小 5.4 倍，
   同樣的絕對誤差讀起來大 5.4 倍。跨帶比較只能用 seed 無關的簽名
   （漂移率、誠實比、可觀性比值）。
2. **不准引用 2026-08-19 之前的驗收數字**當作 deep 的基準 —— 它們全部量在 shallow。

## 改軌跡時必跑的四件事

1. `scenario='shallow'` 必須逐位重現改動前（唯一能證明沒有偷改別的東西的方法）
2. 五支 driver 都要能跑起來，不是只跑你在改的那一支
3. 體檢先於驗收：`w̄` 有沒有低於接觸、`a_bar` 撞 floor 幾次、NaN、
   `b̂` 有沒有出 guard —— **看這些，不是先看 RMS**
4. 包絡相依的量要確認真的重推了；若沒變，要能說出**為什麼不該變**
   （例：`floor_a` 兩帶皆 0.03058，因為殘差的 sup 是 w̄ = 2.41 的內部極大，
   兩個包絡都含住它 —— 這是正確行為不是沒接線）

## 深帶已知的連帶效應

- `h_bar_safe` 在 deep 是 **1.0**（shallow 是 1.5 的 house value）。
  1.5 會蓋掉整個近壁半週期，y₂ 全程被閘掉。
- `env_lo` 一律 `max(h_bottom/R − ENV_LO_MARGIN, H_BAR_MIN_PRIOR)`。
- `formB_ws` 的 ∥ 軸向下探測點會被夾在 1.10，該方向的 θ 寬度因此為 0；
  driver 會印出來，**不是靜默歸零**。
- log 用的真值夾制 (`h_bar_floor_drv`) 必須是 1.001（物理下限），
  **不是** `h_min/R`。兩者共用一個數的時候，深帶有 14% 的步數尺被凍住、
  讀數偏高 16%，而模擬的物理完全正確。這個夾制**每支 driver 各有一份**。
- 追蹤誤差在深帶**變好**（RMS 25.8 → 22.0 nm，hold 21.9 → 10.0）：
  近壁阻尼大，布朗位移被壓住。不要預期它變差。
