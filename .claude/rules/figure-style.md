# 圖表風格規範

- no grid、no title（統計數字印到 console，不放圖上）
- `box on`；legend `northoutside` horizontal
- 顏色：True = 紅、Estimate = 藍、Measured = 淺藍
- controller 時序圖 FontSize 18 bold
- 輸出：`exportgraphics(..., 'Resolution', 150)`
- canonical 範本：`test_script/integration/plot_var_ahat_6state.m`（新圖從它抄起，勿憑記憶重建）
- 圖 caption 一句話

# 模擬對照的呈現義務（2026-08-12 使用者裁示）

**任何被拿來對比的模擬情境,都必須出圖、開圖給使用者,並對照圖說明結果。**

- 只報表格數字不算完成。數字說「差多少」,圖說「差在哪裡、什麼時候、長什麼樣」
- 情境有 N 個就要有 N 組圖;兩個臂對比就左右並列、共用 y 軸,不要讓人翻兩份 PDF 比對
- 產完要用 `open` 打開,並在回覆裡**逐圖指出該看哪一列、看到什麼**,不是只給路徑
- 分析要對到圖上看得到的東西。若某個結論在圖上看不出來(例如它是統計量),要明講「這條要看 console 不是看圖」

# 圖中 notation（2026-08-30 使用者裁示，必檢項）

- 軸標、legend、title 的符號必須與 tex SSOT 逐一對得上；出圖前 grep `ylabel|xlabel|legend|title` 核對，改完重出圖、`open` 逐圖確認
- bar = 已除以尺：w̄ = w/R、ā = a/a_nom、f̄ = a_o f。tex interpreter 畫不出 bar 時，改寫成比值：`a_z / a_{nom}`（a_nom = Ts/γ_N [um/pN]）、`R \delta w_3 [\mum]`；或用 `'Interpreter','latex'` 寫 `$\bar{a}_z$`
- **禁用 `a_z / a_o`**：a_o = Ts/(γ_N R) 是 1/pN，物理增益除它得到 um，不是無因次（2026-08-30 三支 plot 已改）
- hat 一律戴在無因次量上（ā̂、b̂、δw̄̂）；物理量（a、f、p）不戴 hat
- caption 一句話講清楚尺是什麼（例：「增益以遠場 a_nom 正規化」）
- 細則與五個單位邊界點位：`.claude/rules/normalization-boundary.md`
