# 圖表風格規範

- no grid、no title（統計數字印到 console，不放圖上）
- `box on`；legend `northoutside` horizontal
- 顏色：True = 紅、Estimate = 藍、Measured = 淺藍
- controller 時序圖 FontSize 18 bold
- 輸出：`exportgraphics(..., 'Resolution', 150)`
- canonical 範本：`test_script/integration/plot_var_ahat_6state.m`（新圖從它抄起，勿憑記憶重建）
- 圖 caption 一句話
