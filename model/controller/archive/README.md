# model/controller/archive/

已否證／被取代研究線的 controller（2026-07-28 整線下沉，對照表
`reference/eq17_analysis/archive/MOVED.md`）。對應測試在
`test_script/integration/archive/` 與 `test_script/unit_tests/archive/`。

重跑歷史線：`addpath` 本目錄與對應 test archive 目錄即可（shared driver
的 variant 分支已移除，這些 controller 需由各自 archived 測試直接呼叫）。
