# 雙機開發規約(Mac ↔ Windows)

> 兩機透過雙通道同步:git(程式/文件,手動)+ Obsidian ClaudeVault(記憶,自動)。
> 本檔規定「必須怎麼做」;「發生過什麼」寫 auto-memory,不寫這裡。

1. **開工必 pull**:session 第一個動作 = `git fetch` + `git pull`;pull 完成前不修改任何 repo 檔案。
2. **收工落地**:功能完整且測試通過 → 當日 commit + push;實驗性或未完成的改動可留在工作樹,但必須在 memory 記錄狀態與未 commit 檔案清單。不留無記錄的工作樹。
3. **熱點檔**(`model/controller/*.m`、`model/dual_track/run_pure_simulation.m`):動工前 pull、push 前再 pull;同一時段避免兩機同時修改,以「已 push」作為交接訊號。
4. **Production bug 不過夜**:發現即修,同一 session 內 commit + push(防止兩機重複修復同一個 bug)。
5. **Knob/變數命名**:新增 controller knob 或 config 欄位前,先查 MEMORY.md 近期另一機的條目,避免同義異名。
6. **文件 SSOT**:每個主題以唯一一份 .tex 為準(基礎鏈 = `4state_del_hd.tex`;taylor gain = `4state_taylor_gain.tex`;powerlaw = `5state_powerlaw_hd.tex`;expgain = `5state_expgain_hd.tex`;兩形式對照 = `reference/eq17_analysis/shape_ledger.md`)。要修改另一機建立的 SSOT,先在 memory 留異議記錄再動手。
7. **Memory 慣例**:MEMORY.md 每條一行、標機器與日期;跨機待辦寫在「待辦方向」區並指定執行機器;定期檢查 Obsidian conflicted copies。
8. **機器專屬設定不進 repo**:`.claude/settings.local.json` 有進版控、兩機共用,所以裡面
   **只放跨平台的東西**(permissions、`autoMemoryDirectory`)。音效/通知這類綁 OS 的 hook
   一律放各機自己的 user settings(`~/.claude/settings.json`),repo 內不留。
   來源:曾有 `powershell` 的 Stop/Notification hook 寫在專案設定裡,Mac 每次收尾都噴
   `/bin/sh: powershell: command not found`(而 Notification 指向的 `.claude\notification.ps1`
   repo 內根本不存在,Windows 側也是壞的)。Mac 的 Tink 音效本來就在 user settings,是正確位置。

   Windows 若要恢復 chime,貼到 `%USERPROFILE%\.claude\settings.json` 的 `hooks`:
   ```json
   "Stop": [
     { "hooks": [ { "type": "command",
       "command": "powershell -c \"(New-Object Media.SoundPlayer 'C:\\Windows\\Media\\chimes.wav').PlaySync()\"" } ] }
   ]
   ```
