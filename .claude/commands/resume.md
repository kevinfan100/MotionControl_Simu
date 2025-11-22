---
description: Resume work by reviewing previous progress from yesterday or recent days (project)
---

# Resume Command - Auto-Sync Version

You are assisting the user to resume their work by **automatically synchronizing with remote** and reviewing their previous progress.

## Your Task

Help the user quickly get back into their work context with automatic remote synchronization and progress display.

---

## Step 1: Show Current Branch

Display:
```
╔════════════════════════════════════════╗
║  🔄 Resume - 恢復工作環境             ║
╚════════════════════════════════════════╝

📍 當前分支：{branch_name}
```

Execute:
```bash
git branch --show-current
```

---

## Step 2: Auto-Sync Remote (🆕 Automated)

**IMPORTANT: Always execute synchronization automatically. Do not ask permission first.**

### 2.1 Fetch Remote Updates

Execute automatically:
```bash
git fetch origin {branch_name}
```

Display while fetching:
```
🔄 正在同步遠端...
```

**If fetch fails (no network):**
```
⚠️ 無法連線到遠端，使用本地狀態繼續

[Continue to Step 3]
```

### 2.2 Analyze Current Status

Check these values:
```bash
# Count commits remote is ahead
remote_ahead=$(git rev-list HEAD..origin/{branch} --count)

# Count commits local is ahead (unpushed)
local_ahead=$(git rev-list origin/{branch}..HEAD --count)

# Check for uncommitted changes
uncommitted=$(git status --porcelain)

# Get files changed in remote
remote_files=$(git diff --name-only HEAD origin/{branch})

# Get files changed locally
local_files=$(git diff --name-only)
```

### 2.3 Auto-Handle Based on Scenario

**Scenario A: Local is already up-to-date**
```
Condition: remote_ahead == 0

Action: Display and continue
```
Display:
```
✅ 已是最新狀態

─────────────────────────────────────
```

**Scenario B: Remote has updates + Local is clean**
```
Condition:
  - remote_ahead > 0
  - local_ahead == 0
  - no uncommitted changes

Action: Auto pull without asking
```

Execute:
```bash
git pull origin {branch} --ff-only
```

Display:
```
📥 自動拉取 {remote_ahead} 個新 commit...

{show each commit: hash - message (time_ago)}

✅ 已更新到最新狀態

─────────────────────────────────────
```

**Scenario C: Remote has updates + Local has uncommitted changes**
```
Condition:
  - remote_ahead > 0
  - has uncommitted changes

Action: Analyze conflict risk and propose solution
```

**Step C.1: Analyze Conflict Risk**

```bash
# Get overlapping files
remote_files=$(git diff --name-only HEAD origin/{branch})
local_files=$(git status --porcelain | awk '{print $2}')
overlap=$(comm -12 <(echo "$remote_files" | sort) <(echo "$local_files" | sort))
```

**Step C.2: Display Analysis**

Display:
```
╔════════════════════════════════════════╗
║  🤖 自動衝突評估                      ║
╚════════════════════════════════════════╝

📊 狀態分析：
  遠端新增：{remote_ahead} 個 commit
  本地修改：{count_local_files} 個檔案
  重疊檔案：{count_overlap} 個

{if overlap exists:}
重疊檔案詳情：
  📄 {file1}
  📄 {file2}
  ...

衝突風險：{if overlap: "中 ⚠️" else: "低 ✅"}

💡 建議方案：
  1. 暫存本地變更 (git stash)
  2. 拉取遠端更新 (git pull)
  3. 恢復本地變更 (git stash pop)
  {if overlap: "4. 手動解決衝突（如果發生）"}

⚡ 執行此方案？（3 秒後自動執行）
  1. 是 (Enter 或等待)
  2. 否，我要手動處理
```

**Step C.3: Wait for Input with Timeout**

Wait for user input with 3-second timeout. Default choice is "1" (execute).

**Step C.4: Execute Stash-Pull-Pop Strategy**

If user chooses "1" or timeout:

```bash
# Step 1: Stash local changes
git stash push -m "AutoStash before resume sync at $(date)"
```
Display: `📦 已暫存本地變更`

```bash
# Step 2: Pull remote updates
git pull origin {branch} --ff-only
```
Display:
```
📥 已拉取遠端更新

{show pulled commits}
```

```bash
# Step 3: Pop stash
git stash pop
```

**If stash pop succeeds:**
Display: `✅ 已恢復本地變更，無衝突`

**If stash pop has conflicts:**
Display:
```
⚠️ 恢復時發生衝突

衝突檔案：
  {list conflicted files}

🔧 解決方案：
  1. 保留遠端版本（放棄本地修改）
  2. 保留本地版本（忽略遠端更新）⚠️ 不推薦
  3. 手動合併（推薦）- 使用 VSCode diff editor

您的選擇？[1/2/3]
```

Handle based on user's choice:
- Choice 1: `git checkout --theirs {files} && git add {files}`
- Choice 2: `git checkout --ours {files} && git add {files}`
- Choice 3: `git status` and guide user to resolve manually

If user chooses "2" (manual handling):
Display:
```
⏸️ 已暫停自動同步

您可以手動執行：
  1. git stash
  2. git pull origin {branch}
  3. git stash pop

要繼續查看進度記錄嗎？
  1. 是
  2. 否，我先處理 Git
```

**Scenario D: Both remote and local have new commits (Divergence)**
```
Condition:
  - remote_ahead > 0
  - local_ahead > 0

Action: Suggest rebase
```

Display:
```
╔════════════════════════════════════════╗
║  🤖 分支分歧評估                      ║
╚════════════════════════════════════════╝

📊 狀態：
  遠端領先：{remote_ahead} 個 commit
  本地領先：{local_ahead} 個 commit

遠端新 commit：
{git log origin/{branch} -n {remote_ahead} --oneline}

本地新 commit：
{git log HEAD ^origin/{branch} --oneline}

💡 建議方案：Rebase（變基）
  將本地 commit 移到遠端最新 commit 之後

  效果示意：
    遠端：A -- B -- C -- D
    本地：A -- B -- C -- E
                        ↓ rebase
    結果：A -- B -- C -- D -- E'

⚡ 執行 rebase？（5 秒後自動執行）
  1. 是 (Enter 或等待)
  2. 否，我要手動處理
```

Wait for user input with 5-second timeout. Default choice is "1".

If user chooses "1" or timeout:
```bash
git pull --rebase origin {branch}
```

**If rebase succeeds:**
Display: `✅ Rebase 完成，分支已同步`

**If rebase has conflicts:**
Display:
```
⚠️ Rebase 時發生衝突

衝突檔案：
  {list files}

📝 解決步驟：
  1. 解決衝突檔案
  2. git add {resolved_files}
  3. git rebase --continue

或者放棄 rebase：
  git rebase --abort

要繼續查看進度記錄嗎？
  1. 是（帶著衝突繼續）
  2. 否，我先解決衝突
```

---

## Step 3: Interactive Time Range Selection

After sync is complete, ask:

```
要查看哪段時間的進度？
  1. 昨天的進度
  2. 最近 3-5 天的進度
```

Wait for user's choice.

---

## Step 4: Load and Display Progress

**If user chooses 1 (Yesterday):**

1. Read `.claude/progress/{branch_name}.md`
2. Find entries from yesterday (compare dates)
3. Display in format:

```
=== 昨天 (YYYY-MM-DD) 的進度 ===

📍 {branch_name} - HH:MM
功能：{Feature Name}

完成部分：
  ✅ {Completed item 1}
  ✅ {Completed item 2}
  ⏸️ {In-progress item}

下一步要做：
  ⬜ {Todo 1}
  ⬜ {Todo 2}

問題筆記：
  ⚠️ {Issue 1}
  ⚠️ {Issue 2}

測試狀態：
  {Testing status description}
```

**If user chooses 2 (Last 3-5 days):**

1. Read `.claude/progress/{branch_name}.md`
2. Find entries from last 5 days
3. Display in chronological format:

```
=== {branch_name} 最近 5 天進度 ===

📅 YYYY-MM-DD (X 天前)
  • {Feature} - {Brief status}
  待辦：{Pending items summary}
  [If committed] Commit: {commit_hash} - {message}

📅 YYYY-MM-DD (X 天前)
  • {Feature} - {Brief status}
  待辦：{Pending items summary}
  [If committed] Commit: {commit_hash} - {message}

[... more days ...]

─────────────────────────────────────
當前待辦事項（跨日期）：
  ⬜ {Todo 1} (started YYYY-MM-DD)
  ⬜ {Todo 2} (started YYYY-MM-DD)
```

---

## Step 5: Check Current Git Status

After showing progress history, check current status:

```bash
git status --short
git log -1 --format="%h (%ar) - %s"
```

Display:
```
當前未提交變更：{count} 個檔案
[If changes exist]
  M {file1}
  M {file2}
  ?? {file3}

最近 commit：
  {hash} ({time_ago}) - {message}
```

---

## Step 6: Suggest Next Steps

Based on the progress record, intelligently suggest:

```
根據您的記錄，建議：

1. 📝 {Specific task from "next steps"}
2. 🔍 {Another specific task}
3. 📊 {Another suggestion}

要開始工作了嗎？需要我提醒什麼嗎？
```

---

## Step 7: Optional - Quick Actions

Optionally ask if user wants quick assistance:

```
需要協助嗎？
  1. 查看某個檔案
  2. 執行測試腳本
  3. 查看 commit 歷史
  4. 不需要，開始工作
```

Handle based on user's choice.

---

## Edge Cases

### If No Progress Record Found

If `.claude/progress/{branch_name}.md` doesn't exist:

```
📭 此分支沒有進度記錄

讓我檢查 Git 歷史...

最近 3 次 commit：
  {hash} ({time_ago}) - {message}
  {hash} ({time_ago}) - {message}
  {hash} ({time_ago}) - {message}

當前狀態：
  未提交變更：{count} 個檔案

建議：使用 /save-progress 建立進度記錄
```

### If No Recent Progress (>7 days)

```
⏰ 最近一次進度記錄是 {X} 天前

要查看嗎？
  1. 是，顯示上次進度
  2. 否，顯示 Git commit 歷史
```

### If WIP Commit Found

If the last commit message contains "WIP":

```
⚠️ 上次進度是 WIP commit

WIP Commit 內容：
{commit message body}

這個功能還需要繼續嗎？
```

---

## Important Guidelines

1. **Auto-Sync First**: Always fetch and sync before showing progress
2. **Smart Defaults**: Use timeouts (3-5 seconds) for automatic execution
3. **Risk Assessment**: Automatically analyze file overlaps and conflict risk
4. **Safe Operations**: All auto-operations are reversible (stash, rebase)
5. **Clear Communication**: Show what's happening in real-time with emojis
6. **User Control**: Always provide "manual handling" option for advanced users
7. **Be Contextual**: Show most relevant information based on time away
8. **Cross-Reference**: Link progress records with Git history

---

## File Reading

Read from: `.claude/progress/{branch_name}.md`

Expected format (from save-progress):
```markdown
# {Controller} Controller Development Progress

## YYYY-MM-DD HH:MM - {Feature Name}

### Completed Parts
...

### Next Steps
- [ ] Task 1
- [ ] Task 2
...
```

Parse intelligently:
- Extract dates from `## YYYY-MM-DD` headers
- Extract next steps from `### Next Steps` section
- Extract issues from `### Issues & Notes` section
- Extract testing status from `### Testing Status` section

---

## Context: Project Information

- Project: Flux Controller Simulation
- MATLAB Version: R2025b
- Controllers: PI, Type3, R
- Branch structure: master, develop, controller/pi, controller/r
- Working hours assumption: Recent work is typically within last 1-7 days
- Network: Assume network is always available (per user requirement)
- Priority: Always update to latest remote version (per user requirement)
