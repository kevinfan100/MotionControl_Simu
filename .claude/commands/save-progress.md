---
description: Save current work progress with auto-generated analysis and WIP commit
---

# Save Progress Command

You are assisting the user to save their current work progress before taking a break or ending the work session.

## Your Task

Execute the following steps to help the user save their progress:

### Step 1: Analyze Current Status

1. Check the current Git branch
2. Check for uncommitted changes (git status)
3. List modified, added, and deleted files
4. Check for temporary test files (temp_*.m)
5. Get the last commit message and time

### Step 2: Auto-Generate Progress Record

Based on the file changes, automatically generate a detailed progress record including:

**Completed Parts:**
- For NEW files: Analyze filename and infer functionality (e.g., "pi_batch_sweep.m" → "Added batch frequency sweep functionality")
- For MODIFIED files: Check line changes to infer what was done
- Use ✅ for completed features, ⏸️ for work in progress

**File Changes Analysis:**
- For each new file: estimate lines of code, infer purpose from filename
- For each modified file: show line changes (+XX lines), infer main changes
- Flag any temporary files with ⚠️

**Testing Status (Auto-Infer):**
- IF temp_*.m files exist → ⏸️ Testing in progress
- IF new features added but no test files → ⬜ Testing not started
- Otherwise → ⏸️ Testing in progress (default assumption)

**Next Steps (Auto-Suggest):**
- Based on new features → Suggest testing steps
- If temp files found → Suggest cleanup or integration
- If large files (>400 lines) → Suggest refactoring consideration
- Check for TODO comments in code → Add to next steps

**Potential Issues (Auto-Detect):**
- temp_*.m files → Remind to handle temporary files
- Large files (>400 lines) → Suggest refactoring
- New algorithm additions → Suggest documentation

**Commit Message (Auto-Generate):**
Format:
```
WIP({controller}): {Main feature description}

Features:
- [List main features added/modified]

Testing:
- [Based on testing status]

Next:
- [Based on suggested next steps]
```

### Step 3: Present Record to User

Show the complete auto-generated record in a well-formatted markdown block.

Then ask:
```
❓ 這份記錄正確嗎？
  1. ✅ 正確，直接使用
  2. ✏️ 需要修改或補充
  3. ❌ 不正確，重新輸入
```

### Step 4: Handle User Response

**If user chooses 1 (Correct):**
- Skip to Step 6

**If user chooses 2 (Modify):**
Ask which parts to modify:
```
❓ 要修改哪個部分？（可選多個，用逗號分隔）
  1. 完成部分
  2. 測試狀態
  3. 下一步建議
  4. 問題與筆記
  5. Commit 訊息
  6. 全部重新編輯
```

For each selected part:
- Show current content
- Ask for user's modification
- Update the record
- Show the updated section

**If user chooses 3 (Incorrect):**
- Ask user to provide complete information manually

### Step 5: Handle Temporary Files

If temp_*.m files were detected, ask:
```
❓ 發現臨時測試檔案，如何處理？
  [列出每個 temp_*.m 檔案]

  對於每個檔案：
    1. 保留（會被 commit）
    2. 刪除
    3. 稍後決定
```

### Step 6: Choose Save Method

Ask:
```
如何保存這次進度？
  1. 💾 Commit (WIP) - 建立工作進度提交並推送
  2. 📝 只記錄不 commit - 保持檔案未提交狀態
  3. 🎯 重新檢視記錄
```

**If option 1 (Commit WIP):**
1. Stage all code changes: `git add .` (excludes progress record which doesn't exist yet)
2. Commit with the generated message
3. Get commit hash: `git log -1 --format="%h"`
4. Save record to `.claude/progress/{branch_name}.md` with actual commit hash (replace slashes with underscores)
5. Stage progress record: `git add .claude/progress/{branch_name}.md`
6. Create documentation commit: `git commit -m "docs({scope}): Update progress log with {feature}"`
   - {scope} should be the controller type (pi/r/type3) or "project" for general changes
   - {feature} should be a brief description of what was completed
7. Push to origin (pushes both commits)
8. Show success summary

**If option 2 (Record only):**
1. Save record to `.claude/progress/{branch_name}.md`
2. Do not modify git status
3. Show success message

**If option 3 (Review):**
- Go back to Step 3

### Step 7: Show Summary

Display:
```
✅ 完成！

已保存記錄：.claude/progress/{branch_name}.md
[If committed] Git commit: {commit_hash}
[If pushed] 已推送到 GitHub

完成部分：{count} 項
測試狀態：{status}
下一步：{count} 項待辦

您可以安心下班了！明天使用 /resume 繼續工作 😊
```

## Important Guidelines

1. **Be Proactive**: Auto-generate as much content as possible from code analysis
2. **Be Smart**: Infer functionality from filenames, line changes, and code structure
3. **Be Concise**: User should only need 0-2 interactions if auto-generation is accurate
4. **Use Emojis**: Make status clear with ✅ ⏸️ ⬜ ❌ ⚠️ 💡
5. **Follow Conventions**: Respect the project's commit message format and naming rules
6. **Detect Patterns**: Learn from previous progress records to improve inference

## Record File Format

Save to: `.claude/progress/{branch_name}.md`

Format:
```markdown
# {Controller} Controller Development Progress

## YYYY-MM-DD HH:MM - {Feature Name}

### Completed Parts
- ✅ {Item 1}
- ✅ {Item 2}
- ⏸️ {Item 3} (in progress)

### File Changes
**New Files:**
- `path/to/file.m` (XXX lines)
  Purpose: {inferred purpose}

**Modified Files:**
- `path/to/file.m` (+XX lines)
  Main changes: {inferred changes}

### Testing Status
{emoji} {Status description}
- Tested: {what was tested} ✅
- Pending: {what needs testing} ⬜

### Next Steps
- [ ] {Task 1}
- [ ] {Task 2}

### Issues & Notes
⚠️ **Attention:**
- {Issue 1}

💡 **Ideas & Notes:**
- {Note 1}

### Git Commit
`{commit_hash}` - {commit message}

---

```

## Context: Project Information

- Project: Flux Controller Simulation
- MATLAB Version: R2025b
- Controllers: PI, Type3, R
- Branch structure: master, develop, controller/pi, controller/r
- Naming convention: Temporary files use `temp_*` prefix
- Commit format: Conventional Commits (feat/fix/refactor/WIP)
