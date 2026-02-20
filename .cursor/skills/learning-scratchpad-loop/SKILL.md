---
name: learning-scratchpad-loop
description: Maintain a repo-local learning scratchpad that compounds improvements across sessions by recording mistakes, user corrections, preferences, what worked, and what failed. Use when a user asks for persistent self-improvement, memory across sessions, agent notes, a scratchpad, or a workflow where Codex should adapt based on prior errors and feedback.
---

Build and maintain a lightweight learning loop in-repo so each session starts smarter than the last one.

## Core Workflow

1. Initialize the scratchpad.
2. Read it at session start.
3. Apply it as preflight guardrails.
4. Log high-signal learnings immediately during work.
5. Re-read before risky operations.
6. Append concrete learnings before handoff.

Use `.cursor/memory/AGENT_SCRATCHPAD.md` as the default memory file unless the user asks for a different path.

## Step 1: Initialize Scratchpad

If `.cursor/memory/AGENT_SCRATCHPAD.md` does not exist, create it from `references/scratchpad-template.md`.

Keep the structure stable so future sessions can parse it quickly.
Set and record a file policy in the scratchpad:

- `COMMITTED` when the team should share learned patterns
- `IGNORED` when the scratchpad should remain local-only

## Step 2: Session Start Read

At the beginning of each meaningful task:

1. Read the latest entries in `.cursor/memory/AGENT_SCRATCHPAD.md`.
2. Extract:
   - repeated mistakes to avoid
   - explicit user preferences
   - validated working patterns
   - open risks that affect current work
3. Convert those into a short preflight checklist and follow it.

If current user instructions conflict with old notes, prioritize current user instructions and record the change.

## Step 3: During-Task Capture

Capture and write high-signal items immediately when discovered:

- Mistake made
- How it was detected
- User correction (if any)
- Corrective rule for next time

Do not wait for end-of-task if the insight changes current behavior.
Do not log generic statements such as "be careful" or "improve quality."

Use source tags on every operational note:

- `[self]` for agent mistakes and self-corrections
- `[user]` for user corrections and explicit preferences
- `[tool]` for environment/tooling surprises

## Step 4: Risky-Step Checkpoint

Before risky operations, re-read the scratchpad and apply relevant guardrails.

Risky operations include:

- migrations or dependency upgrades
- wide refactors across multiple files
- unfamiliar scripts or tooling paths
- commands with deletion/overwriting potential

## Step 5: End-of-Task Writeback

Before handoff, append one new session entry with:

1. Task summary
2. Mistakes and fixes
3. User preferences learned or reinforced
4. What worked well
5. What failed or was inefficient
6. Guardrails for next session
7. Follow-ups or unresolved risks

Keep every point concrete and testable. Reference files, commands, and checks when relevant.

## Quality Rules

- Write facts, not guesses.
- Prefer short bullets over paragraphs.
- Mark superseded guidance as superseded instead of deleting history.
- Never store secrets or tokens.
- Keep the scratchpad scoped to execution quality, workflow, and user preferences.

## Maintenance

Consolidate when either threshold is hit:

- every 5-10 sessions
- file grows beyond ~150 lines

Keep the file under ~200 lines of high-signal content.

During consolidation, compress older entries into a short "Retained Lessons" section while preserving:

- recurring user preferences
- critical failure patterns
- proven checks that prevent regressions

Use `references/scratchpad-template.md` as the canonical section layout.
