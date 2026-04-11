---
name: devlog-loop
description: Keep a concise, chronological DEVLOG for meaningful tasks. Use when work should be documented for handoff, traceability, or future debugging.
---

Maintain `.cursor/memory/DEVLOG.md` as the durable engineering timeline for this repository.

## Core Workflow

1. Read latest `.cursor/memory/DEVLOG.md` entries before starting meaningful work.
2. During the task, capture commands/checks and key outcomes.
3. Before handoff, append one new dated entry.

Use `.cursor/memory/DEVLOG.md` as the default log file unless the user requests another path.

## Entry Requirements

Each new entry should include:

- Date/time (local)
- What changed (files or behavior)
- Validation performed (build/tests/runtime checks)
- Follow-ups or risks (if any)

Keep entries concise and factual. Prefer bullet points.

## Quality Rules

- Do not log secrets, tokens, or private credentials.
- Do not fabricate validation; only record checks actually run.
- Keep wording concrete and operational.
- If no code changed, still log what was investigated and what was verified.

## Template

Use `references/devlog-entry-template.md` for consistent structure.

## Maintenance

Keep `.cursor/memory/DEVLOG.md` concise enough that recent engineering state is easy to scan.

When the devlog grows large or the user wants a fresh active log, do not delete old entries. Use archive-first rollover:

1. Determine the active date range covered by the current live devlog.
2. Rename `.cursor/memory/DEVLOG.md` to a dated snapshot such as:
   - `.cursor/memory/DEVLOG_YYYY-MM-DD_to_YYYY-MM-DD.md`
3. Prepend a short `Archived Devlog Summary` at the top of the renamed snapshot that states:
   - the active date window
   - the major work completed in that span
   - the main risks or workstreams preserved there
4. Create a new slim `.cursor/memory/DEVLOG.md` whose first entry records:
   - that rollover occurred
   - where the prior detailed history moved
   - what high-level context was intentionally carried forward
5. Leave older archive files and dated snapshots intact unless the user explicitly requests a deeper archive reorganization.

If scratchpad and devlog are being cleaned up together, prefer rolling both in the same pass so the active working memory and the active engineering timeline stay aligned.
