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
