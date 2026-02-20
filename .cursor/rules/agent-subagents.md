---
description: "Encourage use of Cursor subagents for context isolation + parallel work (explore, bash/log-heavy, verification)."
alwaysApply: true
---

# Subagents (use where applicable)

## Required memory context (always)

- Scratchpad skill: `.cursor/skills/learning-scratchpad-loop/SKILL.md`
- Scratchpad template: `.cursor/skills/learning-scratchpad-loop/references/scratchpad-template.md`
- Scratchpad file: `.cursor/memory/AGENT_SCRATCHPAD.md`
- Devlog skill: `.cursor/skills/devlog-loop/SKILL.md`
- Devlog template: `.cursor/skills/devlog-loop/references/devlog-entry-template.md`
- Devlog file: `.cursor/memory/DEVLOG.md`

Use Cursor **subagents** for tasks that would otherwise bloat the main context or benefit from parallelism.
Reference: [Cursor subagents docs](https://cursor.com/docs/context/subagents).

## When to use subagents

- **Codebase exploration / “where is X?”** across many files or unfamiliar areas.
- **Command-heavy work** (lots of build/install/test output) to keep the main thread clean.
- **Independent verification** after implementation (tests, lint, runtime checks).
- **Parallel workstreams** (e.g., docs updates + code changes + test plan) when they don’t depend on each other.

## How to use subagents (hard rules)

- Provide a **self-contained prompt**: relevant file paths, expected output, constraints, and “definition of done”.
- Ask the subagent to return:
  - the exact files/lines involved,
  - a concise conclusion,
  - any recommended next steps.
- Do not launch many generic subagents; prefer **2–4 focused** ones.

## RTOS/EtherCAT-specific guidance

For RTOS/EtherCAT work, subagents are especially useful for:
- validating the plan section referenced by the current phase (14/15/16/17),
- checking whether an implementation matches the locked invariants (NIC MACs, IgH binding, IPC schema),
- running verification steps and summarizing results.

