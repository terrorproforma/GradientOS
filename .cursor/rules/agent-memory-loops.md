---
description: "Always enforce scratchpad + devlog loops for every meaningful task."
alwaysApply: true
---

# Agent Memory Loops (Mandatory)

For every meaningful task in this repository, apply both loops:

- Scratchpad loop via `.cursor/memory/AGENT_SCRATCHPAD.md`
- Devlog loop via `.cursor/memory/DEVLOG.md`

## Skill and file mapping (required)

- Scratchpad skill: `.cursor/skills/learning-scratchpad-loop/SKILL.md`
- Scratchpad template: `.cursor/skills/learning-scratchpad-loop/references/scratchpad-template.md`
- Scratchpad file: `.cursor/memory/AGENT_SCRATCHPAD.md`
- Devlog skill: `.cursor/skills/devlog-loop/SKILL.md`
- Devlog template: `.cursor/skills/devlog-loop/references/devlog-entry-template.md`
- Devlog file: `.cursor/memory/DEVLOG.md`

## Start-of-task requirements

1. Read `.cursor/memory/AGENT_SCRATCHPAD.md` and `.cursor/memory/DEVLOG.md` before making substantial changes.
2. Extract and apply:
   - explicit user preferences
   - repeated mistakes to avoid
   - proven checks/workflows
3. If historical notes conflict with current user instructions, prioritize current user instructions and record the update.

## During-task requirements

1. Capture high-signal issues/learnings as they occur (do not defer all notes to the end).
2. Keep notes concrete and operational (files, commands, outcomes).
3. Never record secrets or credentials.

## End-of-task requirements

Before handoff, append concise entries to both files:

- `.cursor/memory/AGENT_SCRATCHPAD.md`: task summary, mistakes/fixes, user preferences, guardrails, risks/follow-ups.
- `.cursor/memory/DEVLOG.md`: date/time, what changed, validation actually run, risks/follow-ups.

If no code changed but meaningful investigation happened, still log what was checked and verified.
