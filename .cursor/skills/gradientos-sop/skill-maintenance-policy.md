# Skill Maintenance Policy

Use this file when deciding whether new information belongs in the canonical GradientOS skill set or only in the scratchpad/devlog.

## Memory Layers

- `.cursor/memory/AGENT_SCRATCHPAD.md`
  Fast-moving execution memory for mistakes, user corrections, guardrails, and what worked or failed.
- `.cursor/memory/DEVLOG.md`
  Durable chronological record of changes, validation, and follow-up risks.
- `.cursor/skills/gradientos-sop/*`
  Slow-moving canonical SOP guidance for stable architecture and validated operating patterns.

## Canonical Update Threshold

Update the GradientOS skill set only when at least one of these is true:

- architecture or ownership boundaries changed
- a recurring pattern is now clearly standardized
- a workstream is complete and sufficiently reviewed/tested
- the SOP skill has materially diverged from current code reality

## Do Not Update Canonical Skill For

- temporary experiments
- partially implemented refactors
- unvalidated debugging hunches
- one-off findings that are not yet stable operating guidance
- implementation details that belong only in a devlog entry

## Required Decision Gate

Before updating the canonical skill, check:

1. Has the architecture or ownership changed?
2. Is the workstream complete and validated?
3. Is the pattern recurring or important enough to deserve canonical status?
4. Has the user approved consolidation, or explicitly asked for a skill/SOP update?

If any answer is `no`, prefer scratchpad/devlog only.

## Update Workflow

1. Read the current `gradientos-sop` skill files.
2. Read the latest `.cursor/memory/AGENT_SCRATCHPAD.md` entries.
3. Read the latest `.cursor/memory/DEVLOG.md` entries.
4. Identify stable rules that changed the architecture, ownership, safety model, or standard workflow.
5. Ask the user whether to consolidate those changes into the canonical skill unless they already requested it.
6. Keep the canonical skill concise; summarize stable rules rather than mirroring the entire SOP or devlog.

## Maintenance Style

- Prefer updating the smallest relevant reference file instead of editing every file.
- Keep `SKILL.md` compact and routing-focused.
- Keep references one level deep.
- Reuse the same terminology across the skill set.
- Preserve the distinction between profile-specific details and generic stack guidance.
