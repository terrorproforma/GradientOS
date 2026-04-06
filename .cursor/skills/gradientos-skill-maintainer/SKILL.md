---
name: gradientos-skill-maintainer
description: Maintains the project GradientOS skill corpus as a slow-moving living SOP. Use when the user asks to update or refine the GradientOS skill, consolidate architecture lessons from completed work, compare scratchpad/devlog learnings against the canonical SOP, or decide whether a new pattern is stable enough to become part of the shared skill set.
---

# GradientOS Skill Maintainer

Use this skill when maintaining the canonical GradientOS skill set.

## Purpose

Keep `.cursor/skills/gradientos-sop/` accurate, concise, and stable without turning it into a mirror of daily implementation churn.

## Read First

1. `.cursor/skills/gradientos-sop/SKILL.md`
2. `.cursor/skills/gradientos-sop/skill-maintenance-policy.md`
3. The relevant `gradientos-sop` reference file for the subsystem that changed
4. The latest `.cursor/memory/AGENT_SCRATCHPAD.md` entries
5. The latest `.cursor/memory/DEVLOG.md` entries
6. Any current architecture/SOP document the skill is derived from, especially `.cursor/skills/gradientos-sop/RTOS_ETHERCAT_MASTER_OPERATING_PRINCIPLES.md`

## Decision Gate

Update the canonical GradientOS skill only if at least one condition holds:

- architecture or ownership boundaries changed
- a workstream is complete and validated enough to become standard practice
- the same rule keeps recurring in scratchpad/devlog
- the canonical skill materially diverged from current code and SOP reality

If none of those hold, keep the change in scratchpad/devlog only.

## User Confirmation Rule

Unless the user already explicitly requested a canonical skill update, ask whether they want the new information consolidated into the GradientOS skill set.

Use language like:

- `This looks stable enough to promote into the GradientOS skill. Do you want me to consolidate it now?`
- `This is still implementation-level and probably belongs only in scratchpad/devlog for now.`

## Update Workflow

1. Identify which subsystem reference file should change.
2. Extract only the stable guidance from scratchpad/devlog and current code/docs.
3. Rewrite it as concise operational rules, not chat history.
4. Update the smallest relevant skill file instead of broadening every file.
5. Preserve the distinction between:
   - fast-moving execution memory
   - chronological engineering evidence
   - slow-moving canonical SOP guidance

## Consolidation Rules

- Do not copy large devlog or SOP prose blocks into the skill files.
- Do not canonize temporary workarounds until they are validated.
- Do not add vendor-specific rules to generic references when they belong in profile-specific guidance.
- Keep the root `gradientos-sop` `SKILL.md` compact; push detail into the targeted reference file.
- Keep references one level deep and terminology consistent.

## Good Outcomes

- The canonical skill remains short, discoverable, and reusable.
- Scratchpad/devlog continue to absorb the frequent churn.
- New stable architecture or safety rules are eventually promoted into the shared skill set.
