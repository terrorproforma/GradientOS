---
description: "Generic gated workflow for agents: intake → plan → minimal diff → correctness → hygiene → verify/report."
alwaysApply: true
---

# Agent Coding Checklist (Gated)

## Required memory context (always)

- Scratchpad skill: `.cursor/skills/learning-scratchpad-loop/SKILL.md`
- Scratchpad template: `.cursor/skills/learning-scratchpad-loop/references/scratchpad-template.md`
- Scratchpad file: `.cursor/memory/AGENT_SCRATCHPAD.md`
- Devlog skill: `.cursor/skills/devlog-loop/SKILL.md`
- Devlog template: `.cursor/skills/devlog-loop/references/devlog-entry-template.md`
- Devlog file: `.cursor/memory/DEVLOG.md`

## Gate 0 — Intake (MUST PASS BEFORE CODING)
- [ ] Restate the task in 1–3 sentences (no invention).
- [ ] Read `.cursor/memory/AGENT_SCRATCHPAD.md` and `.cursor/memory/DEVLOG.md` and apply relevant guardrails/preferences.
- [ ] List explicit inputs/outputs + success criteria.
- [ ] List ALL assumptions (runtime, versions, deps, data shape/units/timezone, auth, concurrency, persistence).
- [ ] For each assumption: mark as (Given) / (Inferred) / (Unknown).
- [ ] If any (Unknown) affects correctness: STOP and ask 1–3 clarifying questions OR present 2–3 options + recommend one.

### STOP conditions (do not proceed)
- Missing requirement that changes behavior.
- Conflicting requirements not resolved.
- Unspecified library/framework choice that affects architecture or deployment.

## Gate 1 — Plan (MUST PASS BEFORE EDITING FILES)
- [ ] Provide a 3–7 step plan.
- [ ] Identify files likely to change.
- [ ] State verification commands (build/test/lint).
- [ ] Define non-goals (what you will NOT change).
- [ ] Identify edge cases + risks.

## Gate 2 — Minimal Diff Implementation
- [ ] Touch the fewest files/lines possible.
- [ ] Do not refactor unrelated code.
- [ ] Do not rename symbols or reformat unless required.
- [ ] Preserve existing comments and intent.
- [ ] If refactor is unavoidable: isolate as separate step/commit and explain why.

## Gate 3 — Correctness Lock
- [ ] Add/extend tests that encode success criteria (preferred BEFORE implementation, when applicable).
- [ ] Define invariants (must-always-be-true properties).
- [ ] Ensure tests fail before fix (when applicable) and pass after.

## Gate 4 — Complexity Control
- [ ] Default to the simplest correct solution.
- [ ] No new abstractions unless a concrete need is proven.
- [ ] Avoid “framework inside a framework” patterns.
- [ ] Prefer explicit code over meta-programming/config indirection.

## Gate 5 — Performance/Robustness (ONLY IF RELEVANT)
- [ ] Start naive-but-correct.
- [ ] Optimize only after tests pass.
- [ ] Preserve behavior; add benchmarks if perf is a requirement.
- [ ] Avoid obvious inefficiencies (N+1 I/O, quadratic loops, repeated parsing).

## Gate 6 — Hygiene (MUST PASS BEFORE DONE)
- [ ] Remove dead code and unused imports/vars/params.
- [ ] No duplicate helpers or parallel APIs.
- [ ] No “temporary” code paths left behind.
- [ ] Docs/comments updated only where behavior changed (no comment churn).

## Gate 7 — Verification + Report (MUST PASS BEFORE DECLARING DONE)
- [ ] Build succeeds.
- [ ] Tests pass.
- [ ] Lint/format checks pass (if repo uses them).
- [ ] Append concise writebacks to `.cursor/memory/AGENT_SCRATCHPAD.md` and `.cursor/memory/DEVLOG.md`.
- [ ] Provide a short change log:
  - What changed
  - Why
  - How to verify (exact commands)
  - Known risks/edge cases
- [ ] Confirm diff matches request and avoids collateral changes.

---

# Pre-flight prompt (paste above new agent sessions)

Before coding: read `.cursor/memory/AGENT_SCRATCHPAD.md` + `.cursor/memory/DEVLOG.md`, restate task, list assumptions, ask questions for unknowns, propose a 3–7 step plan, define DONE with tests/invariants, then implement minimal diff. No unrelated refactors. Verify with build/tests, append scratchpad/devlog writebacks, and report exact commands.

