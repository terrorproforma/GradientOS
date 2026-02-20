---
description: "Ambiguity triggers (red flags) and required agent response: force clarity before architecture-breaking choices."
alwaysApply: true
---

# Ambiguity Triggers (Red Flags) + Required Agent Response

## Required memory context (always)

- Scratchpad skill: `.cursor/skills/learning-scratchpad-loop/SKILL.md`
- Scratchpad template: `.cursor/skills/learning-scratchpad-loop/references/scratchpad-template.md`
- Scratchpad file: `.cursor/memory/AGENT_SCRATCHPAD.md`
- Devlog skill: `.cursor/skills/devlog-loop/SKILL.md`
- Devlog template: `.cursor/skills/devlog-loop/references/devlog-entry-template.md`
- Devlog file: `.cursor/memory/DEVLOG.md`

## A) Scope Vagueness (means requirements are missing)
**Phrases**
- “just”, “quickly”, “simple”
- “obvious”, “you know what I mean”
- “same as before”, “like we discussed”
- “standard”, “best practice”, “modern”
- “make it work”, “fix it”, “improve performance”
- “production-ready”, “robust”, “secure”
- “clean up”, “refactor”, “tidy”
- “optimize”, “make it faster”

**Agent must**
- Ask: “What does DONE mean?” using concrete acceptance criteria:
  - exact behavior change
  - constraints (latency, memory, cost)
  - environment (versions, OS, cloud)
  - success tests
- Or present 2–3 interpretations + recommend one.

## B) Hidden Constraints (likely to cause wrong assumptions)
**Phrases**
- “in our stack”, “in prod”, “in the pipeline”
- “works with existing system”
- “keep backwards compatible”
- “don’t break anything”
- “minimal changes”
- “no downtime”
- “must be safe/secure”

**Agent must**
- Explicitly enumerate compatibility targets:
  - API contracts, schema versions, migrations
  - rollout strategy
  - allowed deps/versions
- Confirm which constraints are real vs aspirational.

## C) Library/Framework Ambiguity (architecture fork)
**Phrases**
- “use whatever”, “pick a library”
- “add auth”, “add caching”, “add queue”
- “agent swarm”, “MCP”, “browser tool”

**Agent must**
- Do not choose silently.
- Provide a short matrix (2–3 options) with tradeoffs + recommend one.

## D) Conflicting Requirements (must be surfaced)
**Phrases**
- “minimal diff” + “refactor”
- “fast” + “correct” with no tolerance
- “no new deps” + “use X”
- “keep behavior” + “change output format”
- “deterministic” + “use randomness/LLM”

**Agent must**
- State the conflict explicitly.
- Ask for priority ordering or propose a resolution.

## E) Risky “Cleanup” (collateral damage hazard)
**Phrases**
- “while you’re there”
- “feel free to improve”
- “make it nicer”
- “remove weird code”
- “reformat”

**Agent must**
- Treat as separate task.
- Require explicit approval for any non-functional change.
- Keep refactors isolated.

## F) Performance Claims Without Measurement
**Phrases**
- “it’s slow”
- “make it faster”
- “reduce latency”

**Agent must**
- Ask: “Slow where?” and require at least one:
  - baseline metric
  - profiling output
  - representative workload
- If unavailable: add lightweight instrumentation first.

## G) Safety/Security Requirements Without Definition
**Phrases**
- “secure”, “privacy-friendly”, “compliant”
- “sanitize inputs”
- “safe for production”

**Agent must**
- Ask for threat model and compliance target.
- At minimum: validate inputs, avoid secrets in logs, document assumptions.

## H) “Magic Integration” Expectations
**Phrases**
- “plug in”, “drop-in replacement”
- “no code changes needed”
- “should work everywhere”

**Agent must**
- Explicitly list integration points and required changes.
- Provide migration steps and compatibility notes.

