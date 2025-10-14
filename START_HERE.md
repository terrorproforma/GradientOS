# Getting Started with GradientOS

A quick checklist to bring up the development environment on Raspberry Pi or any Debian-based host. For detailed guides, see the documentation links at the end.

1. System packages (camera + OpenGL + libcap):
   ```bash
   ./setup.sh
   ```

2. Create a dedicated Python virtual environment with `uv`:
   ```bash
   uv venv .venv
   ```

3. Activate the environment (required for the next steps and CLI aliases):
   ```bash
   source .venv/bin/activate
   # or: source ./start.sh  # adds project-specific PYTHONPATH/aliases
   ```

4. Install GradientOS (builds the IK solver extension and Python deps):
   ```bash
   uv pip install -e .
   ```

Optional extras:
- Vision AI stack (YOLO + Torch): `uv pip install -e '.[ai]'`
- Dataset tooling (LeRobot export helpers): `uv pip install -e '.[datasets]'`
- Dev/test utilities (pytest, pre-commit): `uv pip install -e '.[dev]'`

Next steps:
- UI setup & troubleshooting: `docs/UI_readme.md`
- Vision module usage: `src/gradient_os/vision/README.md`
- Full project docs & command references: `docs/README.md`
