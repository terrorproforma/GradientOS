# Gradient-05 Asset Template

This folder is the canonical asset bundle for the `gradient-05` robot.

## Required files

- `robot.json` - manifest consumed by runtime/web sync
- `gradient-05.urdf` - controller/web URDF (placeholder included)
- `dh_params.csv` - numeric IK DH parameters (placeholder included)

## Optional files

- `stl-files/` - mesh assets referenced by URDF
- `*.usd`, `*.usda`, `*.usdc` - optional external simulation assets
- additional docs/scripts used for model bring-up

## Fill-in checklist

1. Replace `gradient-05.urdf` with production geometry + joint limits.
2. Build DH from URDF with fitting and write CSV:
   - `& ".\\.venv\\Scripts\\python.exe" "robots/gradient-05/dh_tools.py" --write`
3. Validate DH vs URDF kinematics and build visual output:
   - `& ".\\.venv\\Scripts\\python.exe" "robots/gradient-05/dh_tools.py" --validate --samples 350 --plot "robots/gradient-05/dh_validation.png"`
4. Full one-shot flow (fit + write + validate + visual):
   - `& ".\\.venv\\Scripts\\python.exe" "robots/gradient-05/dh_tools.py" --write --validate --fit-samples 450 --samples 350 --plot "robots/gradient-05/dh_validation.png"`
5. Refine `dh_params.csv` as needed from validation results.
6. If URDF uses meshes, place them in `stl-files/` and keep relative paths.
7. If you add a different source subfolder for web assets, update
   `robot.json -> web.asset_source_dir`.
8. Run:
   - `npm run sync:robot-assets` (from `web-ui/`)
   - `& ".\\.venv\\Scripts\\python.exe" -m gradient_os.run_controller --list-robots`
9. (Optional) preserve `.usd` files here; they are pass-through only and not required.

## Joint limit update workflow

Use this whenever `gradient-05.urdf` joint `<limit .../>` values change.

1. Preview proposed config changes:
   - `& ".\\.venv\\Scripts\\python.exe" "scripts/sync_urdf_limits.py" --dry-run`
2. Apply sync into Python runtime config:
   - `& ".\\.venv\\Scripts\\python.exe" "scripts/sync_urdf_limits.py"`
3. Review the resulting diff in:
   - `src/gradient_os/arm_controller/robots/gradient05/config.py`
4. Run targeted regression checks:
   - `& ".\\.venv\\Scripts\\python.exe" -m pytest "tests/test_gradient05_limits_and_backends.py" -q`
