from __future__ import annotations

import hashlib
import importlib.util
import math
import os
import pathlib
import sys
import tempfile
import time
import types
from dataclasses import dataclass
from typing import Any


class TopologyDependencyError(RuntimeError):
    """Raised when no OpenCascade binding is available."""


class TopologyModelNotFoundError(KeyError):
    """Raised when requesting an unknown topology model."""


@dataclass(frozen=True)
class _EdgeRecord:
    edge_id: str
    part_id: str
    samples: list[tuple[float, float, float]]
    length_m: float


class CADTopologyService:
    """
    Extract exact CAD edge topology from STEP using OpenCascade bindings.

    The service stores parsed models in-memory and provides deterministic edge IDs
    per model fingerprint. It also provides arclength-based sampling helpers for
    weld start/stop parameters in normalized `s` coordinates.
    """

    def __init__(self) -> None:
        self._models_by_id: dict[str, dict[str, Any]] = {}
        self._model_id_by_fingerprint: dict[str, str] = {}

    def load_step(
        self,
        *,
        filename: str,
        step_bytes: bytes,
        sample_count: int = 64,
    ) -> dict[str, Any]:
        if not step_bytes:
            raise ValueError("STEP payload is empty.")
        if sample_count < 8:
            sample_count = 8

        fingerprint = hashlib.sha256(step_bytes).hexdigest()
        cached_model_id = self._model_id_by_fingerprint.get(fingerprint)
        if cached_model_id and cached_model_id in self._models_by_id:
            cached = self._models_by_id[cached_model_id]
            return {
                "model_id": cached["model_id"],
                "filename": cached.get("filename") or filename,
                "fingerprint": cached["fingerprint"],
                "parts": cached["parts"],
                "edges": cached["edges"],
                "cached": True,
            }

        extracted = self._extract_with_occ(step_bytes, sample_count=sample_count)
        model_id = f"step-{fingerprint[:16]}"
        model_payload = {
            "model_id": model_id,
            "filename": filename,
            "fingerprint": fingerprint,
            "created_at": time.time(),
            "parts": extracted["parts"],
            "edges": extracted["edges"],
        }
        self._models_by_id[model_id] = model_payload
        self._model_id_by_fingerprint[fingerprint] = model_id
        return {
            "model_id": model_id,
            "filename": filename,
            "fingerprint": fingerprint,
            "parts": extracted["parts"],
            "edges": extracted["edges"],
            "cached": False,
        }

    def get_model(self, model_id: str) -> dict[str, Any]:
        model = self._models_by_id.get(model_id)
        if model is None:
            raise TopologyModelNotFoundError(model_id)
        return {
            "model_id": model["model_id"],
            "filename": model["filename"],
            "fingerprint": model["fingerprint"],
            "parts": model["parts"],
            "edges": model["edges"],
        }

    def sample_edge_segment(
        self,
        *,
        model_id: str,
        edge_id: str,
        start_s: float,
        end_s: float,
        sample_count: int = 32,
    ) -> list[tuple[float, float, float]]:
        model = self._models_by_id.get(model_id)
        if model is None:
            raise TopologyModelNotFoundError(model_id)

        edge = None
        for item in model.get("edges", []):
            if item.get("id") == edge_id:
                edge = item
                break
        if edge is None:
            raise KeyError(f"edge_id '{edge_id}' not found in model '{model_id}'")

        pts = edge.get("samples") or []
        if len(pts) < 2:
            raise ValueError(f"Edge '{edge_id}' has insufficient samples.")

        a = max(0.0, min(1.0, float(start_s)))
        b = max(0.0, min(1.0, float(end_s)))
        if math.isclose(a, b):
            # Avoid degenerate zero-length trajectories by nudging end.
            b = min(1.0, a + 0.01) if a < 0.999 else max(0.0, a - 0.01)
        if b < a:
            a, b = b, a

        count = max(2, int(sample_count))
        return _sample_polyline_interval(pts, a, b, count)

    def _extract_with_occ(self, step_bytes: bytes, sample_count: int) -> dict[str, Any]:
        occ = _load_occ_api()

        with tempfile.NamedTemporaryFile(suffix=".step", delete=False) as handle:
            handle.write(step_bytes)
            step_path = handle.name

        try:
            reader = occ.STEPControl_Reader()
            status = reader.ReadFile(step_path)
            if status != occ.IFSelect_RetDone:
                raise ValueError("OpenCascade failed to read STEP file.")
            if reader.TransferRoots() <= 0:
                raise ValueError("OpenCascade found no transferable roots in STEP file.")
            root_shape = reader.Shape()
            if root_shape.IsNull():
                raise ValueError("OpenCascade produced an empty shape.")

            solids: list[Any] = []
            solid_explorer = occ.TopExp_Explorer(root_shape, occ.TopAbs_SOLID)
            while solid_explorer.More():
                solids.append(solid_explorer.Current())
                solid_explorer.Next()
            if not solids:
                solids = [root_shape]

            seen_hashes: set[int] = set()
            seen_shapes: list[Any] = []
            edges: list[dict[str, Any]] = []
            parts: list[dict[str, Any]] = []
            edge_index = 0

            for part_index, shape in enumerate(solids):
                part_id = f"part_{part_index}"
                local_count = 0
                edge_explorer = occ.TopExp_Explorer(shape, occ.TopAbs_EDGE)
                while edge_explorer.More():
                    edge_shape = edge_explorer.Current()
                    edge_explorer.Next()

                    # Deduplicate edges that appear in multiple solids.
                    # Some OCC bindings expose HashCode; others do not.
                    shape_hash = _shape_hash(edge_shape)
                    if shape_hash is not None:
                        if shape_hash in seen_hashes:
                            continue
                        seen_hashes.add(shape_hash)
                    else:
                        # Fallback dedupe based on topological identity when possible.
                        duplicate = False
                        for prev_shape in seen_shapes:
                            try:
                                if edge_shape.IsSame(prev_shape):
                                    duplicate = True
                                    break
                            except Exception:
                                continue
                        if duplicate:
                            continue
                        seen_shapes.append(edge_shape)

                    edge = occ.topods_Edge(edge_shape)
                    samples = _sample_occ_edge(occ, edge, sample_count=sample_count)
                    if len(samples) < 2:
                        continue

                    edge_id = f"{part_id}:edge_{edge_index:05d}"
                    edge_index += 1
                    local_count += 1
                    length = _polyline_length(samples)
                    p_min, p_max = _bounds(samples)
                    edges.append(
                        {
                            "id": edge_id,
                            "part_id": part_id,
                            "sample_count": len(samples),
                            "samples": samples,
                            "length_m": length,
                            "bounds_min": p_min,
                            "bounds_max": p_max,
                        }
                    )

                parts.append({"id": part_id, "edge_count": local_count})

            if not edges:
                raise ValueError("No CAD edges found in STEP topology.")
            unit_scale_to_meters = _infer_unit_scale_to_meters(edges)
            if not math.isclose(unit_scale_to_meters, 1.0, rel_tol=0.0, abs_tol=1e-12):
                for edge in edges:
                    samples = [
                        _scale_point(
                            (float(point[0]), float(point[1]), float(point[2])),
                            unit_scale_to_meters,
                        )
                        for point in (edge.get("samples") or [])
                    ]
                    edge["samples"] = samples
                    edge["sample_count"] = len(samples)
                    edge["length_m"] = _polyline_length(samples)
                    bounds_min, bounds_max = _bounds(samples)
                    edge["bounds_min"] = bounds_min
                    edge["bounds_max"] = bounds_max
            return {"parts": parts, "edges": edges}
        finally:
            try:
                os.remove(step_path)
            except OSError:
                pass


@dataclass(frozen=True)
class _OccApi:
    STEPControl_Reader: Any
    IFSelect_RetDone: Any
    TopExp_Explorer: Any
    TopAbs_SOLID: Any
    TopAbs_EDGE: Any
    topods_Edge: Any
    BRepAdaptor_Curve: Any


def _load_occ_api() -> _OccApi:
    """
    Import OpenCascade bindings from either pythonocc (`OCC`) or OCP.
    """
    try:
        from OCC.Core.BRepAdaptor import BRepAdaptor_Curve  # type: ignore
        from OCC.Core.IFSelect import IFSelect_RetDone  # type: ignore
        from OCC.Core.STEPControl import STEPControl_Reader  # type: ignore
        from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_SOLID  # type: ignore
        from OCC.Core.TopExp import TopExp_Explorer  # type: ignore
        from OCC.Core.TopoDS import topods_Edge  # type: ignore

        return _OccApi(
            STEPControl_Reader=STEPControl_Reader,
            IFSelect_RetDone=IFSelect_RetDone,
            TopExp_Explorer=TopExp_Explorer,
            TopAbs_SOLID=TopAbs_SOLID,
            TopAbs_EDGE=TopAbs_EDGE,
            topods_Edge=topods_Edge,
            BRepAdaptor_Curve=BRepAdaptor_Curve,
        )
    except Exception:
        pass

    try:
        from OCP.BRepAdaptor import BRepAdaptor_Curve  # type: ignore
        from OCP.IFSelect import IFSelect_RetDone  # type: ignore
        from OCP.STEPControl import STEPControl_Reader  # type: ignore
        from OCP.TopAbs import TopAbs_EDGE, TopAbs_SOLID  # type: ignore
        from OCP.TopExp import TopExp_Explorer  # type: ignore
        from OCP.TopoDS import TopoDS  # type: ignore

        return _OccApi(
            STEPControl_Reader=STEPControl_Reader,
            IFSelect_RetDone=IFSelect_RetDone,
            TopExp_Explorer=TopExp_Explorer,
            TopAbs_SOLID=TopAbs_SOLID,
            TopAbs_EDGE=TopAbs_EDGE,
            topods_Edge=TopoDS.Edge_s,
            BRepAdaptor_Curve=BRepAdaptor_Curve,
        )
    except Exception:
        pass

    # Compatibility path: some Windows wheels install a lowercase `ocp`
    # package containing `OCP*.pyd` but no resolvable top-level `OCP`.
    try:
        _bootstrap_ocp_namespace_from_lowercase_package()
        from OCP.BRepAdaptor import BRepAdaptor_Curve  # type: ignore
        from OCP.IFSelect import IFSelect_RetDone  # type: ignore
        from OCP.STEPControl import STEPControl_Reader  # type: ignore
        from OCP.TopAbs import TopAbs_EDGE, TopAbs_SOLID  # type: ignore
        from OCP.TopExp import TopExp_Explorer  # type: ignore
        from OCP.TopoDS import TopoDS  # type: ignore

        return _OccApi(
            STEPControl_Reader=STEPControl_Reader,
            IFSelect_RetDone=IFSelect_RetDone,
            TopExp_Explorer=TopExp_Explorer,
            TopAbs_SOLID=TopAbs_SOLID,
            TopAbs_EDGE=TopAbs_EDGE,
            topods_Edge=TopoDS.Edge_s,
            BRepAdaptor_Curve=BRepAdaptor_Curve,
        )
    except Exception as exc:
        raise TopologyDependencyError(
            _missing_occ_dependency_message()
        ) from exc


def _missing_occ_dependency_message() -> str:
    return (
        "No usable OpenCascade Python binding found. "
        "Install the CAD extra with `uv pip install -e '.[cad]'`. "
        "If you are installing packages manually, use a compatible binding "
        "such as `cadquery-ocp` or `pythonocc-core`."
    )


def _bootstrap_ocp_namespace_from_lowercase_package() -> None:
    if importlib.util.find_spec("OCP") is not None:
        return

    spec = importlib.util.find_spec("ocp")
    if spec is None or not spec.submodule_search_locations:
        raise ModuleNotFoundError("ocp package not found")

    ocp_dir = pathlib.Path(next(iter(spec.submodule_search_locations)))
    if not ocp_dir.exists():
        raise FileNotFoundError(f"ocp package directory does not exist: {ocp_dir}")

    site_packages = ocp_dir.parent
    cad_libs = site_packages / "cadquery_ocp.libs"
    vtk_libs = site_packages / "vtk.libs"
    if hasattr(os, "add_dll_directory"):
        if cad_libs.is_dir():
            os.add_dll_directory(str(cad_libs))
        if vtk_libs.is_dir():
            os.add_dll_directory(str(vtk_libs))

    extension = next(ocp_dir.glob("OCP*.pyd"), None)
    if extension is None:
        raise FileNotFoundError(f"No OCP extension module found in {ocp_dir}")

    pkg = types.ModuleType("OCP")
    pkg.__package__ = "OCP"
    pkg.__path__ = [str(ocp_dir)]  # type: ignore[attr-defined]
    sys.modules["OCP"] = pkg

    module_spec = importlib.util.spec_from_file_location("OCP.OCP", str(extension))
    if module_spec is None or module_spec.loader is None:
        raise ImportError(f"Could not create import spec for {extension}")
    module = importlib.util.module_from_spec(module_spec)
    sys.modules["OCP.OCP"] = module
    module_spec.loader.exec_module(module)


def _sample_occ_edge(occ: _OccApi, edge: Any, sample_count: int) -> list[tuple[float, float, float]]:
    curve = occ.BRepAdaptor_Curve(edge)
    first = float(curve.FirstParameter())
    last = float(curve.LastParameter())
    if not math.isfinite(first) or not math.isfinite(last):
        return []
    if math.isclose(first, last):
        return []

    points: list[tuple[float, float, float]] = []
    count = max(2, int(sample_count))
    for idx in range(count):
        t = idx / (count - 1)
        param = first + (last - first) * t
        p = curve.Value(param)
        points.append((float(p.X()), float(p.Y()), float(p.Z())))
    return _dedupe_neighbor_points(points)


def _shape_hash(shape: Any) -> int | None:
    try:
        if hasattr(shape, "HashCode"):
            return int(shape.HashCode(2147483647))
    except Exception:
        pass
    return None


def _dedupe_neighbor_points(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    if not points:
        return []
    out = [points[0]]
    for p in points[1:]:
        if _dist(p, out[-1]) > 1e-9:
            out.append(p)
    return out


def _dist(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _polyline_length(points: list[tuple[float, float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    return sum(_dist(points[i - 1], points[i]) for i in range(1, len(points)))


def _bounds(points: list[tuple[float, float, float]]) -> tuple[list[float], list[float]]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return [min(xs), min(ys), min(zs)], [max(xs), max(ys), max(zs)]


def _scale_point(
    point: tuple[float, float, float],
    factor: float,
) -> tuple[float, float, float]:
    return point[0] * factor, point[1] * factor, point[2] * factor


def _infer_unit_scale_to_meters(edges: list[dict[str, Any]]) -> float:
    """
    Best-effort unit normalization.
    STEP files are often authored in millimeters while the UI/planner uses meters.
    """
    all_points: list[tuple[float, float, float]] = []
    for edge in edges:
        all_points.extend(edge.get("samples") or [])
    if len(all_points) < 2:
        return 1.0
    bounds_min, bounds_max = _bounds(all_points)
    span = max(
        bounds_max[0] - bounds_min[0],
        bounds_max[1] - bounds_min[1],
        bounds_max[2] - bounds_min[2],
    )
    # Most robot workcells are well below 5 m extents; values above this are
    # usually millimeter-based STEP geometry that should be scaled to meters.
    if span > 5.0:
        return 0.001
    return 1.0


def _sample_polyline_interval(
    points: list[tuple[float, float, float]],
    start_s: float,
    end_s: float,
    sample_count: int,
) -> list[tuple[float, float, float]]:
    if len(points) < 2:
        return list(points)
    distances = [0.0]
    for i in range(1, len(points)):
        distances.append(distances[-1] + _dist(points[i - 1], points[i]))
    total = distances[-1]
    if total <= 1e-9:
        return [points[0], points[-1]]

    start_l = max(0.0, min(1.0, start_s)) * total
    end_l = max(0.0, min(1.0, end_s)) * total
    if end_l < start_l:
        start_l, end_l = end_l, start_l

    out: list[tuple[float, float, float]] = []
    count = max(2, int(sample_count))
    for i in range(count):
        alpha = i / (count - 1)
        target = start_l + (end_l - start_l) * alpha
        out.append(_interpolate_along_polyline(points, distances, target))
    return _dedupe_neighbor_points(out)


def _interpolate_along_polyline(
    points: list[tuple[float, float, float]],
    cumulative_lengths: list[float],
    target_length: float,
) -> tuple[float, float, float]:
    if target_length <= 0.0:
        return points[0]
    if target_length >= cumulative_lengths[-1]:
        return points[-1]

    hi = 1
    while hi < len(cumulative_lengths) and cumulative_lengths[hi] < target_length:
        hi += 1
    lo = max(0, hi - 1)
    l0 = cumulative_lengths[lo]
    l1 = cumulative_lengths[hi]
    if math.isclose(l1, l0):
        return points[hi]

    t = (target_length - l0) / (l1 - l0)
    p0 = points[lo]
    p1 = points[hi]
    return (
        p0[0] + (p1[0] - p0[0]) * t,
        p0[1] + (p1[1] - p0[1]) * t,
        p0[2] + (p1[2] - p0[2]) * t,
    )
