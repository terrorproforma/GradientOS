"""
Command-line interface for GradientOS vision module.

Provides quick commands to list cameras, test initialization,
run basic image processing checks, and stream frames to the console.
"""

import argparse
import logging
import time
from typing import Optional
import threading

from .camera_driver import (
    PICAMERA2_AVAILABLE,
    OPENCV_AVAILABLE,
    BACKEND_AUTO,
    BACKEND_PICAMERA,
    BACKEND_USB,
    available_backends,
    create_camera_driver,
    list_cameras_for_backend,
    resolve_backend,
)
from .image_processor import ImageProcessor
from .yolo_detector import YOLODetector


logger = logging.getLogger(__name__)


def _instantiate_driver(camera_id: int,
                        resolution: tuple[int, int],
                        framerate: int,
                        backend_choice: Optional[str]):
    """
    Resolve the desired backend and create the corresponding camera driver.
    """
    try:
        backend_name = resolve_backend(backend_choice)
    except RuntimeError as exc:
        print(f"❌ {exc}")
        return None, None

    try:
        driver = create_camera_driver(
            backend_name,
            camera_id=camera_id,
            resolution=resolution,
            framerate=framerate,
        )
        return driver, backend_name
    except (RuntimeError, ImportError) as exc:
        print(f"❌ {exc}")
        return None, None


def list_cameras(backend_choice: Optional[str]) -> int:
    """List available cameras for the requested backend and return exit code."""
    try:
        if backend_choice in (None, BACKEND_AUTO):
            backends = available_backends()
            if not backends:
                print("❌ No camera backend available. Install picamera2 (for Pi) or opencv-python (for USB webcams).")
                return 1
            exit_code = 1
            for backend in backends:
                cameras = list_cameras_for_backend(backend)
                label = "PiCamera2" if backend == BACKEND_PICAMERA else "USB (OpenCV)"
                print(f"{label} devices:")
                if not cameras:
                    print("  (none detected)")
                    continue
                exit_code = 0
                for info in cameras:
                    ident = info.get("id", "?")
                    descriptor = info.get("name") or info.get("Model") or info
                    res = info.get("resolution")
                    fps = info.get("fps")
                    extras = []
                    if res:
                        extras.append(f"{res[0]}x{res[1]}")
                    if fps:
                        extras.append(f"{fps:.1f}fps" if isinstance(fps, float) else f"{fps}fps")
                    extra_str = f" ({', '.join(extras)})" if extras else ""
                    print(f" - [{ident}] {descriptor}{extra_str}")
            return exit_code

        backend_name = resolve_backend(backend_choice)
    except RuntimeError as exc:
        print(f"❌ {exc}")
        return 1

    cameras = list_cameras_for_backend(backend_name)
    human_label = "PiCamera2" if backend_name == BACKEND_PICAMERA else "USB (OpenCV)"
    print(f"{human_label} devices:")
    if not cameras:
        print("  (none detected)")
        return 1

    for info in cameras:
        ident = info.get("id", "?")
        descriptor = info.get("name") or info.get("Model") or info
        res = info.get("resolution")
        fps = info.get("fps")
        extras = []
        if res:
            extras.append(f"{res[0]}x{res[1]}")
        if fps:
            extras.append(f"{fps:.1f}fps" if isinstance(fps, float) else f"{fps}fps")
        extra_str = f" ({', '.join(extras)})" if extras else ""
        print(f" - [{ident}] {descriptor}{extra_str}")
    return 0


def test_init(camera_id: int,
              resolution: tuple[int, int],
              framerate: int,
              backend_choice: Optional[str]) -> int:
    """Test camera initialization and capture a single image."""
    cam, backend_name = _instantiate_driver(camera_id, resolution, framerate, backend_choice)
    if cam is None:
        return 1

    if backend_name == BACKEND_USB and not OPENCV_AVAILABLE:
        print("❌ OpenCV not available. Install with: pip install opencv-python")
        return 1

    if not cam.initialize():
        print("❌ Failed to initialize camera")
        try:
            cam.close()
        except Exception:
            pass
        return 1

    img = cam.capture_image()
    cam.close()

    if img is None:
        print("❌ Failed to capture image")
        return 1

    print(f"✅ Camera initialized and captured image: {img.shape}")
    return 0


def test_processing(camera_id: int,
                    resolution: tuple[int, int],
                    framerate: int,
                    backend_choice: Optional[str]) -> int:
    """Run a basic image processing pipeline on a captured frame."""
    if not OPENCV_AVAILABLE:
        print("❌ OpenCV not available. Install with: pip install opencv-python")
        return 1

    cam, _backend_name = _instantiate_driver(camera_id, resolution, framerate, backend_choice)
    if cam is None:
        return 1

    if not cam.initialize():
        print("❌ Failed to initialize camera")
        try:
            cam.close()
        except Exception:
            pass
        return 1
    img = cam.capture_image()
    cam.close()

    if img is None:
        print("❌ Failed to capture image")
        return 1

    processor = ImageProcessor()
    processed = processor.preprocess_image(img, resize=(320, 240), blur=5)
    print(f"✅ Preprocessed image: {processed.shape}")

    # Simple color detection example (red-ish tones)
    lower_red = (0, 50, 50)
    upper_red = (10, 255, 255)
    objects = processor.detect_objects_by_color(img, lower_red, upper_red)
    print(f"✅ Detected {len(objects)} red objects")

    edges = processor.detect_edges(img)
    print(f"✅ Edge map computed: {edges.shape}")
    return 0


def stream(camera_id: int,
           resolution: tuple[int, int],
           framerate: int,
           duration_s: Optional[int],
           backend_choice: Optional[str]) -> int:
    """Stream frames and print FPS to the console. Stops after duration if provided."""
    cam, _backend_name = _instantiate_driver(camera_id, resolution, framerate, backend_choice)
    if cam is None:
        return 1

    frame_count = 0
    start_time = time.time()

    def on_frame(_frame):
        nonlocal frame_count
        frame_count += 1
        # Print FPS every ~2 seconds
        elapsed = time.time() - start_time
        if elapsed >= 2.0 and frame_count % max(1, int(2 * framerate)) == 0:
            fps = frame_count / elapsed
            print(f"FPS: {fps:.1f}")

    if not cam.initialize():
        print("❌ Failed to initialize camera")
        try:
            cam.close()
        except Exception:
            pass
        return 1
    if not cam.start_streaming(on_frame):
        print("❌ Failed to start streaming")
        cam.close()
        return 1

    try:
        if duration_s is None or duration_s <= 0:
            print("🎥 Streaming... Press Ctrl+C to stop")
            while True:
                time.sleep(0.25)
        else:
            print(f"🎥 Streaming for {duration_s} seconds...")
            time.sleep(duration_s)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop_streaming()
        cam.close()

    total_elapsed = time.time() - start_time
    if total_elapsed > 0:
        avg_fps = frame_count / total_elapsed
        print(f"Average FPS: {avg_fps:.1f}")
    return 0


def mjpeg_server(host: str,
                 port: int,
                 camera_id: Optional[int],
                 resolution: tuple[int, int],
                 framerate: int,
                 jpeg_quality: int,
                 vflip: bool,
                 hflip: bool,
                 force_both: bool,
                 backend_choice: Optional[str],
                 proc_cfg: Optional[dict] = None,
                 ai_cfg: Optional[dict] = None,
                 overlay_status: bool = True) -> int:
    """Serve MJPEG over HTTP. Auto-uses two cameras if available unless forced single."""
    if not OPENCV_AVAILABLE:
        print("❌ OpenCV not available. Install with: pip install opencv-python")
        return 1

    try:
        backend_name = resolve_backend(backend_choice)
    except RuntimeError as exc:
        print(f"❌ {exc}")
        return 1

    try:
        from http.server import BaseHTTPRequestHandler, HTTPServer
        from socketserver import ThreadingMixIn
        import cv2  # encode_jpeg
    except Exception as e:
        print(f"❌ Missing deps for HTTP server: {e}")
        return 1

    available: list[dict] = []
    candidate_ids: list[int] = []

    # Only enumerate cameras (which probes device indices) when we actually need to.
    if force_both or camera_id is None:
        available = list_cameras_for_backend(backend_name) or []
        for info in available:
            ident = info.get("id")
            if isinstance(ident, int):
                candidate_ids.append(ident)
            else:
                try:
                    candidate_ids.append(int(ident))
                except Exception:
                    candidate_ids.append(len(candidate_ids))
        if not candidate_ids:
            candidate_ids = list(range(len(available)))
        # If user supplied camera ID but also requested dual mode, make sure it is first.
        if force_both and camera_id is not None and camera_id not in candidate_ids:
            candidate_ids.insert(0, camera_id)
    else:
        candidate_ids = [camera_id]

    use_both = False
    if force_both:
        use_both = True
    elif camera_id is None and len(candidate_ids) >= 2:
        use_both = True
    processor: Optional[ImageProcessor] = None
    if proc_cfg and proc_cfg.get("enable", False):
        try:
            processor = ImageProcessor()
        except Exception as e:
            print(f"❌ Failed to initialize ImageProcessor: {e}")
            processor = None

    yolo: Optional[YOLODetector] = None
    if ai_cfg and ai_cfg.get("enable", False):
        try:
            yolo = YOLODetector(
                weights=ai_cfg.get("weights", "yolo11n.pt"),
                confidence_threshold=float(ai_cfg.get("conf", 0.25)),
                image_size=int(ai_cfg.get("imgsz", 640)),
                device=ai_cfg.get("device", "cpu"),
                classes=ai_cfg.get("classes", None),
                max_detections=int(ai_cfg.get("max_det", 300)),
            )
            if not yolo.available:
                msg = yolo.last_error or "unknown reason"
                print(f"⚠️ YOLO unavailable: {msg}. Install 'ultralytics' and 'torch', or fix weights/device.")
                yolo = None
        except Exception as e:
            print(f"❌ Failed to initialize YOLO: {e}")
            yolo = None

    model_label = "AI"
    try:
        if ai_cfg:
            model_label = str(ai_cfg.get("label", ai_cfg.get("weights", "AI")))
    except Exception:
        pass


    latest_lock = threading.Lock()
    latest_single: Optional[bytes] = None
    latest_map: dict[int, Optional[bytes]] = {0: None, 1: None}

    def encode_jpeg(frame, quality: int = 80) -> Optional[bytes]:
        try:
            q = max(30, min(95, int(quality)))
            ret, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), q])
            if not ret:
                return None
            return buf.tobytes()
        except Exception:
            return None

    class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
        daemon_threads = True

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            nonlocal latest_single, latest_map
            if not use_both and self.path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Age', 0)
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
                self.end_headers()
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                try:
                    while True:
                        with latest_lock:
                            jpeg = latest_single
                        if jpeg is None:
                            time.sleep(0.01)
                            continue
                        self.wfile.write(b'--FRAME\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n')
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                except BrokenPipeError:
                    pass
                except Exception:
                    pass
            elif use_both and self.path in ('/cam0.mjpg', '/cam1.mjpg'):
                cam_idx = 0 if 'cam0' in self.path else 1
                self.send_response(200)
                self.send_header('Age', 0)
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
                self.end_headers()
                try:
                    self.wfile.flush()
                except Exception:
                    pass
                try:
                    while True:
                        with latest_lock:
                            jpeg = latest_map.get(cam_idx)
                        if jpeg is None:
                            time.sleep(0.01)
                            continue
                        self.wfile.write(b'--FRAME\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n')
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                except BrokenPipeError:
                    pass
                except Exception:
                    pass
            else:
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.end_headers()
                if use_both:
                    page = (
                        "<html><body style='background-color: black; color: white;'>"
                        "<h1>GradientOS Dual Camera Stream</h1>"
                        "<div style='display:flex;flex-direction:column;gap:10px;'>"
                        "<div><h3>Camera 0</h3><img src='/cam0.mjpg'/></div>"
                        "<div><h3>Camera 1</h3><img src='/cam1.mjpg'/></div>"
                        "</div>"
                        "</body></html>"
                    )
                    self.wfile.write(page.encode('utf-8'))
                else:
                    self.wfile.write(b"<html><body style='background-color: black; color: white;'>" \
                                     b"<h1>GradientOS Camera Stream</h1>" \
                                     b"<img src='/stream.mjpg'/>" \
                                     b"</body></html>")

    # Start cameras
    cams: list[Optional[object]] = []
    if use_both:
        ids_to_use = candidate_ids[:2]
        if len(ids_to_use) < 2:
            ids_to_use.extend(range(len(ids_to_use), 2))
        for display_idx, cam_index in enumerate(ids_to_use[:2]):
            try:
                cam = create_camera_driver(
                    backend_name,
                    camera_id=cam_index,
                    resolution=resolution,
                    framerate=framerate,
                )
            except (RuntimeError, ImportError) as exc:
                print(f"[cam{display_idx}] {exc}")
                cams.append(None)
                continue

            if not cam.initialize():
                print(f"[cam{display_idx}] Failed to initialize camera")
                try:
                    cam.close()
                except Exception:
                    pass
                cams.append(None)
                continue

            def make_cb(slot: int):
                def on_frame(frame):
                    try:
                        import cv2
                        if vflip and hflip:
                            frame_flipped = cv2.flip(frame, -1)
                        elif vflip:
                            frame_flipped = cv2.flip(frame, 0)
                        elif hflip:
                            frame_flipped = cv2.flip(frame, 1)
                        else:
                            frame_flipped = frame
                    except Exception:
                        frame_flipped = frame
                    frame_out = frame_flipped
                    # Optional image processing overlay
                    if processor is not None and proc_cfg and proc_cfg.get("object_detection", False):
                        try:
                            lower = proc_cfg.get("lower_hsv", (0, 50, 50))
                            upper = proc_cfg.get("upper_hsv", (10, 255, 255))
                            min_area = int(proc_cfg.get("min_area", 100))
                            objs = processor.detect_objects_by_color(frame_out, lower, upper, min_area=min_area)
                            frame_out = processor.draw_bounding_boxes(frame_out, objs)
                            try:
                                color_name = proc_cfg.get("color_name", "red")
                                import cv2
                                cv2.putText(frame_out, f"Detect: {color_name}  HSV {lower}-{upper}  min_area={min_area}  count={len(objs)}",
                                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 255), 2)
                            except Exception:
                                pass
                        except Exception:
                            pass
                    if yolo is not None:
                        res = yolo.detect(frame_out)
                        dets = res.get("boxes", [])
                        if dets:
                            frame_out = YOLODetector.draw_detections(frame_out, dets)
                        masks_val = res.get("masks")
                        if masks_val:
                            frame_out = YOLODetector.draw_masks(frame_out, masks_val)
                        else:
                            if isinstance(getattr(yolo, 'weights', ''), str) and ai_cfg and 'seg' in ai_cfg.get('weights', ''):
                                try:
                                    import cv2
                                    cv2.putText(frame_out, "(no masks in result; try --conf 0.15 or --imgsz 640)", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 255, 255), 2)
                                except Exception:
                                    pass
                        if res.get("keypoints"):
                            frame_out = YOLODetector.draw_keypoints(frame_out, res["keypoints"])
                        if overlay_status:
                            try:
                                import cv2
                                cv2.putText(frame_out, f"{model_label}: {len(dets)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                            except Exception:
                                pass
                    else:
                        if overlay_status:
                            try:
                                import cv2
                                status = f"{model_label}: OFF"
                                if ai_cfg:
                                    err = ai_cfg.get("_init_error", "")
                                    if err:
                                        status = f"{model_label}: OFF ({err[:30]})"
                                cv2.putText(frame_out, status, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                            except Exception:
                                pass

                    jpeg = encode_jpeg(frame_out, jpeg_quality)
                    if jpeg is None:
                        return
                    with latest_lock:
                        latest_map[slot] = jpeg
                return on_frame

            if not cam.start_streaming(make_cb(display_idx)):
                print(f"[cam{display_idx}] Failed to start streaming")
                try:
                    cam.close()
                except Exception:
                    pass
                cams.append(None)
                continue
            cams.append(cam)
        if cams and all(c is None for c in cams):
            print('No cameras streaming. Exiting.')
            return 1
    else:
        # Single camera
        if camera_id is not None:
            use_id = camera_id
        elif candidate_ids:
            use_id = candidate_ids[0]
        else:
            use_id = 0

        try:
            cam = create_camera_driver(
                backend_name,
                camera_id=use_id,
                resolution=resolution,
                framerate=framerate,
            )
        except (RuntimeError, ImportError) as exc:
            print(f"❌ {exc}")
            return 1

        if not cam.initialize():
            print('Failed to initialize camera')
            try:
                cam.close()
            except Exception:
                pass
            return 1

        def on_frame_single(frame):
            nonlocal latest_single
            try:
                import cv2
                if vflip and hflip:
                    frame_flipped = cv2.flip(frame, -1)
                elif vflip:
                    frame_flipped = cv2.flip(frame, 0)
                elif hflip:
                    frame_flipped = cv2.flip(frame, 1)
                else:
                    frame_flipped = frame
            except Exception:
                frame_flipped = frame
            frame_out = frame_flipped
            if processor is not None and proc_cfg and proc_cfg.get("object_detection", False):
                try:
                    lower = proc_cfg.get("lower_hsv", (0, 50, 50))
                    upper = proc_cfg.get("upper_hsv", (10, 255, 255))
                    min_area = int(proc_cfg.get("min_area", 100))
                    objs = processor.detect_objects_by_color(frame_out, lower, upper, min_area=min_area)
                    frame_out = processor.draw_bounding_boxes(frame_out, objs)
                    try:
                        import cv2
                        color_name = proc_cfg.get("color_name", "red")
                        cv2.putText(frame_out, f"Detect: {color_name}  HSV {lower}-{upper}  min_area={min_area}  count={len(objs)}",
                                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 255), 2)
                    except Exception:
                        pass
                except Exception:
                    pass
            if yolo is not None:
                res = yolo.detect(frame_out)
                dets = res.get("boxes", [])
                if dets:
                    frame_out = YOLODetector.draw_detections(frame_out, dets)
                masks_val = res.get("masks")
                if masks_val:
                    frame_out = YOLODetector.draw_masks(frame_out, masks_val)
                else:
                    if isinstance(getattr(yolo, 'weights', ''), str) and ai_cfg and 'seg' in ai_cfg.get('weights', ''):
                        try:
                            import cv2
                            cv2.putText(frame_out, "(no masks in result; try --conf 0.15 or --imgsz 640)", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 255, 255), 2)
                        except Exception:
                            pass
                if res.get("keypoints"):
                    frame_out = YOLODetector.draw_keypoints(frame_out, res["keypoints"])
                if overlay_status:
                    try:
                        import cv2
                        cv2.putText(frame_out, f"{model_label}: {len(dets)}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                    except Exception:
                        pass
            else:
                if overlay_status:
                    try:
                        import cv2
                        status = f"{model_label}: OFF"
                        if ai_cfg:
                            err = ai_cfg.get("_init_error", "")
                            if err:
                                status = f"{model_label}: OFF ({err[:30]})"
                        cv2.putText(frame_out, status, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    except Exception:
                        pass

            jpeg = encode_jpeg(frame_out, jpeg_quality)
            if jpeg is None:
                return
            with latest_lock:
                latest_single = jpeg

        cam.start_streaming(on_frame_single)
        cams = [cam]

    try:
        server = ThreadedHTTPServer((host, port), Handler)
    except OSError as exc:
        print(f"❌ Failed to bind MJPEG server on {host}:{port} ({exc})")
        return 1
    if use_both:
        print(f"Serving MJPEG (cam0=/cam0.mjpg, cam1=/cam1.mjpg) on http://{host}:{port}/")
    else:
        print(f"Serving MJPEG on http://{host}:{port}/")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for c in cams:
            try:
                if c is not None:
                    c.stop_streaming()
                    c.close()
            except Exception:
                pass
    return 0

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="GradientOS Vision CLI")
    parser.add_argument(
        "--backend",
        choices=[BACKEND_AUTO, BACKEND_PICAMERA, BACKEND_USB],
        default=BACKEND_AUTO,
        help="Camera backend to use (default: auto-detect).",
    )
    # Subcommand is optional; default to 'stream' so users can just run the tool with sensible defaults
    subparsers = parser.add_subparsers(dest="command")

    # Provide sensible Pi defaults globally so they exist even when no subcommand is provided
    parser.set_defaults(
        command="stream",
        camera=0,
        width=640,
        height=480,
        fps=30,
        duration=0,
    )

    # list
    subparsers.add_parser("list", help="List available cameras")

    # init
    p_init = subparsers.add_parser("init", help="Test camera initialization and capture one frame")
    p_init.add_argument("--camera", type=int, default=0)
    p_init.add_argument("--width", type=int, default=640)
    p_init.add_argument("--height", type=int, default=480)
    p_init.add_argument("--fps", type=int, default=30)

    # processing
    p_proc = subparsers.add_parser("processing", help="Run basic image processing checks")
    p_proc.add_argument("--camera", type=int, default=0)
    p_proc.add_argument("--width", type=int, default=640)
    p_proc.add_argument("--height", type=int, default=480)
    p_proc.add_argument("--fps", type=int, default=30)

    # stream
    p_stream = subparsers.add_parser("stream", help="Stream frames and print FPS to console")
    p_stream.add_argument("--camera", type=int, default=0)
    p_stream.add_argument("--width", type=int, default=640)
    p_stream.add_argument("--height", type=int, default=480)
    p_stream.add_argument("--fps", type=int, default=30)
    p_stream.add_argument("--duration", type=int, default=0, help="Duration in seconds (0 = until Ctrl+C)")

    # mjpeg http server (single or dual cameras)
    p_http = subparsers.add_parser("mjpeg", help="Serve MJPEG over HTTP; auto-dual if two cameras detected")
    p_http.add_argument("--host", type=str, default="0.0.0.0")
    p_http.add_argument("--port", type=int, default=8080)
    p_http.add_argument("--camera", type=int, default=None, help="Camera index for single-camera mode (omit to auto-use dual if available)")
    p_http.add_argument("--width", type=int, default=640)
    p_http.add_argument("--height", type=int, default=480)
    p_http.add_argument("--fps", type=int, default=30)
    p_http.add_argument("--jpeg-quality", type=int, default=80)
    p_http.add_argument("--vflip", action="store_true")
    p_http.add_argument("--hflip", action="store_true")
    p_http.add_argument("--both", action="store_true", help="Force dual-camera mode (cam 0 and 1)")
    p_http.add_argument("--no-overlay", action="store_true", help="Disable on-frame status overlays")

    # Sub-mode: image processing overlay
    http_sub = p_http.add_subparsers(dest="http_mode")
    p_http_ip = http_sub.add_parser("img-proc", help="Enable image processing overlay on the MJPEG stream")
    p_http_ip.add_argument("--object-detection", action="store_true", help="Enable basic color-based object detection overlay")
    p_http_ip.add_argument("--color", choices=["red", "green", "blue", "yellow"], default="red", help="Predefined color range to detect")
    p_http_ip.add_argument("--min-area", type=int, default=100, help="Minimum area for detected objects")
    # Duplicate flip flags here so they work after the subcommand, matching example usage
    p_http_ip.add_argument("--vflip", action="store_true")
    p_http_ip.add_argument("--hflip", action="store_true")

    # Sub-mode: AI detection (YOLO - detect)
    p_http_ai = http_sub.add_parser("ai", help="Enable YOLO-based AI detection overlay on the MJPEG stream")
    p_http_ai.add_argument("--weights", type=str, default="yolo11n.pt", help="Ultralytics weights/preset (YOLO, RT-DETR, etc.)")
    p_http_ai.add_argument("--conf", type=float, default=0.25)
    p_http_ai.add_argument("--imgsz", type=int, default=640)
    p_http_ai.add_argument("--device", type=str, default="cpu")
    p_http_ai.add_argument("--classes", type=str, default="", help="Comma-separated class ids to filter (empty = all)")
    p_http_ai.add_argument("--max-det", type=int, default=300)
    p_http_ai.add_argument("--vflip", action="store_true")
    p_http_ai.add_argument("--hflip", action="store_true")

    # Sub-mode: AI segmentation
    p_http_ai_seg = http_sub.add_parser("ai-seg", help="Enable YOLO segmentation overlay (requires yolo11*-seg.pt)")
    p_http_ai_seg.add_argument("--weights", type=str, default="yolo11n-seg.pt")
    p_http_ai_seg.add_argument("--conf", type=float, default=0.25)
    p_http_ai_seg.add_argument("--imgsz", type=int, default=640)
    p_http_ai_seg.add_argument("--device", type=str, default="cpu")
    p_http_ai_seg.add_argument("--classes", type=str, default="")
    p_http_ai_seg.add_argument("--max-det", type=int, default=300)
    p_http_ai_seg.add_argument("--vflip", action="store_true")
    p_http_ai_seg.add_argument("--hflip", action="store_true")

    # Sub-mode: AI pose
    p_http_ai_pose = http_sub.add_parser("ai-pose", help="Enable YOLO pose overlay (requires yolo11*-pose.pt)")
    p_http_ai_pose.add_argument("--weights", type=str, default="yolo11n-pose.pt")
    p_http_ai_pose.add_argument("--conf", type=float, default=0.25)
    p_http_ai_pose.add_argument("--imgsz", type=int, default=640)
    p_http_ai_pose.add_argument("--device", type=str, default="cpu")
    p_http_ai_pose.add_argument("--classes", type=str, default="")
    p_http_ai_pose.add_argument("--max-det", type=int, default=300)
    p_http_ai_pose.add_argument("--vflip", action="store_true")
    p_http_ai_pose.add_argument("--hflip", action="store_true")

    parser.add_argument("--verbose", "-v", action="store_true", help="Enable debug logging")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s')

    backend_choice = getattr(args, "backend", BACKEND_AUTO)

    if args.command == "list":
        return list_cameras(backend_choice)

    resolution = (args.width, args.height)

    if args.command == "init":
        return test_init(args.camera, resolution, args.fps, backend_choice)
    if args.command == "processing":
        return test_processing(args.camera, resolution, args.fps, backend_choice)
    if args.command == "stream":
        duration: Optional[int] = args.duration if hasattr(args, "duration") else 0
        return stream(args.camera, resolution, args.fps, duration, backend_choice)
    if args.command == "mjpeg":
        # Build optional image processing config if sub-mode selected
        proc_cfg = None
        if getattr(args, "http_mode", None) == "img-proc":
            # Predefined HSV color ranges
            color_lut = {
                "red": ((0, 50, 50), (10, 255, 255)),
                "green": ((40, 50, 50), (80, 255, 255)),
                "blue": ((90, 50, 50), (130, 255, 255)),
                "yellow": ((20, 80, 80), (35, 255, 255)),
            }
            lower_hsv, upper_hsv = color_lut.get(getattr(args, "color", "red"), ((0, 50, 50), (10, 255, 255)))
            proc_cfg = {
                "enable": True,
                "object_detection": bool(getattr(args, "object_detection", False)),
                "lower_hsv": lower_hsv,
                "upper_hsv": upper_hsv,
                "min_area": int(getattr(args, "min_area", 100)),
                "color_name": getattr(args, "color", "red"),
            }

        # Prefer sub-mode flip flags if provided (support usage: mjpeg <sub> --vflip --hflip)
        vflip = getattr(args, "vflip", False)
        hflip = getattr(args, "hflip", False)

        ai_cfg = None
        if getattr(args, "http_mode", None) in {"ai", "ai-seg", "ai-pose"}:
            def _parse_classes(s: str):
                s = (s or "").strip()
                if not s:
                    return None
                try:
                    return [int(x.strip()) for x in s.split(',') if x.strip() != '']
                except Exception:
                    return None

            ai_cfg = {
                "enable": True,
                "weights": getattr(args, "weights", "yolo11n.pt"),
                "conf": getattr(args, "conf", 0.25),
                "imgsz": getattr(args, "imgsz", 640),
                "device": getattr(args, "device", "cpu"),
                "classes": _parse_classes(getattr(args, "classes", "")),
                "max_det": getattr(args, "max_det", 300),
                "label": getattr(args, "weights", "AI"),
            }
            # Probe init once to capture any error for overlay status
            try:
                _probe = YOLODetector(
                    weights=ai_cfg["weights"],
                    confidence_threshold=float(ai_cfg["conf"]),
                    image_size=int(ai_cfg["imgsz"]),
                    device=ai_cfg["device"],
                    classes=ai_cfg["classes"],
                    max_detections=int(ai_cfg["max_det"]),
                )
                if not _probe.available:
                    ai_cfg["_init_error"] = _probe.last_error or "init failed"
                else:
                    # Discard probe; actual instance will be created in server
                    pass
            except Exception as e:
                ai_cfg["_init_error"] = f"probe: {e}"

        return mjpeg_server(
            host=getattr(args, "host", "0.0.0.0"),
            port=getattr(args, "port", 8080),
            camera_id=getattr(args, "camera", None),
            resolution=resolution,
            framerate=getattr(args, "fps", 30),
            jpeg_quality=getattr(args, "jpeg_quality", 80),
            vflip=vflip,
            hflip=hflip,
            force_both=getattr(args, "both", False),
            backend_choice=getattr(args, "backend", BACKEND_AUTO),
            proc_cfg=proc_cfg,
            ai_cfg=ai_cfg,
            overlay_status=not getattr(args, "no_overlay", False),
        )

    parser.error("Unknown command")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
