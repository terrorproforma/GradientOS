import {
  forwardRef,
  type ForwardedRef,
  useEffect,
  useImperativeHandle,
  useRef,
} from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import URDFLoader, { type URDFRobot } from "urdf-loader";

type ArmVisualizerProps = {
  joints?: number[];
};

const GRID_CELL_SIZE = 0.05; // 10 cm per square
const GRID_CELLS_PER_SIDE = 80; // spans 4 m total (enough workspace)
const GRID_SIZE = GRID_CELL_SIZE * GRID_CELLS_PER_SIDE;
const ROBOT_SCALE = 1; // make the robot look bit bigger
const BOUNDING_MARKER_COLORS = [
  0xf87171,
  0xfbbf24,
  0x34d399,
  0x38bdf8,
  0xa855f7,
  0xf472b6,
  0x22d3ee,
  0xf97316,
] as const;

export type ArmVisualizerHandle = {
  resetView: () => void;
};

export const ArmVisualizer = forwardRef(function ArmVisualizer(
  { joints }: ArmVisualizerProps,
  ref: ForwardedRef<ArmVisualizerHandle>,
) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const robotRef = useRef<URDFRobot | null>(null);
  const targetAnglesRef = useRef<number[] | null>(null);
  const currentAnglesRef = useRef<number[] | null>(null);
  const previousTimeRef = useRef<number | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const initialControlsTarget = useRef(new THREE.Vector3(0.25, 0.15, 0));
  const defaultCameraOffset = useRef(new THREE.Vector3(1.15, 0.85, 1.6));
  const defaultCameraOffsetCaptured = useRef(false);
  const isGroundedRef = useRef(false);
  const boundingCenterRef = useRef(new THREE.Vector3());
  const pendingDynamicBoundsRef = useRef(false);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) {
      return;
    }

    const scene = new THREE.Scene();
    scene.background = new THREE.Color("#020617");

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    const camera = new THREE.PerspectiveCamera(
      45,
      container.clientWidth / Math.max(1, container.clientHeight),
      0.05,
      50,
    );
    camera.position
      .copy(initialControlsTarget.current)
      .add(defaultCameraOffset.current);
    cameraRef.current = camera;

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.copy(initialControlsTarget.current);
    controlsRef.current = controls;

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const keyLight = new THREE.DirectionalLight(0xffffff, 1.2);
    keyLight.position.set(1.5, 2, 1.5);
    scene.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0x89cff0, 0.4);
    fillLight.position.set(-1.5, 1, -1.5);
    scene.add(fillLight);

    const hemiLight = new THREE.HemisphereLight(0x9be7ff, 0x0f172a, 0.3);
    scene.add(hemiLight);

    const grid = new THREE.GridHelper(
      GRID_SIZE,
      GRID_CELLS_PER_SIDE,
      0x38bdf8,
      0x1e293b,
    );
    scene.add(grid);

    const loader = new URDFLoader();
    const assetBasePath = "/assets/mini-6dof-arm/";
    const urdfPath = `${assetBasePath}mini-6dof-arm.urdf`;
    loader.workingPath = assetBasePath;
    let debugMarkers: THREE.Object3D[] = [];
    const isFiniteBox = (box: THREE.Box3) =>
      Number.isFinite(box.min.x) &&
      Number.isFinite(box.min.y) &&
      Number.isFinite(box.min.z) &&
      Number.isFinite(box.max.x) &&
      Number.isFinite(box.max.y) &&
      Number.isFinite(box.max.z);

    const updateBoundingMarkers = (box: THREE.Box3) => {
      const corners = [
        new THREE.Vector3(box.min.x, box.min.y, box.min.z),
        new THREE.Vector3(box.min.x, box.min.y, box.max.z),
        new THREE.Vector3(box.min.x, box.max.y, box.min.z),
        new THREE.Vector3(box.min.x, box.max.y, box.max.z),
        new THREE.Vector3(box.max.x, box.min.y, box.min.z),
        new THREE.Vector3(box.max.x, box.min.y, box.max.z),
        new THREE.Vector3(box.max.x, box.max.y, box.min.z),
        new THREE.Vector3(box.max.x, box.max.y, box.max.z),
      ];

      if (debugMarkers.length === 0) {
        corners.forEach((corner, index) => {
          const marker = new THREE.Mesh(
            new THREE.SphereGeometry(0.005, 12, 12),
            new THREE.MeshBasicMaterial({
              color: BOUNDING_MARKER_COLORS[index % BOUNDING_MARKER_COLORS.length],
            }),
          );
          marker.position.copy(corner);
          scene.add(marker);
          debugMarkers.push(marker);
        });
      } else {
        debugMarkers.forEach((marker, index) => {
          marker.position.copy(corners[index]);
        });
      }
    };

    const alignToGroundAndUpdateBounds = (options?: {
      snapCamera?: boolean;
      applySnapshot?: boolean;
    }) => {
      const robot = robotRef.current;
      if (!robot) {
        return;
      }
      robot.updateMatrixWorld(true, true);

      const candidateNames = ["base", "base_link", "link0", "base_link_inertia"];
      let baseObject: THREE.Object3D | null = null;
      const linksRecord = robot.links as unknown as Record<
        string,
        THREE.Object3D | undefined
      >;
      for (const name of candidateNames) {
        const fromLinks = linksRecord?.[name];
        if (fromLinks) {
          baseObject = fromLinks;
          break;
        }
        const found = robot.getObjectByName(name);
        if (found) {
          baseObject = found;
          break;
        }
      }

      const baseSource =
        baseObject ?? linksRecord?.base ?? linksRecord?.base_link ?? robot;
      const baseBox = new THREE.Box3().setFromObject(baseSource);
      if (!isFiniteBox(baseBox)) {
        return;
      }

      const deltaY = baseBox.min.y;
      if (Number.isFinite(deltaY) && Math.abs(deltaY) > 1e-5) {
        robot.position.y -= deltaY;
        robot.updateMatrixWorld(true, true);
      }

      const groundedBBox = new THREE.Box3().setFromObject(robot);
      if (!isFiniteBox(groundedBBox)) {
        return;
      }

      const center = groundedBBox.getCenter(boundingCenterRef.current);
      initialControlsTarget.current.copy(center);

      updateBoundingMarkers(groundedBBox);

      if (options?.applySnapshot) {
        const values = targetAnglesRef.current;
        if (values) {
          currentAnglesRef.current = values.slice();
          values.forEach((value, index) => {
            const joint = robot.joints[`joint${index + 1}`];
            if (joint) {
              joint.setJointValue(value);
            }
          });
        }
      }

      const controlsInstance = controlsRef.current;
      const cameraInstance = cameraRef.current;
      if (options?.snapCamera && controlsInstance && cameraInstance) {
        const offset = cameraInstance.position
          .clone()
          .sub(controlsInstance.target);
        controlsInstance.target.copy(initialControlsTarget.current);
        cameraInstance.position
          .copy(initialControlsTarget.current)
          .add(offset);
        cameraInstance.updateProjectionMatrix();
        controlsInstance.update();
        if (!defaultCameraOffsetCaptured.current) {
          defaultCameraOffset.current.copy(offset);
          defaultCameraOffsetCaptured.current = true;
        }
      }
    };

    const scheduleBoundingRefresh = () => {
      if (!isGroundedRef.current) {
        return;
      }
      pendingDynamicBoundsRef.current = true;
    };

    loader.load(
      urdfPath,
      (robot) => {
        console.info("[ArmVisualizer] URDF loaded", {
          jointNames: Object.keys(robot.joints),
          linkNames: Object.keys(robot.links),
        });
        robot.scale.setScalar(ROBOT_SCALE);
        robot.rotation.x = -Math.PI / 2;

        const defaultMaterial = new THREE.MeshStandardMaterial({
          color: 0x1e293b,
          metalness: 0.1,
          roughness: 0.8,
        });
        const accentMaterial = new THREE.MeshStandardMaterial({
          color: 0x38bdf8,
          metalness: 0.2,
          roughness: 0.5,
        });

        robot.traverse((obj) => {
          if ((obj as THREE.Mesh).isMesh) {
            const mesh = obj as THREE.Mesh;
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            const material =
              mesh.name.toLowerCase().includes("wrist") ||
              mesh.name.toLowerCase().includes("tool")
                ? accentMaterial
                : defaultMaterial;

            if (Array.isArray(mesh.material)) {
              mesh.material.forEach((mat) => {
                (mat as THREE.Material).dispose();
              });
            } else if (mesh.material) {
              (mesh.material as THREE.Material).dispose();
            }
            mesh.material = material;
          }
        });

        scene.add(robot);

        robotRef.current = robot;
        if (!currentAnglesRef.current) {
          currentAnglesRef.current = new Array(6).fill(0);
        }
        if (!targetAnglesRef.current) {
          targetAnglesRef.current = new Array(6).fill(0);
        }

        const initialiseScene = () => {
          alignToGroundAndUpdateBounds({ snapCamera: true, applySnapshot: true });
          isGroundedRef.current = true;
        };

        initialiseScene();
        renderer.render(scene, camera);
      },
      undefined,
      (error) => {
        console.error("[ArmVisualizer] Failed to load URDF", {
          error,
          attemptedPath: urdfPath,
        });
      },
    );

    const handleResize = () => {
      if (!container) {
        return;
      }
      const { clientWidth, clientHeight } = container;
      renderer.setSize(clientWidth, clientHeight);
      camera.aspect = clientWidth / Math.max(clientHeight, 1);
      camera.updateProjectionMatrix();
    };

    let animationFrameId: number;
    const animate = (time?: number) => {
      animationFrameId = requestAnimationFrame(animate);

      const deltaSeconds =
        previousTimeRef.current !== null && time !== undefined
          ? Math.min((time - previousTimeRef.current) / 1000, 0.05)
          : 0.016;
      previousTimeRef.current = time ?? null;

      const targetAngles = targetAnglesRef.current;
      const robot = robotRef.current;
      let jointsChanged = false;
      if (robot && targetAngles && targetAngles.length > 0 && isGroundedRef.current) {
        if (!currentAnglesRef.current) {
          currentAnglesRef.current = targetAngles.slice();
        }
        const currentAngles = currentAnglesRef.current!;
        const jointsMap = robot.joints;
        const smoothing = 12; // rad/s tracking speed

        for (let index = 0; index < targetAngles.length; index += 1) {
          const jointName = `joint${index + 1}`;
          const joint = jointsMap[jointName];
          if (!joint) {
            continue;
          }
          const currentValue =
            typeof currentAngles[index] === "number" ? currentAngles[index] : 0;
          const targetValue = targetAngles[index];
          if (!Number.isFinite(targetValue)) {
            continue;
          }
          const blend = Math.min(1, deltaSeconds * smoothing);
          const nextValue = currentValue + (targetValue - currentValue) * (Number.isFinite(blend) ? blend : 1);
          if (Math.abs(nextValue - currentValue) > 1e-5) {
            jointsChanged = true;
          }
          joint.setJointValue(nextValue);
          currentAngles[index] = nextValue;
        }
      }

      if (jointsChanged) {
        scheduleBoundingRefresh();
      }

      if (pendingDynamicBoundsRef.current) {
        pendingDynamicBoundsRef.current = false;
        alignToGroundAndUpdateBounds({ snapCamera: false, applySnapshot: false });
      }

      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    window.addEventListener("resize", handleResize);

    return () => {
      window.removeEventListener("resize", handleResize);
      cancelAnimationFrame(animationFrameId);
      controls.dispose();
      controlsRef.current = null;
      renderer.dispose();
      grid.geometry.dispose();
      if (Array.isArray(grid.material)) {
        grid.material.forEach((material) => material.dispose());
      } else {
        grid.material.dispose();
      }
      if (container.contains(renderer.domElement)) {
        container.removeChild(renderer.domElement);
      }
      scene.clear();
      robotRef.current = null;
      targetAnglesRef.current = null;
      currentAnglesRef.current = null;
      previousTimeRef.current = null;
      cameraRef.current = null;
      isGroundedRef.current = false;
      pendingDynamicBoundsRef.current = false;
    };
  }, []);

  useImperativeHandle(
    ref,
    () => ({
      resetView: () => {
        const camera = cameraRef.current;
        const controls = controlsRef.current;
        if (!camera || !controls) {
          return;
        }
        controls.target.copy(initialControlsTarget.current);
        camera
          .position.copy(initialControlsTarget.current)
          .add(defaultCameraOffset.current);
        camera.updateProjectionMatrix();
        controls.update();
      },
    }),
    [],
  );

  useEffect(() => {
    if (!joints || joints.length === 0) {
      return;
    }
    targetAnglesRef.current = joints.slice();
    if (!currentAnglesRef.current) {
      currentAnglesRef.current = joints.slice();
    }
    if (!isGroundedRef.current) {
      return;
    }
    const robot = robotRef.current;
    if (robot) {
      joints.forEach((value, index) => {
        const joint = robot.joints[`joint${index + 1}`];
        if (joint) {
          joint.setJointValue(value);
        }
      });
      pendingDynamicBoundsRef.current = true;
    }
  }, [joints]);

  return <div ref={containerRef} className="absolute inset-0" />;
});
