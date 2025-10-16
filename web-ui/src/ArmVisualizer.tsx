import { useEffect, useRef } from "react";
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

export function ArmVisualizer({ joints }: ArmVisualizerProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const robotRef = useRef<URDFRobot | null>(null);
  const targetAnglesRef = useRef<number[] | null>(null);
  const currentAnglesRef = useRef<number[] | null>(null);
  const previousTimeRef = useRef<number | null>(null);

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
    camera.position.set(1.4, 1.0, 1.6);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.target.set(0.25, 0.15, 0);

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

        const computeBoundingBox = () => {
          robot.updateMatrixWorld(true, true);
          const bbox = new THREE.Box3().setFromObject(robot);
          const isFiniteBox =
            Number.isFinite(bbox.min.x) &&
            Number.isFinite(bbox.min.y) &&
            Number.isFinite(bbox.min.z) &&
            Number.isFinite(bbox.max.x) &&
            Number.isFinite(bbox.max.y) &&
            Number.isFinite(bbox.max.z);
          if (!isFiniteBox) {
            requestAnimationFrame(computeBoundingBox);
            return;
          }

          const initialSize = bbox.getSize(new THREE.Vector3());

          robot.position.y -= bbox.min.y;
          robot.updateMatrixWorld(true, true);

          const groundedBBox = new THREE.Box3().setFromObject(robot);
          const groundedSize = groundedBBox.getSize(new THREE.Vector3());

          console.info("[ArmVisualizer] Bounding box (grounded)", {
            min: groundedBBox.min.toArray(),
            max: groundedBBox.max.toArray(),
            size: groundedSize.toArray(),
            initialSize: initialSize.toArray(),
          });

          debugMarkers.forEach((marker) => {
            scene.remove(marker);
            marker.traverse((child) => {
              if ((child as THREE.Mesh).isMesh) {
                (child as THREE.Mesh).geometry.dispose();
              }
            });
          });
          debugMarkers = [];

          const cornerColors = [
            0xf87171,
            0xfbbf24,
            0x34d399,
            0x38bdf8,
            0xa855f7,
            0xf472b6,
            0x22d3ee,
            0xf97316,
          ];
          const { min, max } = groundedBBox;
          const corners = [
            new THREE.Vector3(min.x, min.y, min.z),
            new THREE.Vector3(min.x, min.y, max.z),
            new THREE.Vector3(min.x, max.y, min.z),
            new THREE.Vector3(min.x, max.y, max.z),
            new THREE.Vector3(max.x, min.y, min.z),
            new THREE.Vector3(max.x, min.y, max.z),
            new THREE.Vector3(max.x, max.y, min.z),
            new THREE.Vector3(max.x, max.y, max.z),
          ];

          corners.forEach((corner, index) => {
            const marker = new THREE.Mesh(
              new THREE.SphereGeometry(0.005, 12, 12),
              new THREE.MeshBasicMaterial({
                color: cornerColors[index % cornerColors.length],
              }),
            );
            marker.position.copy(corner);
            scene.add(marker);
            debugMarkers.push(marker);
          });
        };

        computeBoundingBox();
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

      const robot = robotRef.current;
      const targetAngles = targetAnglesRef.current;
      if (robot && targetAngles && targetAngles.length > 0) {
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
          joint.setJointValue(nextValue);
          currentAngles[index] = nextValue;
        }
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
    };
  }, []);

  useEffect(() => {
    if (!joints || joints.length === 0) {
      return;
    }
    targetAnglesRef.current = joints.slice();
    if (!currentAnglesRef.current) {
      currentAnglesRef.current = joints.slice();
      const robot = robotRef.current;
      if (robot) {
        joints.forEach((value, index) => {
          const joint = robot.joints[`joint${index + 1}`];
          if (joint) {
            joint.setJointValue(value);
          }
        });
      }
    }
  }, [joints]);

  return <div ref={containerRef} className="absolute inset-0" />;
}
