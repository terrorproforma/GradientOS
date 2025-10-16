import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls";
import URDFLoader, { type URDFRobot } from "urdf-loader";

type ArmVisualizerProps = {
  joints?: number[];
};

const GRID_CELL_SIZE = 0.1; // 10 cm per square
const GRID_CELLS_PER_SIDE = 40; // spans 4 m total (enough workspace)
const GRID_SIZE = GRID_CELL_SIZE * GRID_CELLS_PER_SIDE;
const ROBOT_SCALE = 1; // Temporary until we confirm units

export function ArmVisualizer({ joints }: ArmVisualizerProps) {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const robotRef = useRef<URDFRobot | null>(null);

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
    loader.load(
      urdfPath,
      (robot) => {
        console.info("[ArmVisualizer] URDF loaded", {
          jointNames: Object.keys(robot.joints),
          linkNames: Object.keys(robot.links),
        });
        robot.scale.setScalar(ROBOT_SCALE);
        const bbox = new THREE.Box3().setFromObject(robot);
        const size = new THREE.Vector3();
        bbox.getSize(size);
        console.info("[ArmVisualizer] Bounding box", {
          min: bbox.min.toArray(),
          max: bbox.max.toArray(),
          size: size.toArray(),
        });
        const offset = new THREE.Vector3();
        bbox.getCenter(offset).negate();
        robot.position.copy(offset);

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
        const corners = bbox.getCorners
          ? (bbox.getCorners() as THREE.Vector3[])
          : [
              new THREE.Vector3(bbox.min.x, bbox.min.y, bbox.min.z),
              new THREE.Vector3(bbox.min.x, bbox.min.y, bbox.max.z),
              new THREE.Vector3(bbox.min.x, bbox.max.y, bbox.min.z),
              new THREE.Vector3(bbox.min.x, bbox.max.y, bbox.max.z),
              new THREE.Vector3(bbox.max.x, bbox.min.y, bbox.min.z),
              new THREE.Vector3(bbox.max.x, bbox.min.y, bbox.max.z),
              new THREE.Vector3(bbox.max.x, bbox.max.y, bbox.min.z),
              new THREE.Vector3(bbox.max.x, bbox.max.y, bbox.max.z),
            ];
        corners.forEach((corner, index) => {
          const marker = new THREE.Mesh(
            new THREE.SphereGeometry(0.02, 12, 12),
            new THREE.MeshStandardMaterial({
              color: cornerColors[index % cornerColors.length],
              emissive: cornerColors[index % cornerColors.length],
              emissiveIntensity: 1.0,
            }),
          );
          marker.position.copy(corner.clone().add(offset));
          scene.add(marker);
        });

        robotRef.current = robot;
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
    const animate = () => {
      animationFrameId = requestAnimationFrame(animate);
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
    };
  }, []);

  useEffect(() => {
    if (!joints || joints.length === 0) {
      return;
    }
    const robot = robotRef.current;
    if (!robot) {
      return;
    }
    joints.forEach((value, index) => {
      const jointName = `joint${index + 1}`;
      const joint = robot.joints[jointName];
      if (joint) {
        joint.setJointValue(value);
      }
    });
  }, [joints]);

  return <div ref={containerRef} className="absolute inset-0" />;
}
