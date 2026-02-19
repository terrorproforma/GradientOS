import { cpSync, existsSync, mkdirSync, readdirSync, readFileSync, rmSync, statSync, writeFileSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const repoRoot = resolve(__dirname, "..", "..");
const robotsRoot = join(repoRoot, "robots");
const outputRoot = join(repoRoot, "web-ui", "public", "assets", "robots");

function readManifest(robotDir) {
  const manifestPath = join(robotDir, "robot.json");
  if (!existsSync(manifestPath)) {
    throw new Error(`Missing robot manifest: ${manifestPath}`);
  }

  const raw = readFileSync(manifestPath, "utf8");
  const manifest = JSON.parse(raw);
  if (!manifest || typeof manifest !== "object") {
    throw new Error(`Invalid manifest JSON object: ${manifestPath}`);
  }
  if (typeof manifest.robot_id !== "string" || manifest.robot_id.length === 0) {
    throw new Error(`Manifest missing required robot_id: ${manifestPath}`);
  }
  if (!manifest.web || typeof manifest.web !== "object") {
    throw new Error(`Manifest missing required web section: ${manifestPath}`);
  }
  if (typeof manifest.web.asset_source_dir !== "string" || manifest.web.asset_source_dir.length === 0) {
    throw new Error(`Manifest missing required web.asset_source_dir: ${manifestPath}`);
  }
  if (typeof manifest.web.urdf !== "string" || manifest.web.urdf.length === 0) {
    throw new Error(`Manifest missing required web.urdf: ${manifestPath}`);
  }
  return manifest;
}

function syncRobotAsset(manifest, robotDir, index) {
  const robotId = manifest.robot_id;
  const sourceDir = resolve(robotDir, manifest.web.asset_source_dir);
  if (!existsSync(sourceDir) || !statSync(sourceDir).isDirectory()) {
    throw new Error(`Invalid asset source directory for ${robotId}: ${sourceDir}`);
  }

  const targetDir = join(outputRoot, robotId);
  mkdirSync(targetDir, { recursive: true });

  const sourceUrdfPath = join(sourceDir, manifest.web.urdf);
  if (!existsSync(sourceUrdfPath) || !statSync(sourceUrdfPath).isFile()) {
    throw new Error(`web.urdf path not found for ${robotId}: ${sourceUrdfPath}`);
  }
  const targetUrdfPath = join(targetDir, manifest.web.urdf);
  mkdirSync(dirname(targetUrdfPath), { recursive: true });
  cpSync(sourceUrdfPath, targetUrdfPath);

  // Common convention: if the URDF references a sibling stl-files/ directory,
  // mirror it for browser loading.
  const sourceStlDir = join(sourceDir, "stl-files");
  if (existsSync(sourceStlDir) && statSync(sourceStlDir).isDirectory()) {
    cpSync(sourceStlDir, join(targetDir, "stl-files"), { recursive: true });
  }

  // Preserve USD scene files as optional pass-through assets.
  // These are never required and should not block sync/build if absent.
  for (const entry of readdirSync(sourceDir, { withFileTypes: true })) {
    if (!entry.isFile()) {
      continue;
    }
    if (!/\.(usd|usda|usdc)$/i.test(entry.name)) {
      continue;
    }
    cpSync(join(sourceDir, entry.name), join(targetDir, entry.name));
  }

  const canonicalUrdfPath = join(targetDir, "robot.urdf");
  cpSync(targetUrdfPath, canonicalUrdfPath);

  index.robots[robotId] = {
    urdfPath: `/assets/robots/${robotId}/robot.urdf`,
  };

  if (manifest.default === true) {
    if (index.defaultRobotId && index.defaultRobotId !== robotId) {
      throw new Error(
        `Multiple default robots found (${index.defaultRobotId}, ${robotId}). Exactly one default is required.`,
      );
    }
    index.defaultRobotId = robotId;
  }
}

if (!existsSync(robotsRoot) || !statSync(robotsRoot).isDirectory()) {
  throw new Error(`Robots catalog not found: ${robotsRoot}`);
}

rmSync(outputRoot, { recursive: true, force: true });
mkdirSync(outputRoot, { recursive: true });

const index = {
  defaultRobotId: null,
  robots: {},
};

for (const entry of readdirSync(robotsRoot, { withFileTypes: true })) {
  if (!entry.isDirectory()) {
    continue;
  }
  const robotDir = join(robotsRoot, entry.name);
  const manifest = readManifest(robotDir);
  if (manifest.robot_id !== entry.name) {
    throw new Error(
      `Manifest robot_id mismatch in ${robotDir}: expected '${entry.name}', got '${manifest.robot_id}'.`,
    );
  }
  syncRobotAsset(manifest, robotDir, index);
}

if (!index.defaultRobotId) {
  throw new Error("No default robot found. Set `default: true` in exactly one robots/<id>/robot.json.");
}

const indexPath = join(outputRoot, "index.json");
writeFileSync(indexPath, `${JSON.stringify(index, null, 2)}\n`, "utf8");

console.log(`[sync-robot-assets] Synced ${Object.keys(index.robots).length} robot asset bundle(s).`);
console.log(`[sync-robot-assets] Default robot: ${index.defaultRobotId}`);
