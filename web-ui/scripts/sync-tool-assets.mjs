import {
  cpSync,
  existsSync,
  mkdirSync,
  readdirSync,
  readFileSync,
  rmSync,
  statSync,
  writeFileSync,
} from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const repoRoot = resolve(__dirname, "..", "..");
const libraryRoot = join(repoRoot, "tools", "library");
const outputRoot = join(repoRoot, "web-ui", "public", "assets", "tools");

function readToolDefinition(toolDirName) {
  const toolDir = join(libraryRoot, toolDirName);
  const defPath = join(toolDir, "tool.json");
  if (!existsSync(defPath) || !statSync(defPath).isFile()) {
    return null;
  }
  const raw = readFileSync(defPath, "utf8");
  const parsed = JSON.parse(raw);
  if (!parsed || typeof parsed !== "object") {
    throw new Error(`Invalid tool definition object: ${defPath}`);
  }
  const toolId =
    typeof parsed.tool_id === "string" && parsed.tool_id.trim().length > 0
      ? parsed.tool_id.trim()
      : toolDirName;
  return { toolId, toolDir, parsed };
}

function listToolDefinitions() {
  if (!existsSync(libraryRoot) || !statSync(libraryRoot).isDirectory()) {
    throw new Error(`Tool library root not found: ${libraryRoot}`);
  }
  const out = [];
  for (const entry of readdirSync(libraryRoot, { withFileTypes: true })) {
    if (!entry.isDirectory()) {
      continue;
    }
    if (entry.name.startsWith(".") || entry.name.startsWith("_")) {
      continue;
    }
    const def = readToolDefinition(entry.name);
    if (def) {
      out.push(def);
    }
  }
  return out;
}

function normalizeAssetPath(assetPath) {
  if (typeof assetPath !== "string" || assetPath.trim().length === 0) {
    return null;
  }
  const normalized = assetPath.replace(/\\/g, "/").replace(/^\/+/, "");
  if (!normalized) {
    return null;
  }
  return normalized;
}

function detectLocalMeshAssetPath(toolDir, toolId) {
  for (const entry of readdirSync(toolDir, { withFileTypes: true })) {
    if (!entry.isFile()) {
      continue;
    }
    const lower = entry.name.toLowerCase();
    if (
      lower.endsWith(".stl") ||
      lower.endsWith(".glb") ||
      lower.endsWith(".gltf")
    ) {
      return `${toolId}/${entry.name}`;
    }
  }
  return null;
}

rmSync(outputRoot, { recursive: true, force: true });
mkdirSync(outputRoot, { recursive: true });

const index = {
  tools: {},
};

for (const entry of listToolDefinitions()) {
  const toolId = entry.toolId;
  const tool = entry.parsed;
  const meshField = tool?.mesh;
  const meshAssetPathRaw = normalizeAssetPath(
    typeof meshField === "string" ? meshField : meshField?.asset_path,
  );
  let meshAssetPath = meshAssetPathRaw;
  if (meshAssetPath && !meshAssetPath.includes("/")) {
    meshAssetPath = `${toolId}/${meshAssetPath}`;
  }
  if (!meshAssetPath) {
    meshAssetPath = detectLocalMeshAssetPath(entry.toolDir, toolId);
  }
  let publicAssetPath = null;
  if (meshAssetPath) {
    const sourcePath = resolve(libraryRoot, meshAssetPath);
    if (existsSync(sourcePath) && statSync(sourcePath).isFile()) {
      const targetPath = join(outputRoot, meshAssetPath);
      mkdirSync(dirname(targetPath), { recursive: true });
      cpSync(sourcePath, targetPath);
      publicAssetPath = `/assets/tools/${meshAssetPath}`;
    } else {
      console.warn(
        `[sync-tool-assets] Mesh asset missing for ${toolId}: ${sourcePath}`,
      );
    }
  }
  index.tools[toolId] = {
    meshAssetPath: publicAssetPath,
  };
}

const indexPath = join(outputRoot, "index.json");
writeFileSync(indexPath, `${JSON.stringify(index, null, 2)}\n`, "utf8");

console.log(
  `[sync-tool-assets] Synced ${Object.keys(index.tools).length} tool definition(s).`,
);
