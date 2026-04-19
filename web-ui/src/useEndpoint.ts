export function resolveDefaultApiHost(): string {
  const { protocol, hostname } = window.location;
  // NOTE: API port is 4400 (not 4000). Port 4000 is grabbed by Windows
  // `iphlpsvc` (IP Helper) dynamically on many machines, which breaks Cursor
  // Remote-SSH port forwarding (`Local port 4000 could not be used...`
  // popup followed by a random high port fallback that the browser then
  // cannot reach via `hostname:4000`). Keep this in sync with
  // `start-stack.sh` `API_PORT` default and `api/main.py` `--port` default.
  const apiPort = 4400;
  return `${protocol}//${hostname}:${apiPort}`;
}

export function resolveDefaultVisionHost(): string {
  const { protocol, hostname } = window.location;
  const visionPort = 8080;
  return `${protocol}//${hostname}:${visionPort}`;
}
