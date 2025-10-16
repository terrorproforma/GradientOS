export function resolveDefaultApiHost(): string {
  const { protocol, hostname } = window.location;
  const apiPort = 8000;
  return `${protocol}//${hostname}:${apiPort}`;
}

export function resolveDefaultVisionHost(): string {
  const { protocol, hostname } = window.location;
  const visionPort = 8080;
  return `${protocol}//${hostname}:${visionPort}`;
}
