export function resolveDefaultApiHost(): string {
  const { protocol, hostname } = window.location;
  const apiPort = 8000;
  return `${protocol}//${hostname}:${apiPort}`;
}
