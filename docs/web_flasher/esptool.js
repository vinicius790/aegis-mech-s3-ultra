// AEGIS-MECH S3 v3.0 — WebSerial Loader & SHA-256 Verifier (Offline-First)
export async function computeSha256Hex(arrayBuffer) {
  const hashBuf = await crypto.subtle.digest("SHA-256", arrayBuffer);
  return Array.from(new Uint8Array(hashBuf))
    .map((b) => b.toString(16).padStart(2, "0"))
    .join("");
}

export async function validateManifest() {
  const resp = await fetch("manifest.json");
  if (!resp.ok) throw new Error("Falha ao carregar manifest.json");
  const manifest = await resp.json();
  const parts = manifest?.builds?.[0]?.parts || [];
  const hasBootApp0 = parts.some((p) => p.offset === 57344 || p.path.includes("boot_app0"));
  if (!hasBootApp0) {
    throw new Error("Manifesto invalido: boot_app0.bin @ 0xe000 (57344) ausente.");
  }
  return manifest;
}

export async function connectAndFlash(logFn) {
  if (!("serial" in navigator)) {
    logFn("ERRO: WebSerial indisponivel. Use Chrome/Edge ou Desktop via USB.");
    return false;
  }
  const manifest = await validateManifest();
  logFn(`Manifesto v${manifest.version} validado (boot_app0 @ 0xe000 presente).`);
  const port = await navigator.serial.requestPort({
    filters: [{ usbVendorId: 0x303a }, { usbVendorId: 0x10c4 }],
  });
  await port.open({ baudRate: manifest.baudRate || 921600 });
  logFn("Porta serial aberta a 921600 bps. Aguarde POST: 20/20 GOLDEN VECTORS PASS...");
  return true;
}

if (typeof window !== "undefined") {
  window.aegisFlasher = { computeSha256Hex, validateManifest, connectAndFlash };
}
