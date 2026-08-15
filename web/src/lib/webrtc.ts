export type StreamState = "connecting" | "live" | "offline"

function gatewayOrigin() {
  // The production bundle is served by FastAPI, so it must use the page origin.
  // Respect the override only under Vite dev; otherwise a saved localhost value
  // makes a 127.0.0.1 page cross-origin (and vice versa).
  const configured = import.meta.env.DEV
    ? import.meta.env.VITE_GATEWAY_URL?.trim()
    : undefined
  return configured || window.location.origin
}

function gatewayUrl(path: string) {
  return new URL(path, gatewayOrigin()).toString()
}

async function waitForIceGathering(
  peer: RTCPeerConnection,
  signal: AbortSignal,
) {
  if (peer.iceGatheringState === "complete") return

  await new Promise<void>((resolve, reject) => {
    const abort = () => {
      cleanup()
      reject(new DOMException("WebRTC setup aborted", "AbortError"))
    }
    const change = () => {
      if (peer.iceGatheringState === "complete") {
        cleanup()
        resolve()
      }
    }
    const cleanup = () => {
      signal.removeEventListener("abort", abort)
      peer.removeEventListener("icegatheringstatechange", change)
    }

    signal.addEventListener("abort", abort, { once: true })
    peer.addEventListener("icegatheringstatechange", change)
  })
}

export async function connectWebRtcVideo(
  video: HTMLVideoElement,
  signal: AbortSignal,
  onState: (state: StreamState) => void,
) {
  const peer = new RTCPeerConnection()
  onState("connecting")

  peer.addTransceiver("video", { direction: "recvonly" })
  peer.addEventListener("track", ({ track, streams }) => {
    video.srcObject = streams[0] ?? new MediaStream([track])
    void video.play().catch(() => undefined)
  })
  peer.addEventListener("connectionstatechange", () => {
    if (peer.connectionState === "connected") onState("live")
    if (["closed", "disconnected", "failed"].includes(peer.connectionState)) {
      onState("offline")
    }
  })

  const offer = await peer.createOffer()
  await peer.setLocalDescription(offer)
  await waitForIceGathering(peer, signal)

  const response = await fetch(gatewayUrl("/api/webrtc/offer"), {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      sdp: peer.localDescription?.sdp,
      type: peer.localDescription?.type,
    }),
    signal,
  })
  if (!response.ok) {
    throw new Error(`WebRTC offer failed with HTTP ${response.status}`)
  }

  const answer = (await response.json()) as RTCSessionDescriptionInit
  await peer.setRemoteDescription(answer)
  return peer
}

export function websocketUrl() {
  const url = new URL("/ws/events", gatewayOrigin())
  url.protocol = url.protocol === "https:" ? "wss:" : "ws:"
  return url.toString()
}
