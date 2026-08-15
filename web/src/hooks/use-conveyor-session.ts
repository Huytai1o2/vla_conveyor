import * as React from "react"

import { websocketUrl } from "@/lib/webrtc"

export type GatewayState = "connecting" | "connected" | "offline"
export type Severity =
  | "USER"
  | "INFO"
  | "SUCCESS"
  | "WARNING"
  | "RECONNECTING"
  | "ERROR"
export type MessagePresentation = "user" | "thinking" | "final"

export interface ConversationMessage {
  id: string
  severity: Severity
  message: string
  timestamp: Date
  runId?: string
  presentation?: MessagePresentation
}

export interface ConveyorTelemetry {
  step: number
  direction: "LEFT" | "RIGHT" | "STOP"
  waypointIndex: number | null
  waypointCount: number | null
  destinationX: number | null
  camera: string
  mcu: string
  model: string
  calibrationPoints: number
  calibrationValid: boolean
  frameMode: "calibration" | "calibrated"
  previewWidth: number
  previewHeight: number
}

type GatewayEvent =
  | {
      kind: "message"
      severity: Severity
      message: string
      timestamp?: number
      runId?: string
      presentation?: MessagePresentation
    }
  | { kind: "state"; state: string }
  | { kind: "prompt_ready" }
  | { kind: "fatal" }
  | {
      kind: "command_result"
      command: string
      ok: boolean
      error?: string
    }
  | ({ kind: "telemetry" } & Partial<ConveyorTelemetry>)

const initialTelemetry: ConveyorTelemetry = {
  step: 0,
  direction: "STOP",
  waypointIndex: null,
  waypointCount: null,
  destinationX: null,
  camera: "Waiting",
  mcu: "Waiting",
  model: "Waiting",
  calibrationPoints: 0,
  calibrationValid: false,
  frameMode: "calibration",
  previewWidth: 1000,
  previewHeight: 600,
}

function makeMessage(
  severity: Severity,
  message: string,
  metadata: {
    timestamp?: number
    runId?: string
    presentation?: MessagePresentation
  } = {},
): ConversationMessage {
  return {
    id: crypto.randomUUID(),
    severity,
    message,
    timestamp: metadata.timestamp
      ? new Date(metadata.timestamp * 1000)
      : new Date(),
    runId: metadata.runId,
    presentation: metadata.presentation,
  }
}

export function useConveyorSession() {
  const socketRef = React.useRef<WebSocket | null>(null)
  const [gatewayState, setGatewayState] = React.useState<GatewayState>("connecting")
  const [controllerState, setControllerState] = React.useState("CONNECTING")
  const [promptReady, setPromptReady] = React.useState(false)
  const [telemetry, setTelemetry] = React.useState(initialTelemetry)
  const [messages, setMessages] = React.useState<ConversationMessage[]>([
    makeMessage("INFO", "Dashboard ready. Waiting for the conveyor gateway."),
  ])

  React.useEffect(() => {
    let disposed = false
    let reconnectTimer: number | undefined

    const connect = () => {
      if (disposed) return
      setGatewayState("connecting")
      const socket = new WebSocket(websocketUrl())
      socketRef.current = socket

      socket.addEventListener("open", () => {
        setGatewayState("connected")
        setMessages([])
      })
      socket.addEventListener("message", (event) => {
        try {
          const payload = JSON.parse(String(event.data)) as GatewayEvent
          if (payload.kind === "message") {
            setMessages((current) => [
              ...current,
              makeMessage(payload.severity, payload.message, payload),
            ])
          } else if (payload.kind === "state") {
            setControllerState(payload.state)
            setPromptReady(payload.state === "WAITING_FOR_PROMPT")
          } else if (payload.kind === "prompt_ready") {
            setPromptReady(true)
          } else if (payload.kind === "fatal") {
            setPromptReady(false)
          } else if (payload.kind === "command_result") {
            if (!payload.ok) {
              setMessages((current) => [
                ...current,
                makeMessage(
                  "ERROR",
                  `${payload.command}: ${payload.error ?? "Command failed."}`,
                ),
              ])
            }
          } else if (payload.kind === "telemetry") {
            setTelemetry((current) => ({ ...current, ...payload }))
          }
        } catch {
          setMessages((current) => [
            ...current,
            makeMessage("ERROR", "Gateway sent an unreadable event."),
          ])
        }
      })
      socket.addEventListener("close", () => {
        if (socketRef.current === socket) socketRef.current = null
        setGatewayState("offline")
        setPromptReady(false)
        if (!disposed) reconnectTimer = window.setTimeout(connect, 2500)
      })
      socket.addEventListener("error", () => socket.close())
    }

    connect()
    return () => {
      disposed = true
      if (reconnectTimer !== undefined) window.clearTimeout(reconnectTimer)
      socketRef.current?.close()
      socketRef.current = null
    }
  }, [])

  const sendCommand = React.useCallback((command: Record<string, unknown>) => {
    const socket = socketRef.current
    if (!socket || socket.readyState !== WebSocket.OPEN) return false
    socket.send(JSON.stringify(command))
    return true
  }, [])

  const sendPrompt = React.useCallback((instruction: string) => {
    if (!promptReady) return false
    const sent = sendCommand({ kind: "prompt", instruction })
    if (sent) setPromptReady(false)
    return sent
  }, [promptReady, sendCommand])

  const stop = React.useCallback(() => {
    sendCommand({ kind: "stop" })
    setPromptReady(false)
  }, [sendCommand])

  return {
    gatewayState,
    controllerState,
    promptReady,
    telemetry,
    messages,
    sendPrompt,
    stop,
    captureCalibration: () => sendCommand({ kind: "calibration_capture" }),
    resumeCalibrationLive: () => sendCommand({ kind: "calibration_live" }),
    addCalibrationPoint: (x: number, y: number) =>
      sendCommand({ kind: "calibration_point", x, y }),
    undoCalibrationPoint: () => sendCommand({ kind: "calibration_undo" }),
    resetCalibrationPoints: () => sendCommand({ kind: "calibration_reset" }),
    confirmCalibration: () => sendCommand({ kind: "calibration_confirm" }),
  }
}
