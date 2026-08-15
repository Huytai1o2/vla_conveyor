import * as React from "react"
import {
  ArrowUp,
  Camera,
  ChevronDown,
  CircleAlert,
  CircleCheck,
  CircleX,
  Cpu,
  Expand,
  Info,
  LoaderCircle,
  Maximize2,
  Radio,
  RotateCcw,
  Square,
  Waypoints,
  Wifi,
  WifiOff,
} from "lucide-react"

import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Separator } from "@/components/ui/separator"
import { Textarea } from "@/components/ui/textarea"
import {
  type ConversationMessage,
  type GatewayState,
  useConveyorSession,
} from "@/hooks/use-conveyor-session"
import { cn } from "@/lib/utils"
import { connectWebRtcVideo, type StreamState } from "@/lib/webrtc"

function ConnectionMark({ state }: { state: GatewayState | StreamState }) {
  const connected = state === "connected" || state === "live"
  const connecting = state === "connecting"
  return (
    <span
      className={cn(
        "size-1.5 rounded-full",
        connected ? "bg-zinc-950" : "bg-zinc-400",
        connecting && "animate-pulse",
      )}
    />
  )
}

function Header({
  gatewayState,
  onStop,
}: {
  gatewayState: GatewayState
  onStop: () => void
}) {
  const toggleFullscreen = () => {
    if (document.fullscreenElement) void document.exitFullscreen()
    else void document.documentElement.requestFullscreen()
  }

  return (
    <header className="flex h-14 shrink-0 items-center justify-between border-b border-zinc-200 px-4 sm:px-5">
      <div className="flex min-w-0 items-center gap-3">
        <div className="grid size-7 place-items-center rounded-md bg-zinc-950 text-white">
          <Waypoints className="size-3.5" />
        </div>
        <div className="min-w-0">
          <h1 className="truncate text-sm font-semibold tracking-tight">Conveyor VLA</h1>
          <p className="font-mono text-[10px] uppercase tracking-[0.12em] text-zinc-500">
            Calibrated controller
          </p>
        </div>
      </div>

      <div className="flex items-center gap-1.5">
        <div className="mr-2 hidden items-center gap-2 text-xs text-zinc-500 sm:flex">
          <ConnectionMark state={gatewayState} />
          <span className="capitalize">Gateway {gatewayState}</span>
        </div>
        <Button variant="ghost" size="icon" onClick={toggleFullscreen} title="Toggle full-screen">
          <Expand />
        </Button>
        <Button variant="outline" size="sm" onClick={onStop}>
          <Square className="fill-current" />
          Stop / Exit
        </Button>
      </div>
    </header>
  )
}

function useVideoStream(videoRef: React.RefObject<HTMLVideoElement | null>) {
  const [streamState, setStreamState] = React.useState<StreamState>("connecting")

  React.useEffect(() => {
    const video = videoRef.current
    if (!video) return
    const abortController = new AbortController()
    let peer: RTCPeerConnection | undefined
    let retryTimer: number | undefined

    const connect = async () => {
      try {
        peer = await connectWebRtcVideo(video, abortController.signal, setStreamState)
      } catch (error) {
        if (abortController.signal.aborted) return
        console.warn("WebRTC stream unavailable", error)
        setStreamState("offline")
        retryTimer = window.setTimeout(connect, 3000)
      }
    }
    void connect()

    return () => {
      abortController.abort()
      if (retryTimer !== undefined) window.clearTimeout(retryTimer)
      peer?.close()
      video.srcObject = null
    }
  }, [videoRef])

  return streamState
}

function Metric({
  icon: Icon,
  label,
  value,
}: {
  icon: React.ComponentType<{ className?: string }>
  label: string
  value: string
}) {
  return (
    <div className="flex min-w-0 items-center gap-2.5">
      <Icon className="size-3.5 shrink-0 text-zinc-400" />
      <div className="min-w-0">
        <p className="font-mono text-[9px] uppercase tracking-[0.12em] text-zinc-400">{label}</p>
        <p className="truncate text-xs font-medium text-zinc-800">{value}</p>
      </div>
    </div>
  )
}

function StreamPanel({
  controllerState,
  gatewayState,
  telemetry,
  addCalibrationPoint,
}: ReturnType<typeof useConveyorSession>) {
  const videoRef = React.useRef<HTMLVideoElement>(null)
  const [videoFit, setVideoFit] = React.useState<"contain" | "cover">("contain")
  const streamState = useVideoStream(videoRef)
  const waypoint =
    telemetry.waypointIndex === null || telemetry.waypointCount === null
      ? "No active plan"
      : `${telemetry.waypointIndex + 1} / ${telemetry.waypointCount} · x=${telemetry.destinationX ?? "—"}`
  const selecting = controllerState === "CALIBRATION_SELECTING"

  const selectCalibrationPoint = (event: React.MouseEvent<HTMLVideoElement>) => {
    const video = videoRef.current
    if (!selecting || streamState !== "live" || !video?.videoWidth || !video.videoHeight) return
    const bounds = video.getBoundingClientRect()
    const scale = videoFit === "contain"
      ? Math.min(bounds.width / video.videoWidth, bounds.height / video.videoHeight)
      : Math.max(bounds.width / video.videoWidth, bounds.height / video.videoHeight)
    const renderedWidth = video.videoWidth * scale
    const renderedHeight = video.videoHeight * scale
    const originX = (bounds.width - renderedWidth) / 2
    const originY = (bounds.height - renderedHeight) / 2
    const sourceX = (event.clientX - bounds.left - originX) / scale
    const sourceY = (event.clientY - bounds.top - originY) / scale
    if (
      sourceX < 0 || sourceY < 0 ||
      sourceX >= video.videoWidth || sourceY >= video.videoHeight
    ) return
    addCalibrationPoint(sourceX / video.videoWidth, sourceY / video.videoHeight)
  }

  return (
    <section className="flex min-h-0 flex-col gap-3 border-b border-zinc-200 p-3 lg:border-r lg:border-b-0">
      <div className="relative min-h-[340px] flex-1 overflow-hidden rounded-xl bg-zinc-950 shadow-sm ring-1 ring-black/10">
        <div className="absolute inset-0 bg-[linear-gradient(to_right,rgba(255,255,255,0.035)_1px,transparent_1px),linear-gradient(to_bottom,rgba(255,255,255,0.035)_1px,transparent_1px)] bg-[size:40px_40px]" />
        <video
          ref={videoRef}
          autoPlay
          muted
          playsInline
          onClick={selectCalibrationPoint}
          className={cn(
            "relative z-10 size-full object-contain transition-opacity duration-300",
            videoFit === "cover" && "object-cover",
            selecting && "cursor-crosshair",
            streamState === "live" ? "opacity-100" : "opacity-0",
          )}
        />

        {streamState !== "live" && (
          <div className="absolute inset-0 z-20 grid place-items-center">
            <div className="flex flex-col items-center gap-3 text-center">
              <div className="grid size-11 place-items-center rounded-full border border-white/15 bg-white/5 text-white">
                {streamState === "connecting" ? (
                  <Radio className="size-4 animate-pulse" />
                ) : (
                  <WifiOff className="size-4" />
                )}
              </div>
              <div>
                <p className="text-sm font-medium text-white">
                  {streamState === "connecting" ? "Connecting video" : "Stream unavailable"}
                </p>
                <p className="mt-1 font-mono text-[10px] uppercase tracking-[0.14em] text-zinc-500">
                  WebRTC · automatic retry
                </p>
              </div>
            </div>
          </div>
        )}

        <div className="absolute top-3 left-3 z-30 flex items-center gap-2">
          <Badge className="border-white/15 bg-black/45 text-white backdrop-blur-md">
            <ConnectionMark state={streamState} />
            {streamState}
          </Badge>
          <Badge className="hidden border-white/15 bg-black/45 text-zinc-300 backdrop-blur-md sm:flex">
            {telemetry.frameMode === "calibrated" ? "calibrated · 1000×300" : "calibration · TL/BL/TR/BR"}
          </Badge>
        </div>

        <div className="absolute right-3 bottom-3 left-3 z-30 flex items-end justify-between gap-3">
          <div className="min-w-0 rounded-lg border border-white/10 bg-black/55 px-3 py-2 text-white backdrop-blur-md">
            <p className="font-mono text-[9px] uppercase tracking-[0.14em] text-zinc-400">Controller state</p>
            <p className="mt-0.5 truncate text-sm font-medium">{controllerState.replaceAll("_", " ")}</p>
          </div>
          <Button
            variant="outline"
            size="icon"
            className="border-white/15 bg-black/45 text-white hover:bg-white hover:text-black"
            title={videoFit === "contain" ? "Fill video panel" : "Fit entire video"}
            onClick={() => setVideoFit((current) => current === "contain" ? "cover" : "contain")}
          >
            <Maximize2 />
          </Button>
        </div>
      </div>

      <div className="grid shrink-0 grid-cols-2 gap-x-4 gap-y-3 rounded-xl border border-zinc-200 bg-white p-3.5 sm:grid-cols-4">
        <Metric icon={Camera} label="Camera" value={streamState === "live" ? telemetry.camera : streamState} />
        <Metric
          icon={Waypoints}
          label={telemetry.frameMode === "calibration" ? "Corners" : "Waypoint"}
          value={telemetry.frameMode === "calibration" ? `${telemetry.calibrationPoints} / 4 selected` : waypoint}
        />
        <Metric icon={Cpu} label="MCU" value={gatewayState === "connected" ? telemetry.mcu : "Offline"} />
        <Metric icon={RotateCcw} label="Action" value={`${telemetry.direction} · step ${telemetry.step}`} />
      </div>
    </section>
  )
}

function CalibrationControls(session: ReturnType<typeof useConveyorSession>) {
  const selecting = session.controllerState === "CALIBRATION_SELECTING"
  const points = session.telemetry.calibrationPoints
  const labels = ["TL", "BL", "TR", "BR"]
  const nextPoint = points < 4 ? `${points + 1} · ${labels[points]}` : "Ready to confirm"

  return (
    <div className="shrink-0 border-t border-zinc-200 p-3">
      <div className="rounded-xl border border-zinc-200 bg-zinc-50 p-3">
        <div className="flex items-start justify-between gap-3">
          <div>
            <p className="font-mono text-[9px] uppercase tracking-[0.14em] text-zinc-400">Camera calibration</p>
            <p className="mt-1 text-sm font-medium text-zinc-950">
              {selecting ? nextPoint : "Capture the full conveyor"}
            </p>
          </div>
          <Badge>{points} / 4</Badge>
        </div>
        <p className="mt-2 text-xs leading-5 text-zinc-500">
          {selecting
            ? "Click physical corners on the video in order: top-left, bottom-left, top-right, bottom-right."
            : "Keep the complete belt visible, then freeze one frame before selecting corners."}
        </p>

        {!selecting ? (
          <Button
            className="mt-3 w-full"
            disabled={session.gatewayState !== "connected" || session.controllerState === "ERROR"}
            onClick={session.captureCalibration}
          >
            <Camera />
            Capture frame
          </Button>
        ) : (
          <div className="mt-3 grid grid-cols-2 gap-2">
            <Button variant="outline" size="sm" onClick={session.undoCalibrationPoint} disabled={points === 0}>
              Undo
            </Button>
            <Button variant="outline" size="sm" onClick={session.resetCalibrationPoints} disabled={points === 0}>
              Reset
            </Button>
            <Button variant="outline" size="sm" onClick={session.resumeCalibrationLive}>
              <RotateCcw />
              Live
            </Button>
            <Button size="sm" onClick={session.confirmCalibration} disabled={!session.telemetry.calibrationValid}>
              Confirm
            </Button>
          </div>
        )}
      </div>
      <p className="mt-2 text-center text-[10px] text-zinc-400">
        Camera ownership remains in the Python controller service.
      </p>
    </div>
  )
}

function Message({
  item,
  labelOverride,
}: {
  item: ConversationMessage
  labelOverride?: string
}) {
  const user = item.severity === "USER"
  const label = labelOverride ?? (user ? "You" : item.severity === "INFO" ? "VLA" : item.severity)
  const time = new Intl.DateTimeFormat(undefined, {
    hour: "2-digit",
    minute: "2-digit",
  }).format(item.timestamp)

  return (
    <article className={cn("group flex flex-col gap-1.5", user && "items-end")}>
      <div className="flex items-center gap-2 px-1 font-mono text-[9px] uppercase tracking-[0.14em] text-zinc-400">
        <span>{label}</span>
        <span>{time}</span>
      </div>
      <div
        className={cn(
          "max-w-[94%] text-sm leading-6",
          user
            ? "rounded-xl rounded-tr-sm bg-zinc-950 px-3.5 py-2.5 text-white"
            : "border-l border-zinc-200 py-1 pl-3.5 text-zinc-700",
          item.severity === "ERROR" && !user && "font-mono text-xs text-zinc-950",
        )}
      >
        {item.message}
      </div>
    </article>
  )
}

type RunTimelineEntry = {
  type: "run"
  id: string
  user?: ConversationMessage
  thinking: ConversationMessage[]
  final?: ConversationMessage
}

type TimelineEntry =
  | { type: "message"; id: string; message: ConversationMessage }
  | RunTimelineEntry

function buildTimeline(messages: ConversationMessage[]): TimelineEntry[] {
  const timeline: TimelineEntry[] = []
  const runs = new Map<string, RunTimelineEntry>()

  for (const message of messages) {
    if (!message.runId) {
      timeline.push({ type: "message", id: message.id, message })
      continue
    }

    let run = runs.get(message.runId)
    if (!run) {
      run = { type: "run", id: message.runId, thinking: [] }
      runs.set(message.runId, run)
      timeline.push(run)
    }

    if (message.presentation === "user" || message.severity === "USER") {
      run.user = message
    } else if (message.presentation === "final") {
      run.final = message
    } else {
      run.thinking.push(message)
    }
  }

  return timeline
}

function ProcessIcon({ severity }: { severity: ConversationMessage["severity"] }) {
  if (severity === "ERROR") return <CircleX className="size-3.5" />
  if (severity === "WARNING") return <CircleAlert className="size-3.5" />
  if (severity === "SUCCESS") return <CircleCheck className="size-3.5" />
  return <Info className="size-3.5" />
}

function ThinkingProcess({
  items,
  complete,
}: {
  items: ConversationMessage[]
  complete: boolean
}) {
  const [expanded, setExpanded] = React.useState(!complete)
  const wasComplete = React.useRef(complete)

  React.useEffect(() => {
    if (complete && !wasComplete.current) setExpanded(false)
    wasComplete.current = complete
  }, [complete])

  const latest = items.at(-1)?.message ?? "Preparing the controller…"

  return (
    <section className="overflow-hidden rounded-xl border border-zinc-200 bg-zinc-50/70">
      <button
        type="button"
        className="flex w-full items-center gap-2.5 px-3 py-2.5 text-left transition-colors hover:bg-zinc-100"
        onClick={() => setExpanded((current) => !current)}
        aria-expanded={expanded}
      >
        {complete ? (
          <CircleCheck className="size-4 shrink-0 text-zinc-500" />
        ) : (
          <LoaderCircle className="size-4 shrink-0 animate-spin text-zinc-700" />
        )}
        <div className="min-w-0 flex-1">
          <div className="flex items-center gap-2">
            <span
              className={cn(
                "text-xs font-medium",
                !complete && "thinking-shimmer",
              )}
            >
              {complete ? "Process" : "Thinking"}
            </span>
            <span className="font-mono text-[9px] uppercase tracking-[0.12em] text-zinc-400">
              {items.length} {items.length === 1 ? "step" : "steps"}
            </span>
          </div>
          {!expanded && (
            <p className="mt-0.5 truncate text-[11px] text-zinc-500">{latest}</p>
          )}
        </div>
        <ChevronDown
          className={cn(
            "size-4 shrink-0 text-zinc-400 transition-transform duration-200",
            expanded && "rotate-180",
          )}
        />
      </button>

      {expanded && (
        <div className="border-t border-zinc-200 px-3 py-3">
          {items.length === 0 ? (
            <p className="border-l border-zinc-300 pl-4 text-xs text-zinc-500">
              Waiting for the first controller event…
            </p>
          ) : (
            <ol className="space-y-3 border-l border-zinc-300 pl-4">
              {items.map((item) => (
                <li key={item.id} className="relative">
                  <span className="absolute -left-[22px] top-0.5 grid size-3.5 place-items-center bg-zinc-50 text-zinc-500">
                    <ProcessIcon severity={item.severity} />
                  </span>
                  <div className="flex items-center gap-2 font-mono text-[9px] uppercase tracking-[0.1em] text-zinc-400">
                    <span>{item.severity}</span>
                    <span>
                      {new Intl.DateTimeFormat(undefined, {
                        hour: "2-digit",
                        minute: "2-digit",
                        second: "2-digit",
                      }).format(item.timestamp)}
                    </span>
                  </div>
                  <p
                    className={cn(
                      "mt-1 break-words text-xs leading-5 text-zinc-600",
                      item.severity === "ERROR" && "font-mono text-zinc-950",
                      item.severity === "WARNING" && "text-zinc-800",
                    )}
                  >
                    {item.message}
                  </p>
                </li>
              ))}
            </ol>
          )}
        </div>
      )}
    </section>
  )
}

function RunConversation({ run }: { run: RunTimelineEntry }) {
  return (
    <div className="flex flex-col gap-3">
      {run.user && <Message item={run.user} />}
      <ThinkingProcess items={run.thinking} complete={Boolean(run.final)} />
      {run.final && <Message item={run.final} labelOverride="VLA" />}
    </div>
  )
}

function ConversationPanel(session: ReturnType<typeof useConveyorSession>) {
  const [instruction, setInstruction] = React.useState("")
  const scrollRef = React.useRef<HTMLDivElement>(null)
  const canSend = session.promptReady && instruction.trim().length > 0
  const timeline = React.useMemo(
    () => buildTimeline(session.messages),
    [session.messages],
  )

  React.useEffect(() => {
    const viewport = scrollRef.current
    if (viewport) viewport.scrollTop = viewport.scrollHeight
  }, [session.messages])

  const submit = () => {
    const value = instruction.trim()
    if (!value || !session.sendPrompt(value)) return
    setInstruction("")
  }

  return (
    <aside className="flex min-h-0 flex-col bg-white">
      <div className="flex h-[61px] shrink-0 items-center justify-between px-4">
        <div>
          <h2 className="text-sm font-semibold tracking-tight">VLA conversation</h2>
          <p className="mt-0.5 text-xs text-zinc-500">One instruction at a time</p>
        </div>
        <Badge>
          <ConnectionMark state={session.gatewayState} />
          {session.gatewayState}
        </Badge>
      </div>
      <Separator />

      <div ref={scrollRef} className="minimal-scrollbar min-h-0 flex-1 overflow-y-auto px-4 py-5">
        <div className="flex flex-col gap-5">
          {timeline.map((entry) =>
            entry.type === "run" ? (
              <RunConversation key={entry.id} run={entry} />
            ) : (
              <Message key={entry.id} item={entry.message} />
            ),
          )}
        </div>
      </div>

      {session.controllerState.startsWith("CALIBRATION_") ? (
        <CalibrationControls {...session} />
      ) : (
      <div className="shrink-0 border-t border-zinc-200 p-3">
        <div className="overflow-hidden rounded-xl border border-zinc-300 bg-white shadow-[0_1px_2px_rgba(0,0,0,0.04)] focus-within:border-zinc-950 focus-within:ring-1 focus-within:ring-zinc-950">
          <Textarea
            value={instruction}
            onChange={(event) => setInstruction(event.target.value)}
            onKeyDown={(event) => {
              if (event.key === "Enter" && (event.ctrlKey || event.metaKey)) {
                event.preventDefault()
                submit()
              }
            }}
            disabled={!session.promptReady}
            placeholder={
              session.gatewayState !== "connected"
                ? "Waiting for gateway…"
                : session.promptReady
                  ? "Describe the object and its destination(s)…"
                  : `Controller is ${session.controllerState.toLowerCase().replaceAll("_", " ")}…`
            }
          />
          <div className="flex items-center justify-between px-2.5 pb-2.5">
            <span className="px-1 font-mono text-[9px] uppercase tracking-[0.12em] text-zinc-400">
              Ctrl ↵ to send
            </span>
            <Button size="icon" className="size-8 rounded-lg" disabled={!canSend} onClick={submit}>
              <ArrowUp />
              <span className="sr-only">Send instruction</span>
            </Button>
          </div>
        </div>
        <div className="mt-2 flex items-center justify-center gap-1.5 text-center text-[10px] text-zinc-400">
          {session.gatewayState === "connected" ? <Wifi className="size-3" /> : <WifiOff className="size-3" />}
          Commands are acknowledged by the local controller gateway.
        </div>
      </div>
      )}
    </aside>
  )
}

export default function App() {
  const session = useConveyorSession()

  return (
    <div className="flex h-svh min-h-[640px] flex-col overflow-hidden bg-zinc-50 text-zinc-950">
      <Header gatewayState={session.gatewayState} onStop={session.stop} />
      <main className="grid min-h-0 flex-1 grid-rows-[minmax(420px,3fr)_minmax(420px,2fr)] lg:grid-cols-[minmax(0,2fr)_minmax(360px,1fr)] lg:grid-rows-1">
        <StreamPanel {...session} />
        <ConversationPanel {...session} />
      </main>
    </div>
  )
}
