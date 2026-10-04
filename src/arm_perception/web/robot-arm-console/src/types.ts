export type HealthStatus = 'healthy' | 'stale' | 'offline'

export interface ChannelHealth {
  status: HealthStatus
  ageSeconds: number | null
  fps?: number
  source?: 'compressed' | 'raw_jpeg_fallback' | null
}

export interface Health {
  mode: 'live' | 'replay'
  camera: ChannelHealth
  cameraInfo: ChannelHealth
  joints: ChannelHealth
  perception: ChannelHealth
  objects3d: ChannelHealth
  armStatus: ChannelHealth
  robotDescription: ChannelHealth
  system: ChannelHealth
  readOnly: true
}

export interface JointState {
  stampNs: number
  names: string[]
  positions: number[]
  sourceKind: 'commanded_open_loop'
}

export interface Position {
  x: number
  y: number
  z: number
}

export interface CameraInfo {
  stampNs: number
  frameId: string
  width: number
  height: number
  fx?: number
  fy?: number
  cx?: number
  cy?: number
  distortion?: number[]
}

/**
 * Yayindaki intrinsics'in HANGI kalibrasyon dosyasindan geldigi ve o
 * kalibrasyonun kendi belirsizligi. `matched` false ise havada olan sayilar
 * repodaki hicbir dosyayla uyusmuyor demektir -- bu sessizce gecilmemeli.
 */
export interface Calibration {
  matched: boolean
  file: string | null
  sha256: string | null
  reason: string
  candidatesChecked: number
  fitRmsPx?: number | null
  validationMeanPx?: number | null
  validationMaxPx?: number | null
  fitFrames?: number | null
  validationFrames?: number | null
  principalPointPolicy?: string | null
  flipMethod?: number | null
  squareSizeMm?: number | null
  calibratedAt?: string | null
  poseSpreadMedianDeg?: number | null
  perceptionFloorDeg?: number | null
  uncertaintyRatio?: number | null
  uncertaintyGateMax?: number | null
  uncertaintyMeasuredAt?: string | null
  uncertaintyVerdict?: 'pass' | 'fail' | 'unmeasured'
}

export interface FlipContract {
  status: 'match' | 'mismatch' | 'unknown'
  expected: number | null
  actual: number | null
}

export interface SerialPort {
  device: string
  held: boolean
  owners: { pid: number; process: string }[]
}

export interface ThermalTrend {
  windowSeconds: number
  samples: number[]
  minC: number | null
  maxC: number | null
  slopeCPerMin: number | null
}

export interface MeasurementEntry {
  id: string
  label: string
  value: string
  measuredAt: string | null
  ageDays: number | null
  source: string
  note: string
}

export interface MeasurementRegistry {
  available: boolean
  entries: MeasurementEntry[]
}

export interface SystemTelemetry {
  hostname: string
  platform: string
  cpuCount: number
  cpuTemperatureC: number | null
  loadAverage: [number, number, number]
  memoryUsedPercent: number | null
  diskFreeBytes: number | null
  uptimeSeconds: number | null
  throttle: {
    raw: string | null
    active: boolean | null
    historical: boolean | null
  }
  repoCommit: string
}

export interface DetectedObject {
  id: string
  className: string
  confidence: number
  frameId: string
  stampNs: number
  position: Position
  orientation: { x: number; y: number; z: number; w: number }
  dimensions: number[]
}

export interface EventItem {
  id: string
  stamp: string
  level: 'info' | 'warning' | 'error' | 'recording'
  title: string
  detail: string
}

export interface RecordingState {
  recording: string | null
  replay: string | null
  freeBytes: number
  minimumFreeBytes: number
}

export interface RecordingSession {
  id: string
  name: string
  note: string
  status: string
  startedAt: string
  endedAt: string | null
  replayAvailable: boolean
}

export interface Snapshot {
  type: 'snapshot'
  sequence: number
  mode: 'live' | 'replay'
  jointState: JointState
  objects: DetectedObject[]
  armStatus: {
    stampNs: number
    state: number
    phase: string
    progress: number
    objectGrasped: boolean
    error: string
  } | null
  cameraInfo: CameraInfo | null
  system: SystemTelemetry | null
  health: Health
  events: EventItem[]
  recording: RecordingState
  calibration: Calibration | null
  flipContract: FlipContract
  serialPorts: SerialPort[]
  thermal: ThermalTrend
  measurements: MeasurementRegistry
}

export interface Bootstrap {
  apiVersion: 'v1'
  readOnly: true
  jointStateSemantics: 'commanded_open_loop'
  jointStateNotice: string
  robotDescription: string
  packageMap: Record<string, string>
  capabilities: {
    cameraOverlay: boolean
    recording: boolean
    replay: boolean
    motionCommands: false
  }
}
