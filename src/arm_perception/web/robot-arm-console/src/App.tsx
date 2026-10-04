import { lazy, Suspense, useMemo, useState } from 'react'
import {
  Activity, AlertTriangle, Box, Camera, CircleStop, Database,
  Cpu, Gauge, HardDrive, MemoryStick, Play, Radio, RefreshCw,
  ScanLine, Square, Thermometer, Video, WifiOff,
  Crosshair, FlipHorizontal, History, Plug, Ruler, TrendingUp,
} from 'lucide-react'
import {
  startRecording, startReplay, stopRecording, stopReplay,
} from './api'
import { useConsole } from './useConsole'
import type {
  Calibration, ChannelHealth, FlipContract, HealthStatus,
  MeasurementRegistry, SerialPort, ThermalTrend,
} from './types'

const RobotViewer = lazy(() => import('./RobotViewer').then((module) => ({
  default: module.RobotViewer,
})))

const statusText: Record<HealthStatus, string> = {
  healthy: 'Sağlıklı',
  stale: 'Veri eski',
  offline: 'Çevrimdışı',
}

function ageText(age: number | null): string {
  if (age === null) return 'veri yok'
  if (age < 1) return `${Math.round(age * 1000)} ms önce`
  return `${age.toFixed(1)} sn önce`
}

function angleText(radians: number): string {
  return `${radians.toFixed(3)} rad / ${(radians * 180 / Math.PI).toFixed(1)}°`
}

function durationText(seconds: number | null | undefined): string {
  if (seconds === null || seconds === undefined) return 'ölçülmedi'
  const hours = Math.floor(seconds / 3600)
  const minutes = Math.floor((seconds % 3600) / 60)
  return hours > 0 ? `${hours} sa ${minutes} dk` : `${minutes} dk`
}

function cameraSourceText(source?: string | null): string {
  if (source === 'compressed') return 'ROS compressed'
  if (source === 'raw_jpeg_fallback') return 'Ham JPEG fallback'
  return 'Kaynak bekleniyor'
}

function StatusCard({ icon, label, channel }: {
  icon: React.ReactNode
  label: string
  channel?: ChannelHealth
}) {
  const state = channel?.status ?? 'offline'
  return (
    <div className="status-card">
      <div className={`status-icon status-${state}`}>{icon}</div>
      <div>
        <span className="eyebrow">{label}</span>
        <strong>{statusText[state]}</strong>
        <small>{ageText(channel?.ageSeconds ?? null)}</small>
      </div>
    </div>
  )
}

/**
 * Belirsizligi TABANA GORE gosteren olcek.
 *
 * Cubuk 0-3x araligini kapsar ve kapi (1.0x) SABIT olarak ucte birde durur.
 * Dolgu oranla birlikte uzar; kapiyi gecince uyari rengine doner. Amac, sayiyi
 * okumadan once "cizginin solunda miyiz sagninda miyiz" sorusunun
 * cevaplanabilmesi.
 *
 * Not: onceki surumde dolgu ve tasma AYRI flex parcalariydi ve toplamlari
 * %100'u asinca ikisi birlikte kuculuyordu -- sayi dogru, resim yanlisti.
 */
const RATIO_SCALE = 3

function RatioBar({ ratio, gate }: { ratio: number; gate: number }) {
  const passed = ratio <= gate
  const filled = Math.min(ratio / RATIO_SCALE, 1)
  const gateAt = Math.min(gate / RATIO_SCALE, 1)
  return (
    <div className={`ratio-bar ${passed ? 'ratio-pass' : 'ratio-fail'}`} role="img"
      aria-label={`Taban oranı ${ratio.toFixed(2)}, kapı ${gate.toFixed(2)}`}>
      <span className="ratio-fill" style={{ width: `${filled * 100}%` }} />
      <span className="ratio-gate" style={{ left: `${gateAt * 100}%` }} />
      {ratio > RATIO_SCALE && <span className="ratio-clip">›</span>}
    </div>
  )
}

/** Sicaklik EGILIMI -- anlik deger tirmanisi gostermez. */
function Sparkline({ values, width = 132, height = 34 }: {
  values: number[]
  width?: number
  height?: number
}) {
  if (values.length < 2) {
    return <span className="sparkline-empty">eğilim için veri toplanıyor</span>
  }
  const min = Math.min(...values)
  const max = Math.max(...values)
  const span = max - min < 1 ? 1 : max - min
  const step = width / (values.length - 1)
  const points = values
    .map((value, index) => {
      const x = index * step
      const y = height - ((value - min) / span) * (height - 4) - 2
      return `${x.toFixed(1)},${y.toFixed(1)}`
    })
    .join(' ')
  return (
    <svg className="sparkline" viewBox={`0 0 ${width} ${height}`}
      width={width} height={height} role="img"
      aria-label={`Sıcaklık eğilimi ${min.toFixed(1)} ile ${max.toFixed(1)} derece arası`}>
      <polyline points={points} fill="none" strokeWidth={1.6}
        strokeLinejoin="round" strokeLinecap="round" />
    </svg>
  )
}

function CalibrationCard({ calibration }: { calibration?: Calibration | null }) {
  if (!calibration) {
    return (
      <article className="confidence-card">
        <span className="eyebrow"><Crosshair size={13} /> Kalibrasyon</span>
        <strong className="confidence-headline muted">CameraInfo bekleniyor</strong>
      </article>
    )
  }
  if (!calibration.matched) {
    return (
      <article className="confidence-card confidence-alarm">
        <span className="eyebrow"><Crosshair size={13} /> Kalibrasyon</span>
        <strong className="confidence-headline">TANINMADI</strong>
        <p className="confidence-note">{calibration.reason}</p>
      </article>
    )
  }
  const ratio = calibration.uncertaintyRatio
  const gate = calibration.uncertaintyGateMax ?? 1
  const verdict = calibration.uncertaintyVerdict ?? 'unmeasured'
  return (
    <article className={`confidence-card confidence-${verdict}`}>
      <span className="eyebrow"><Crosshair size={13} /> Kalibrasyon belirsizliği</span>
      {ratio == null ? (
        <>
          <strong className="confidence-headline muted">ÖLÇÜLMEDİ</strong>
          <p className="confidence-note">
            Bu kalibrasyonun kendi belirsizliği hiç ölçülmemiş. Reprojeksiyon
            RMS'i bu sayıyı göremez.
          </p>
        </>
      ) : (
        <>
          <strong className="confidence-headline">
            {ratio.toFixed(1)}<span className="confidence-unit">× taban</span>
          </strong>
          <RatioBar ratio={ratio} gate={gate} />
          <p className="confidence-note">
            Kapı {gate.toFixed(1)}× · taban {calibration.perceptionFloorDeg?.toFixed(3)}° ·
            yayılım {calibration.poseSpreadMedianDeg?.toFixed(3)}°
          </p>
        </>
      )}
      <dl className="confidence-facts">
        <div><dt>Dosya</dt><dd title={calibration.file ?? ''}>{calibration.file}</dd></div>
        <div><dt>SHA</dt><dd className="mono">{calibration.sha256}</dd></div>
        {/* Alan yoksa 'serbest' DEME -- bilinmeyeni bilinen gibi sunmak bu
              panelin onlemek icin var oldugu hatanin ta kendisi. */}
        <div><dt>Asal nokta</dt><dd>{calibration.principalPointPolicy == null
          ? 'belirtilmemiş'
          : calibration.principalPointPolicy === 'fixed_image_center'
            ? 'merkeze çakılı'
            : calibration.principalPointPolicy}</dd></div>
        <div><dt>Reproj (doğrulama)</dt><dd>{calibration.validationMeanPx == null ? '—' : `${calibration.validationMeanPx.toFixed(3)} px`}</dd></div>
      </dl>
    </article>
  )
}

function FlipCard({ contract }: { contract?: FlipContract }) {
  const status = contract?.status ?? 'unknown'
  const label = status === 'match' ? 'UYUMLU'
    : status === 'mismatch' ? 'UYUMSUZ' : 'BİLİNMİYOR'
  const tone = status === 'match' ? 'pass'
    : status === 'mismatch' ? 'alarm' : 'unmeasured'
  return (
    <article className={`confidence-card confidence-${tone}`}>
      <span className="eyebrow"><FlipHorizontal size={13} /> Flip sözleşmesi</span>
      <strong className="confidence-headline">{label}</strong>
      <p className="confidence-note">
        {status === 'mismatch'
          ? 'Kalibrasyonun çekildiği flip ile node’un uyguladığı flip farklı — intrinsics sessizce geçersiz.'
          : status === 'unknown'
            ? 'Kamera node’unun parametresi okunamadı; bilinmeyen “uyumlu” sayılmaz.'
            : 'Kalibrasyon ve node aynı flip’i kullanıyor.'}
      </p>
      <dl className="confidence-facts">
        <div><dt>Kalibrasyon</dt><dd>{contract?.expected ?? '—'}</dd></div>
        <div><dt>Node</dt><dd>{contract?.actual ?? '—'}</dd></div>
      </dl>
    </article>
  )
}

function SerialCard({ ports }: { ports?: SerialPort[] }) {
  const list = ports ?? []
  const held = list.filter((port) => port.held)
  return (
    <article className={`confidence-card ${held.length ? 'confidence-busy' : ''}`}>
      <span className="eyebrow"><Plug size={13} /> Seri port</span>
      <strong className="confidence-headline">
        {list.length === 0 ? 'CİHAZ YOK' : held.length ? 'TUTULUYOR' : 'BOŞTA'}
      </strong>
      {list.length === 0 ? (
        <p className="confidence-note">
          Eşleşen /dev/ttyUSB* veya /dev/ttyACM* yok — ESP32 bağlı değil.
        </p>
      ) : (
        <ul className="serial-list">
          {list.map((port) => (
            <li key={port.device}>
              <span className="mono">{port.device}</span>
              <span className={port.held ? 'danger-text' : 'muted'}>
                {port.held
                  ? port.owners.map((owner) => `${owner.process} (${owner.pid})`).join(', ')
                  : 'sahipsiz görünüyor'}
              </span>
            </li>
          ))}
        </ul>
      )}
      <p className="confidence-footnote">
        Port açılmadı, /proc okundu. Başka kullanıcının tuttuğu port burada
        görünmez; boş liste “serbest” demek değildir.
      </p>
    </article>
  )
}

function MeasurementPanel({ registry }: { registry?: MeasurementRegistry }) {
  const entries = registry?.entries ?? []
  return (
    <article className="panel">
      <div className="panel-heading">
        <div>
          <span className="eyebrow">İddia ≠ ölçüm</span>
          <h2>Ölçüm tazeliği</h2>
        </div>
        <span className="count-badge">{entries.length}</span>
      </div>
      {entries.length === 0 ? (
        <p className="empty-state">Ölçüm kayıt defteri yüklenmedi.</p>
      ) : (
        <ul className="measurement-list">
          {entries.map((entry) => (
            <li key={entry.id} className={entry.ageDays != null && entry.ageDays > 30 ? 'measurement-stale' : ''}>
              <div className="measurement-main">
                <strong>{entry.label}</strong>
                <span className="measurement-value">{entry.value}</span>
              </div>
              <div className="measurement-meta">
                <span><History size={12} /> {entry.ageDays == null ? 'tarih yok' : entry.ageDays === 0 ? 'bugün' : `${entry.ageDays} gün önce`}</span>
                <span className="mono measurement-source" title={entry.source}>{entry.source}</span>
              </div>
            </li>
          ))}
        </ul>
      )}
    </article>
  )
}

export default function App() {
  const {
    bootstrap, snapshot, recordings, connected, error, clearError,
    refreshRecordings,
  } = useConsole()
  const [recordingLabel, setRecordingLabel] = useState('Robot Arm gözlem oturumu')
  const [sessionNote, setSessionNote] = useState('')
  const [busy, setBusy] = useState(false)
  const [actionError, setActionError] = useState<string | null>(null)

  const recording = snapshot?.recording.recording ?? null
  const replay = snapshot?.recording.replay ?? null
  const freeGb = (snapshot?.recording.freeBytes ?? 0) / 1_000_000_000
  const sortedObjects = useMemo(
    () => snapshot?.health.objects3d.status === 'healthy'
      ? [...snapshot.objects].sort((a, b) => b.confidence - a.confidence)
      : [],
    [snapshot?.health.objects3d.status, snapshot?.objects],
  )
  const jointRows = useMemo(() => {
    const state = snapshot?.jointState
    if (!state) return []
    return state.names.map((name, index) => ({
      name,
      position: state.positions[index],
    })).filter((row) => Number.isFinite(row.position))
  }, [snapshot?.jointState])
  const system = snapshot?.system
  const thermal = snapshot?.thermal
  const temperatureState = system?.cpuTemperatureC == null
    ? 'unknown'
    : system.cpuTemperatureC >= 80
      ? 'danger'
      : system.cpuTemperatureC >= 70 ? 'warning' : 'healthy'

  const run = async (operation: () => Promise<unknown>) => {
    setBusy(true)
    setActionError(null)
    try {
      await operation()
      await refreshRecordings()
    } catch (reason) {
      setActionError(reason instanceof Error ? reason.message : 'İşlem tamamlanamadı')
    } finally {
      setBusy(false)
    }
  }

  return (
    <main className="app-shell">
      <header className="topbar">
        <div className="brand">
          <div className="brand-mark" aria-hidden="true"><Activity size={22} /></div>
          <div>
            <span className="eyebrow">Robot operatör konsolu</span>
            <h1>Robot Arm</h1>
          </div>
        </div>
        <div className="topbar-actions">
          <span className={`connection-chip ${connected ? 'connected' : 'disconnected'}`}>
            {connected ? <Radio size={16} /> : <WifiOff size={16} />}
            {connected ? 'Gateway bağlı' : 'Bağlantı bekleniyor'}
          </span>
          <span className="readonly-chip">Salt okunur</span>
        </div>
      </header>

      {(error || actionError) && (
        <div className="error-banner" role="alert">
          <AlertTriangle size={18} />
          <span>{error || actionError}</span>
          <button type="button" onClick={() => { clearError(); setActionError(null) }}>Kapat</button>
        </div>
      )}

      <section className="status-grid" aria-label="Sistem sağlığı">
        <StatusCard icon={<Camera size={20} />} label="Kamera" channel={snapshot?.health.camera} />
        <StatusCard icon={<Box size={20} />} label="YOLO algılama" channel={snapshot?.health.perception} />
        <StatusCard icon={<Activity size={20} />} label="Eklem durumu" channel={snapshot?.health.joints} />
        <StatusCard icon={<Database size={20} />} label="3B nesneler" channel={snapshot?.health.objects3d} />
        <StatusCard icon={<ScanLine size={20} />} label="Kol durumu" channel={snapshot?.health.armStatus} />
        <StatusCard icon={<Cpu size={20} />} label="Pi 5 / Gateway" channel={snapshot?.health.system} />
      </section>

      {/*
        Kanal saglikli olabilir ve tasidigi sayi yine de guvenilmez olabilir.
        Bu bant "veri geliyor mu" ile "gelen veriye ne kadar guveniyoruz"
        sorularini ayirir; bu projede kabul metriginin onemli hatayi
        gormedigi dort vaka var.
      */}
      <section className="confidence-grid" aria-label="Ölçüm güveni">
        <CalibrationCard calibration={snapshot?.calibration} />
        <FlipCard contract={snapshot?.flipContract} />
        <SerialCard ports={snapshot?.serialPorts} />
      </section>

      <section className="workspace-grid">
        <article className="panel robot-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Dijital ikiz</span>
              <h2>Robot ve hedefler</h2>
            </div>
            <span className="warning-chip">Komut edilen poz</span>
          </div>
          <Suspense fallback={<div className="viewer-loading" role="status">3B görünüm yükleniyor</div>}>
            <RobotViewer
              bootstrap={bootstrap}
              joints={snapshot?.jointState}
              objects={sortedObjects}
            />
          </Suspense>
          <div className="truth-notice">
            <AlertTriangle size={16} />
            <span>{bootstrap?.jointStateNotice ?? 'Encoder geri bildirimi yok'}</span>
          </div>
        </article>

        <article className="panel camera-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">IMX219 · 640 × 480</span>
              <h2>Canlı kamera</h2>
            </div>
            <span className={`status-dot-label status-${snapshot?.health.camera.status ?? 'offline'}`}>
              <span className="dot" />
              {snapshot?.mode === 'replay' ? 'Kayıt tekrarı' : 'Canlı'}
            </span>
          </div>
          <div className="camera-frame">
            <img src="/api/v1/camera/stream" alt="Robot Arm kol kamerası canlı yayını" />
            <div className="camera-overlay-stats" aria-label="Kamera yayın bilgileri">
              <span>{(snapshot?.health.camera.fps ?? 0).toFixed(1)} FPS</span>
              <span>{cameraSourceText(snapshot?.health.camera.source)}</span>
            </div>
            {snapshot?.health.camera.status !== 'healthy' && (
              <div className="camera-unavailable" role="status">
                <Video size={26} />
                <strong>Kamera verisi güncel değil</strong>
                <span>{ageText(snapshot?.health.camera.ageSeconds ?? null)}</span>
              </div>
            )}
          </div>
          <div className="camera-footer">
            <span>Kutular yalnız görüntü zaman damgasıyla eşleştiğinde çizilir.</span>
            <span>{(snapshot?.health.camera.fps ?? 0).toFixed(1)} FPS · {sortedObjects.length} nesne</span>
          </div>
        </article>

        <aside className="panel object-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Algılama sonucu</span>
              <h2>Nesneler</h2>
            </div>
            <span className="count-badge">{sortedObjects.length}</span>
          </div>
          <div className="object-list">
            {sortedObjects.length === 0 ? (
              <div className="empty-state">
                <Box size={24} />
                <strong>Güncel nesne yok</strong>
                <span>Eski sonuçlar aktif olarak gösterilmez.</span>
              </div>
            ) : sortedObjects.map((object) => (
              <div className="object-row" key={object.id}>
                <div className={`object-swatch class-${object.className}`} aria-hidden="true" />
                <div className="object-main">
                  <strong>{object.className.replaceAll('_', ' ')}</strong>
                  <span>{object.id}</span>
                  <code>
                    x {object.position.x.toFixed(3)} · y {object.position.y.toFixed(3)} · z {object.position.z.toFixed(3)} m
                  </code>
                </div>
                <span className="confidence">{Math.round(object.confidence * 100)}%</span>
              </div>
            ))}
          </div>
        </aside>
      </section>

      <section className="telemetry-grid" aria-label="Operasyon telemetrisi">
        <article className="panel telemetry-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Açık çevrim telemetri</span>
              <h2>Eklem değerleri</h2>
            </div>
            <span className="warning-chip">Encoder değil</span>
          </div>
          <div className="joint-table" role="table" aria-label="Komut edilen eklem değerleri">
            {jointRows.length === 0 ? (
              <div className="compact-empty">/joint_states bekleniyor</div>
            ) : jointRows.map((joint) => (
              <div className="joint-row" role="row" key={joint.name}>
                <span role="cell">{joint.name.replaceAll('_', ' ')}</span>
                <code role="cell">{angleText(joint.position)}</code>
              </div>
            ))}
          </div>
          <div className="panel-note">
            Son güncelleme: {ageText(snapshot?.health.joints.ageSeconds ?? null)}
          </div>
        </article>

        <article className="panel telemetry-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Gateway host</span>
              <h2>Pi 5 sistem sağlığı</h2>
            </div>
            <span className={`metric-state metric-${temperatureState}`}>
              <Thermometer size={15} />
              {system?.cpuTemperatureC == null ? '—' : `${system.cpuTemperatureC.toFixed(1)}°C`}
            </span>
          </div>
          <dl className="metric-list">
            <div><dt><Cpu size={15} /> Host</dt><dd>{system?.hostname ?? 'bekleniyor'}</dd></div>
            <div><dt><Gauge size={15} /> Yük (1/5/15 dk)</dt><dd>{system?.loadAverage.map((value) => value.toFixed(2)).join(' / ') ?? '—'}</dd></div>
            <div><dt><MemoryStick size={15} /> RAM kullanımı</dt><dd>{system?.memoryUsedPercent == null ? '—' : `${system.memoryUsedPercent.toFixed(1)}%`}</dd></div>
            <div><dt><HardDrive size={15} /> Kayıt alanı</dt><dd>{system?.diskFreeBytes == null ? '—' : `${(system.diskFreeBytes / 1_000_000_000).toFixed(1)} GB boş`}</dd></div>
            <div><dt><Activity size={15} /> Çalışma süresi</dt><dd>{durationText(system?.uptimeSeconds)}</dd></div>
          </dl>
          <div className="thermal-trend">
            <div className="thermal-trend-head">
              <span className="eyebrow"><TrendingUp size={13} /> Sıcaklık eğilimi</span>
              <span className={`thermal-slope ${(thermal?.slopeCPerMin ?? 0) > 1 ? 'has-warning' : ''}`}>
                {thermal?.slopeCPerMin == null
                  ? '—'
                  : `${thermal.slopeCPerMin >= 0 ? '+' : ''}${thermal.slopeCPerMin.toFixed(1)} °C/dk`}
              </span>
            </div>
            <Sparkline values={thermal?.samples ?? []} />
            <span className="thermal-range">
              {thermal?.minC == null
                ? `son ${Math.round((thermal?.windowSeconds ?? 600) / 60)} dk`
                : `${thermal.minC.toFixed(1)}° – ${thermal.maxC?.toFixed(1)}° · son ${Math.round(thermal.windowSeconds / 60)} dk`}
            </span>
          </div>
          <div className={`throttle-banner ${system?.throttle.active || system?.throttle.historical ? 'has-warning' : ''}`}>
            <strong>Throttle:</strong>
            <span>{system?.throttle.raw ?? 'desteklenmiyor'}</span>
            {system?.throttle.active && <b>Şu an aktif</b>}
            {!system?.throttle.active && system?.throttle.historical && <b>Geçmiş kayıt var</b>}
          </div>
        </article>

        <article className="panel telemetry-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Kaynak ve görev</span>
              <h2>Kamera / kol bağlamı</h2>
            </div>
            <span className="readonly-chip">{snapshot?.mode === 'replay' ? 'Replay' : 'Live'}</span>
          </div>
          <dl className="metric-list">
            <div><dt>Kamera</dt><dd>{snapshot?.cameraInfo ? `${snapshot.cameraInfo.width} × ${snapshot.cameraInfo.height}` : 'CameraInfo bekleniyor'}</dd></div>
            <div><dt>Frame</dt><dd>{snapshot?.cameraInfo?.frameId || '—'}</dd></div>
            <div><dt>Taşıma</dt><dd>{cameraSourceText(snapshot?.health.camera.source)}</dd></div>
            <div><dt>Görev fazı</dt><dd>{snapshot?.armStatus?.phase || 'arm_status bekleniyor'}</dd></div>
            <div><dt>İlerleme</dt><dd>{snapshot?.armStatus ? `${Math.round(snapshot.armStatus.progress * 100)}%` : '—'}</dd></div>
            <div><dt>Son hata</dt><dd className={snapshot?.armStatus?.error ? 'danger-text' : ''}>{snapshot?.armStatus?.error || 'Yok / raporlanmadı'}</dd></div>
          </dl>
          <div className="panel-note">Gateway sıra no: {snapshot?.sequence ?? 0}</div>
        </article>
      </section>

      <section className="lower-grid lower-grid-wide">
        <article className="panel recording-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Senkron veri</span>
              <h2>Kayıt ve tekrar</h2>
            </div>
            <span className="disk-label">{freeGb.toFixed(1)} GB boş</span>
          </div>
          <div className="recording-controls">
            <label>
              <span>Oturum adı</span>
              <input
                value={recordingLabel}
                onChange={(event) => setRecordingLabel(event.target.value)}
                disabled={Boolean(recording) || busy}
              />
            </label>
            <label>
              <span>Operatör notu</span>
              <input
                value={sessionNote}
                onChange={(event) => setSessionNote(event.target.value)}
                placeholder="Bu oturumda ne doğrulanıyor?"
                disabled={Boolean(recording) || busy}
              />
            </label>
            {recording ? (
              <button className="button destructive" type="button" disabled={busy} onClick={() => void run(stopRecording)}>
                <CircleStop size={18} /> Kaydı durdur
              </button>
            ) : (
              <button
                className="button primary" type="button"
                disabled={busy || Boolean(replay) || recordingLabel.trim().length === 0}
                onClick={() => void run(() => startRecording(recordingLabel, sessionNote))}
              >
                <Square size={17} /> Kaydı başlat
              </button>
            )}
          </div>
          <div className="session-list">
            {recordings.slice(0, 6).map((session) => (
              <div className="session-row" key={session.id}>
                <div>
                  <strong>{session.name}</strong>
                  <span>{new Date(session.startedAt).toLocaleString('tr-TR')}</span>
                </div>
                <span className="session-status">{session.status}</span>
                <button
                  className="icon-button" type="button"
                  aria-label={`${session.name} kaydını oynat`}
                  title="Kaydı oynat"
                  disabled={busy || Boolean(recording) || Boolean(replay) || !session.replayAvailable}
                  onClick={() => void run(() => startReplay(session.id))}
                >
                  <Play size={18} />
                </button>
              </div>
            ))}
            {replay && (
              <button className="button secondary" type="button" disabled={busy} onClick={() => void run(stopReplay)}>
                <CircleStop size={18} /> Tekrarı durdur
              </button>
            )}
          </div>
        </article>

        <article className="panel events-panel">
          <div className="panel-heading">
            <div>
              <span className="eyebrow">Oturum geçmişi</span>
              <h2>Olaylar</h2>
            </div>
            <button className="icon-button" type="button" title="Kayıtları yenile" aria-label="Kayıt listesini yenile" onClick={() => void refreshRecordings()}>
              <RefreshCw size={17} />
            </button>
          </div>
          <div className="event-list">
            {(snapshot?.events ?? []).slice(0, 8).map((event) => (
              <div className={`event-row event-${event.level}`} key={event.id}>
                <span className="event-marker" />
                <div>
                  <strong>{event.title}</strong>
                  <span>{event.detail}</span>
                </div>
                <time>{new Date(event.stamp).toLocaleTimeString('tr-TR')}</time>
              </div>
            ))}
          </div>
        </article>
        <MeasurementPanel registry={snapshot?.measurements} />

      </section>
    </main>
  )
}
