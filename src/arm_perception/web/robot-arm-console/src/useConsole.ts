import { useCallback, useEffect, useRef, useState } from 'react'
import { getBootstrap, getRecordings, liveSocketUrl } from './api'
import type { Bootstrap, RecordingSession, Snapshot } from './types'

export function useConsole() {
  const [bootstrap, setBootstrap] = useState<Bootstrap | null>(null)
  const [snapshot, setSnapshot] = useState<Snapshot | null>(null)
  const [recordings, setRecordings] = useState<RecordingSession[]>([])
  const [connected, setConnected] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const retryRef = useRef(1000)

  const refreshRecordings = useCallback(async () => {
    try {
      setRecordings(await getRecordings())
    } catch (reason) {
      setError(reason instanceof Error ? reason.message : 'Kayıtlar okunamadı')
    }
  }, [])

  useEffect(() => {
    getBootstrap().then(setBootstrap).catch((reason) => {
      setError(reason instanceof Error ? reason.message : 'Gateway bilgisi alınamadı')
    })
    void refreshRecordings()
  }, [refreshRecordings])

  useEffect(() => {
    let active = true
    let socket: WebSocket | null = null
    let retryTimer: number | undefined

    const connect = () => {
      if (!active) return
      socket = new WebSocket(liveSocketUrl())
      socket.onopen = () => {
        retryRef.current = 1000
        setConnected(true)
        setError(null)
      }
      socket.onmessage = (event) => {
        try {
          setSnapshot(JSON.parse(event.data) as Snapshot)
        } catch {
          setError('Gateway geçersiz telemetri gönderdi')
        }
      }
      socket.onerror = () => socket?.close()
      socket.onclose = () => {
        setConnected(false)
        if (!active) return
        retryTimer = window.setTimeout(connect, retryRef.current)
        retryRef.current = Math.min(retryRef.current * 1.8, 10_000)
      }
    }
    connect()
    return () => {
      active = false
      if (retryTimer !== undefined) window.clearTimeout(retryTimer)
      socket?.close()
    }
  }, [])

  return {
    bootstrap,
    snapshot,
    recordings,
    connected,
    error,
    clearError: () => setError(null),
    refreshRecordings,
  }
}
