import type { Bootstrap, RecordingSession } from './types'

async function request<T>(path: string, options?: RequestInit): Promise<T> {
  const response = await fetch(path, {
    ...options,
    headers: { 'Content-Type': 'application/json', ...(options?.headers ?? {}) },
  })
  const body = await response.json()
  if (!response.ok) {
    throw new Error(body.error || `İstek başarısız (${response.status})`)
  }
  return body as T
}

export const getBootstrap = () => request<Bootstrap>('/api/v1/bootstrap')
export const getRecordings = () => request<RecordingSession[]>('/api/v1/recordings')

export const startRecording = (name: string, note: string) =>
  request('/api/v1/recordings/start', {
    method: 'POST',
    body: JSON.stringify({ name, note }),
  })

export const stopRecording = () =>
  request('/api/v1/recordings/stop', { method: 'POST', body: '{}' })

export const startReplay = (id: string) =>
  request('/api/v1/replay/start', {
    method: 'POST',
    body: JSON.stringify({ id, rate: 1.0 }),
  })

export const stopReplay = () =>
  request('/api/v1/replay/stop', { method: 'POST', body: '{}' })

export function liveSocketUrl(): string {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  return `${protocol}//${window.location.host}/api/v1/live`
}
