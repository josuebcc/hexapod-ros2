import { resetManualMotion } from './motion.js'
import { state } from './state.js'
import { el, log } from '../shared/dom.js'

let switching = false
let polling = false

function applyMode(payload) {
  const mode = payload.available ? payload.mode : null
  if (state.motion.controlMode !== mode) {
    state.motion.controlMode = null
    resetManualMotion()
    state.motion.controlMode = mode
  }
  const manual = mode === 'manual' && !switching
  el('manual-controls').inert = !manual
  el('manual-controls').classList.toggle('is-disabled', !manual)
  el('motion-elastic-btn').disabled = !manual
  if (!switching) el('autonomous-mode').checked = mode === 'autonomous'
  el('autonomous-mode').disabled = !payload.available || switching
  el('control-mode-status').textContent = switching
    ? 'Cambiando...'
    : mode === 'autonomous' ? 'Autonomo' : mode === 'manual' ? 'Manual' : 'Sin conexion'
}

async function pollMode() {
  if (switching || polling) return
  polling = true
  try {
    const response = await fetch('/control_mode', { signal: AbortSignal.timeout(2500) })
    if (!response.ok) throw new Error(`HTTP ${response.status}`)
    const payload = await response.json()
    if (!switching) applyMode(payload)
  } catch {
    if (!switching) applyMode({ available: false })
  } finally {
    polling = false
  }
}

export function initControlMode() {
  el('autonomous-mode').addEventListener('change', async (event) => {
    const mode = event.target.checked ? 'autonomous' : 'manual'
    switching = true
    applyMode({ available: false })
    try {
      const response = await fetch('/control_mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode }),
        signal: AbortSignal.timeout(3000),
      })
      const payload = await response.json()
      if (!response.ok || !payload.ok) throw new Error(payload.error || 'Cambio rechazado')
      switching = false
      applyMode(payload)
    } catch (error) {
      log(`Modo de control: ${error.message}`, false)
      switching = false
      applyMode({ available: false })
    }
    await pollMode()
  })
  applyMode({ available: false })
  pollMode()
  setInterval(pollMode, 500)
}
