import { getJson, sendRobotCommand } from './api.js'
import { setPostureInputs } from './config.js'
import { POSTURE_SUFFIXES, state } from './state.js'
import { el, formatPostureValue, hasAllValues } from '../shared/dom.js'

function renderPostureReadout() {
  Object.entries(POSTURE_SUFFIXES).forEach(([axis, suffix]) => {
    el(`sv-${axis}`).textContent = formatPostureValue(
      state.posture.target[axis],
      suffix,
      true,
    )
    el(`rv-${axis}`).textContent = formatPostureValue(
      state.posture.current[axis],
      suffix,
    )
  })
}

function readPostureInputs() {
  return {
    roll: parseFloat(el('sl-roll').value),
    pitch: parseFloat(el('sl-pitch').value),
    yaw: parseFloat(el('sl-yaw').value),
    x: parseFloat(el('sl-x').value),
    y: parseFloat(el('sl-y').value),
    z: parseFloat(el('sl-z').value),
  }
}

function hydratePosture() {
  if (state.posture.hydrated || state.posture.dirty ||
      !hasAllValues(state.posture.current)) {
    return
  }

  setPostureInputs(state.posture.current)
  state.posture.target = readPostureInputs()
  state.posture.hydrated = true
}

function updatePosture() {
  const toRad = (degrees) => ((degrees * Math.PI) / 180).toFixed(4)
  const posture = readPostureInputs()
  state.posture.dirty = true
  state.posture.hydrated = true
  state.posture.target = posture
  renderPostureReadout()
  sendRobotCommand(
    `POS,${toRad(posture.roll)},${toRad(posture.pitch)},` +
    `${toRad(posture.yaw)},${posture.x},${posture.y},${posture.z}`,
  )
}

function resetPosture() {
  ;Object.keys(POSTURE_SUFFIXES).forEach((axis) => {
    el(`sl-${axis}`).value = 0
  })
  updatePosture()
}

function updateSmoothing() {
  const value = parseFloat(el('sl-smo').value)
  state.posture.smoothingTime = value
  state.posture.smoothingDirty = true
  el('sv-smo').textContent = `${value.toFixed(1)} s`
  sendRobotCommand(`SMO,${value.toFixed(1)}`)
}

export function pollPosture() {
  getJson('/posture')
    .then((data) => {
      if (data.current) {
        Object.assign(state.posture.current, data.current)
      }
      if (!state.posture.dirty && data.target && hasAllValues(data.target)) {
        Object.assign(state.posture.target, data.target)
        setPostureInputs(state.posture.target)
        state.posture.hydrated = true
      }
      if (!state.posture.smoothingDirty && Number.isFinite(data.smoothing_time)) {
        state.posture.smoothingTime = data.smoothing_time
        el('sl-smo').value = data.smoothing_time
        el('sv-smo').textContent = `${data.smoothing_time.toFixed(1)} s`
      }
      hydratePosture()
      renderPostureReadout()
    })
    .catch(() => {})
}

export function initPostureControls() {
  Object.keys(POSTURE_SUFFIXES).forEach((axis) => {
    el(`sl-${axis}`).addEventListener('input', updatePosture)
  })
  if (Number.isFinite(state.posture.smoothingTime)) {
    el('sl-smo').value = state.posture.smoothingTime
    el('sv-smo').textContent = `${state.posture.smoothingTime.toFixed(1)} s`
  }
  el('sl-smo').addEventListener('input', updateSmoothing)
  el('posture-reset').addEventListener('click', resetPosture)
  renderPostureReadout()
}
