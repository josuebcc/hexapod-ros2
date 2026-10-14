import { sendVelocity } from './api.js'
import { getYawSliderRange } from './config.js'
import { DUTY_CYCLES, state } from './state.js'
import { el, numberOrZero } from '../shared/dom.js'

let moveJoystick

function getMotionLimits() {
  const duty = DUTY_CYCLES[state.motion.activeGaitMode]
  const stanceDuration = state.motion.currentPeriod * duty

  return {
    vxMax: state.ui.calibration.motion.maxStepXMm / stanceDuration,
    vyMax: state.ui.calibration.motion.maxStepYMm / stanceDuration,
    yawMax: state.ui.calibration.motion.maxYawStepRad / stanceDuration,
  }
}

function updateMotionLimitUi() {
  const { vxMax, vyMax, yawMax } = getMotionLimits()
  el('motion-limit-pill').textContent =
    `vx ${vxMax.toFixed(0)} · vy ${vyMax.toFixed(0)} · giro ${yawMax.toFixed(2)}`
  el('yaw-min-label').textContent = (-yawMax).toFixed(2)
  el('yaw-mid-label').textContent = '0.00'
  el('yaw-max-label').textContent = yawMax.toFixed(2)
}

function updateMotionSourceUi() {
  const isManual = state.motion.inputSource === 'manual'
  const pill = el('motion-source-pill')
  pill.textContent = `entrada ${isManual ? 'manual' : 'joystick'}`
  pill.className = isManual
    ? 'info-chip is-manual'
    : 'info-chip'
}

function setMotionInputSource(source) {
  state.motion.inputSource = source
  updateMotionSourceUi()
}

function refreshRequestedMotion() {
  if (state.motion.inputSource === 'manual') {
    state.motion.vxReq = state.motion.manual.vx
    state.motion.vyReq = state.motion.manual.vy
    state.motion.vyawReq = state.motion.manual.vyaw
    return
  }

  const { vxMax, vyMax, yawMax } = getMotionLimits()
  state.motion.vxReq = state.motion.inputY * vxMax
  state.motion.vyReq = -state.motion.inputX * vyMax
  state.motion.vyawReq = state.motion.yawInput * yawMax
}

function limitedMotion() {
  const { vxMax, vyMax, yawMax } = getMotionLimits()
  const totalDemand =
    (Math.abs(state.motion.vxReq) / vxMax) +
    (Math.abs(state.motion.vyReq) / vyMax) +
    (Math.abs(state.motion.vyawReq) / yawMax)
  const scale = totalDemand > 1.0 ? 1.0 / totalDemand : 1.0

  return {
    vx: state.motion.vxReq * scale,
    vy: state.motion.vyReq * scale,
    vyaw: state.motion.vyawReq * scale,
  }
}

export function sendCurrentCommand() {
  if (state.motion.controlMode === 'manual' && state.motion.activeMode !== 0) {
    sendVelocity(state.motion.vx, state.motion.vy, state.motion.vyaw)
  }
}

export function refreshMotionState() {
  refreshRequestedMotion()
  Object.assign(state.motion, limitedMotion())
  el('vx-v').textContent = state.motion.vx.toFixed(2)
  el('vy-v').textContent = state.motion.vy.toFixed(2)
  el('vyaw-v').textContent = state.motion.vyaw.toFixed(3)
  updateMotionLimitUi()
  sendCurrentCommand()
}

function makeJoystick(onMove) {
  const zone = el('joy-move')
  const knob = el('jmk')
  let active = false
  let originX = 0
  let originY = 0

  function getTravelRadius() {
    const zoneRect = zone.getBoundingClientRect()
    const knobRect = knob.getBoundingClientRect()
    return Math.max(Math.min(
      (zoneRect.width - knobRect.width) / 2,
      (zoneRect.height - knobRect.height) / 2,
    ), 1)
  }

  function update(cursorX, cursorY) {
    const radius = getTravelRadius()
    let deltaX = cursorX - originX
    let deltaY = cursorY - originY
    const distance = Math.hypot(deltaX, deltaY)

    if (distance > radius) {
      deltaX = (deltaX / distance) * radius
      deltaY = (deltaY / distance) * radius
    }

    knob.style.transform =
      `translate(calc(-50% + ${deltaX}px), calc(-50% + ${deltaY}px))`
    onMove(
      parseFloat((deltaX / radius).toFixed(2)),
      parseFloat((-deltaY / radius).toFixed(2)),
    )
  }

  function reset(silent = false) {
    knob.style.transform = 'translate(-50%, -50%)'
    if (!silent) {
      onMove(0, 0)
    }
    active = false
  }

  function start(cursorX, cursorY) {
    const zoneRect = zone.getBoundingClientRect()
    originX = zoneRect.left + (zoneRect.width / 2)
    originY = zoneRect.top + (zoneRect.height / 2)
    active = true
    update(cursorX, cursorY)
  }

  function release() {
    if (!active) {
      return
    }
    if (state.motion.elastic) {
      reset()
    } else {
      active = false
    }
  }

  zone.addEventListener('mousedown', (event) => start(event.clientX, event.clientY))
  window.addEventListener('mousemove', (event) => {
    if (active) {
      update(event.clientX, event.clientY)
    }
  })
  window.addEventListener('mouseup', release)

  zone.addEventListener('touchstart', (event) => {
    event.preventDefault()
    start(event.touches[0].clientX, event.touches[0].clientY)
  }, { passive: false })
  window.addEventListener('touchmove', (event) => {
    if (active) {
      update(event.touches[0].clientX, event.touches[0].clientY)
    }
  }, { passive: true })
  window.addEventListener('touchend', release)

  return { reset }
}

function onYawSlider(source = 'joystick') {
  const rawYaw = parseFloat(el('sl-vyaw').value) / getYawSliderRange()
  state.motion.yawInput = Math.max(-1.0, Math.min(1.0, rawYaw))
  if (source) {
    setMotionInputSource(source)
  }
  refreshMotionState()
}

function resetYaw(source = 'joystick') {
  el('sl-vyaw').value = 0
  onYawSlider(source)
}

function updateMotionElasticUi() {
  const button = el('motion-elastic-btn')
  button.textContent = `Elástico: ${state.motion.elastic ? 'ON' : 'OFF'}`
  button.className = state.motion.elastic
    ? 'button button-toggle is-on'
    : 'button button-toggle'
}

function toggleMotionElastic() {
  state.motion.elastic = !state.motion.elastic
  if (state.motion.elastic) {
    moveJoystick.reset(state.motion.inputSource === 'manual')
    resetYaw(state.motion.inputSource === 'manual' ? null : 'joystick')
  }
  updateMotionElasticUi()
}

function readManualMotionInputs() {
  return {
    vx: numberOrZero(el('manual-vx').value),
    vy: numberOrZero(el('manual-vy').value),
    vyaw: numberOrZero(el('manual-vyaw').value),
  }
}

function syncManualMotionInputs() {
  el('manual-vx').value = state.motion.manual.vx.toFixed(1)
  el('manual-vy').value = state.motion.manual.vy.toFixed(1)
  el('manual-vyaw').value = state.motion.manual.vyaw.toFixed(3)
}

function neutralizeJoystick() {
  state.motion.inputX = 0
  state.motion.inputY = 0
  state.motion.yawInput = 0
  moveJoystick.reset(true)
  el('sl-vyaw').value = 0
}

function applyManualMotion() {
  state.motion.manual = readManualMotionInputs()
  setMotionInputSource('manual')
  neutralizeJoystick()
  refreshMotionState()
}

export function resetManualMotion() {
  state.motion.manual = { vx: 0, vy: 0, vyaw: 0 }
  syncManualMotionInputs()
  setMotionInputSource('manual')
  neutralizeJoystick()
  refreshMotionState()
}

export function initMotionControls() {
  moveJoystick = makeJoystick((normalizedX, normalizedY) => {
    setMotionInputSource('joystick')
    state.motion.inputX = normalizedX
    state.motion.inputY = normalizedY
    refreshMotionState()
  })

  el('motion-elastic-btn').addEventListener('click', toggleMotionElastic)
  el('sl-vyaw').addEventListener('input', () => onYawSlider())
  el('manual-apply').addEventListener('click', applyManualMotion)
  el('manual-reset').addEventListener('click', resetManualMotion)
  ;['manual-vx', 'manual-vy', 'manual-vyaw'].forEach((id) => {
    el(id).addEventListener('keydown', (event) => {
      if (event.key === 'Enter') {
        applyManualMotion()
      }
    })
  })

  const releaseYaw = () => {
    if (state.motion.elastic) {
      resetYaw()
    }
  }
  ;['mouseup', 'touchend', 'touchcancel', 'pointerup', 'pointercancel']
    .forEach((eventName) => el('sl-vyaw').addEventListener(eventName, releaseYaw))

  updateMotionElasticUi()
  updateMotionSourceUi()
  syncManualMotionInputs()
  refreshMotionState()
}
