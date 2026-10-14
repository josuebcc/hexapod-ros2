import {
  defaultServo,
  formatServoValue,
  getServoMeta,
  SERVO_ERROR_FLAGS,
  SERVO_META,
  servoEl,
  servoState,
} from './data.js'
import { openServoRegisters } from './registers.js'

function formatLoad(servo) {
  if (!servo.valid) {
    return '--'
  }
  return `${servo.load_dir || '+'}${formatServoValue(servo.load_pct, '%')}`
}

function evaluateServoStatus(servo) {
  const reasons = []
  if (!servo.valid) {
    return { isFailure: true, severity: 'stale', reasons: ['Sin telemetría'] }
  }

  SERVO_ERROR_FLAGS.forEach((flag) => {
    if ((servo.error_raw ?? 0) & flag.bit) {
      reasons.push(flag.label)
    }
  })
  if ((servo.temperature_c ?? 0) >= 60 && !((servo.error_raw ?? 0) & 0x04)) {
    reasons.push('Temperatura crítica')
  }

  return {
    isFailure: reasons.length > 0,
    severity: reasons.length > 0
      ? 'hot'
      : ((servo.temperature_c ?? 0) >= 50 ? 'warn' : 'ok'),
    reasons,
  }
}

function ensureServoLayout() {
  const stage = servoEl('servo-hexapod-stage')
  if (!stage || servoEl('servo-joint-1')) {
    return
  }

  stage.innerHTML = `
    <div class="servo-stage-bg">
      <svg class="servo-skeleton" viewBox="0 0 1000 780" aria-hidden="true">
        <polyline points="500,270 360,240 245,185 120,120"></polyline>
        <polyline points="500,390 350,390 215,390 85,390"></polyline>
        <polyline points="500,510 360,540 245,595 120,660"></polyline>
        <polyline points="500,270 640,240 755,185 880,120"></polyline>
        <polyline points="500,390 650,390 785,390 915,390"></polyline>
        <polyline points="500,510 640,540 755,595 880,660"></polyline>
      </svg>
      <div class="servo-body-panel">
        <div class="servo-body-label">HEXAPODO</div>
      </div>
      ${Array.from(SERVO_META.values())
        .sort((left, right) => left.id - right.id)
        .map((servo) => `
          <article
            class="servo-joint-node ${servo.nodeClass}"
            id="servo-joint-${servo.id}"
          ></article>
        `)
        .join('')}
    </div>
  `
}

function renderJointNode(servo, status) {
  const meta = getServoMeta(servo.id)
  const node = servoEl(`servo-joint-${servo.id}`)
  node.className = `servo-joint-node ${meta.nodeClass} servo-joint-${status.severity}`
  node.tabIndex = 0
  node.setAttribute('role', 'button')
  node.setAttribute('aria-label', `Abrir registros del servo ${servo.id}`)
  node.onclick = () => openServoRegisters(servo.id)
  node.onkeydown = (event) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault()
      openServoRegisters(servo.id)
    }
  }
  node.innerHTML = `
    <div class="servo-joint-top">
      <span class="servo-joint-id">ID ${servo.id}</span>
      <span class="servo-joint-pill">${meta.legCode}</span>
    </div>
    <div class="servo-joint-name">${meta.joint}</div>
    <div class="servo-joint-reading">
      ${servo.valid ? formatServoValue(servo.temperature_c, 'C') : 'sin dato'}
    </div>
    <div class="servo-joint-meta">
      ${status.isFailure
        ? status.reasons.join(' · ')
        : (servo.valid
          ? `${formatLoad(servo)} · ${formatServoValue(servo.voltage_v, 'V')}`
          : meta.legName)}
    </div>
  `
}

export function renderServoDiagram() {
  ensureServoLayout()
  const servoIndex = new Map(servoState.servos.map((servo) => [servo.id, servo]))
  Array.from(SERVO_META.keys())
    .sort((left, right) => left - right)
    .map((id) => ({ ...defaultServo(id), ...(servoIndex.get(id) || {}) }))
    .forEach((servo) => renderJointNode(servo, evaluateServoStatus(servo)))
}

export function pollServoHealth() {
  fetch('/servo_health')
    .then((response) => response.json())
    .then((data) => {
      servoState.servos = data.servos || []
      renderServoDiagram()
    })
    .catch(() => {})
}
