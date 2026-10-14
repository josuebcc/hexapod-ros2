import { sendRobotCommand } from './api.js'
import { refreshMotionState } from './motion.js'
import { MAX_STEP_HEIGHT_MM, MIN_GAIT_PERIODS, state } from './state.js'
import { clamp, el } from '../shared/dom.js'

function limitedGaitInputs() {
  const periodSlider = el('sl-period')
  const stepSlider = el('sl-step')
  const minPeriod = MIN_GAIT_PERIODS[state.motion.activeGaitMode]
  const period = clamp(
    parseFloat(periodSlider.value),
    minPeriod,
    parseFloat(periodSlider.max),
  )
  const step = clamp(
    parseFloat(stepSlider.value),
    parseFloat(stepSlider.min),
    MAX_STEP_HEIGHT_MM,
  )

  periodSlider.min = minPeriod
  periodSlider.value = period
  stepSlider.max = MAX_STEP_HEIGHT_MM
  stepSlider.value = step
  state.motion.currentPeriod = period
  state.motion.stepHeightMm = step
  el('sv-period').textContent = `${period.toFixed(1)} s`
  el('sv-step').textContent = `${step} mm`
  return { period, step }
}

function setMode(mode) {
  state.motion.activeMode = mode
  if (MIN_GAIT_PERIODS[mode] !== undefined) {
    state.motion.activeGaitMode = mode
  }

  ;[0, 1, 2, 3].forEach((value) => {
    el(`m${value}`).classList.toggle('active', value === mode)
  })

  sendRobotCommand(`MOD,${mode}`)
  if (mode !== 0) {
    const { period, step } = limitedGaitInputs()
    sendRobotCommand(`GAT,${period.toFixed(1)},${step}`)
  }
  refreshMotionState()
}

function updateGait() {
  const { period, step } = limitedGaitInputs()
  sendRobotCommand(`GAT,${period.toFixed(1)},${step}`)
  refreshMotionState()
}

export function initGaitControls() {
  ;[0, 1, 2, 3].forEach((mode) => {
    el(`m${mode}`).classList.toggle('active', mode === state.motion.activeMode)
    el(`m${mode}`).addEventListener('click', () => setMode(mode))
  })
  el('sl-period').value = state.motion.currentPeriod
  el('sl-step').value = state.motion.stepHeightMm
  limitedGaitInputs()
  el('sl-period').addEventListener('input', updateGait)
  el('sl-step').addEventListener('input', updateGait)
}
