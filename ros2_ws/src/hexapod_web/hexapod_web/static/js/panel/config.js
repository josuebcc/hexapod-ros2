import { getJson } from './api.js'
import { state } from './state.js'
import { clamp, el, quantizeToStep } from '../shared/dom.js'

export function getYawSliderRange() {
  return Math.round(state.ui.calibration.motion.maxYawStepRad * 1000)
}

function getPostureSliderConfig() {
  return {
    roll: state.ui.calibration.posture.rollDeg,
    pitch: state.ui.calibration.posture.pitchDeg,
    yaw: state.ui.calibration.posture.yawDeg,
    x: state.ui.calibration.posture.xMm,
    y: state.ui.calibration.posture.yMm,
    z: state.ui.calibration.posture.zMm,
  }
}

function sliderValueForAxis(axis, value) {
  const config = getPostureSliderConfig()[axis]
  const clamped = clamp(value, config.min, config.max)
  return quantizeToStep(clamped, config.min, config.step)
}

export function setPostureInputs(posture) {
  Object.entries(posture).forEach(([axis, value]) => {
    if (value !== null && value !== undefined && !Number.isNaN(value)) {
      el(`sl-${axis}`).value = sliderValueForAxis(axis, value)
    }
  })
}

export function applyUiCalibration() {
  const yawSlider = el('sl-vyaw')
  const yawRange = getYawSliderRange()
  yawSlider.min = -yawRange
  yawSlider.max = yawRange
  yawSlider.step = 1

  Object.entries(getPostureSliderConfig()).forEach(([name, config]) => {
    const slider = el(`sl-${name}`)
    slider.min = config.min
    slider.max = config.max
    slider.step = config.step
  })
}

export async function loadUiConfig() {
  const data = await getJson('/config')
  if (!data.motion || !data.posture) {
    throw new Error('Configuración web incompleta')
  }
  state.ui.calibration = data
}
