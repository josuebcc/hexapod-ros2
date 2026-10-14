import { applyUiCalibration, loadUiConfig } from './config.js'
import { initGaitControls } from './gait.js'
import { initControlMode } from './control_mode.js'
import { initMotionControls, sendCurrentCommand } from './motion.js'
import { initPostureControls, pollPosture } from './posture.js'
import { hydrateRobotState } from './robot_state.js'
import { state } from './state.js'
import { initTelemetryControls, pollOdometry } from './telemetry.js'
import { log } from '../shared/dom.js'

async function bootPanel() {
  try {
    await Promise.all([loadUiConfig(), hydrateRobotState()])
  } catch (error) {
    log(`No se pudo iniciar el panel: ${error}`, false)
    return
  }
  applyUiCalibration()
  initMotionControls()
  initControlMode()
  initGaitControls()
  initPostureControls()
  initTelemetryControls()

  setInterval(
    sendCurrentCommand,
    1000 / state.ui.calibration.motion.cmdVelRateHz,
  )
  setInterval(pollOdometry, 1000)
  setInterval(pollPosture, 200)
  pollOdometry()
  pollPosture()
  log(`Panel conectado a ${window.location.origin}`, false)
}

bootPanel()
