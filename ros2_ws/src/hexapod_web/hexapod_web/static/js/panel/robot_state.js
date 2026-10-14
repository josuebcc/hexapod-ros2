import { getJson } from './api.js'
import { state } from './state.js'
import { log } from '../shared/dom.js'

function applyStoredConfiguration(robotState) {
  if ([0, 1, 2, 3].includes(robotState.mode)) {
    state.motion.activeMode = robotState.mode
  }
  if ([1, 2, 3].includes(robotState.gait_mode)) {
    state.motion.activeGaitMode = robotState.gait_mode
  }
  if (Number.isFinite(robotState.gait_period_s)) {
    state.motion.currentPeriod = robotState.gait_period_s
  }
  if (Number.isFinite(robotState.step_height_mm)) {
    state.motion.stepHeightMm = robotState.step_height_mm
  }
  if (Number.isFinite(robotState.smoothing_time)) {
    state.posture.smoothingTime = robotState.smoothing_time
  }
}

export function hydrateRobotState() {
  return getJson('/robot_state')
    .then(applyStoredConfiguration)
    .catch((error) => log(`Estado del robot no disponible: ${error}`, false))
}
