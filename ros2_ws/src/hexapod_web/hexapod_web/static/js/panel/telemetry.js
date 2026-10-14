import { getJson, sendRobotCommand } from './api.js'
import { el } from '../shared/dom.js'

export function pollOdometry() {
  getJson('/odom')
    .then((data) => {
      el('odo-x').textContent = data.x.toFixed(2)
      el('odo-y').textContent = data.y.toFixed(2)
      el('odo-yaw').textContent = data.yaw.toFixed(3)
    })
    .catch(() => {})
}

export function initTelemetryControls() {
  el('odom-reset').addEventListener('click', () => sendRobotCommand('RST'))
}
