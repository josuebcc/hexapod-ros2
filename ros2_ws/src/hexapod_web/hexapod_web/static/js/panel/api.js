import { log } from '../shared/dom.js'

let velocityRequestPending = false

export function sendRobotCommand(command) {
  log(`-> ${command}`)
  fetch('/robot_command', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ cmd: command }),
  }).catch((error) => log(`ERR: ${error}`, false))
}

export function sendVelocity(vxMmS, vyMmS, yawRadS) {
  if (velocityRequestPending) {
    return
  }

  velocityRequestPending = true
  fetch('/cmd_vel', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      linear_x: vxMmS / 1000,
      linear_y: vyMmS / 1000,
      angular_z: yawRadS,
    }),
  })
    .catch((error) => log(`ERR: ${error}`, false))
    .finally(() => {
      velocityRequestPending = false
    })
}

export function getJson(path) {
  return fetch(path).then((response) => {
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`)
    }
    return response.json()
  })
}
