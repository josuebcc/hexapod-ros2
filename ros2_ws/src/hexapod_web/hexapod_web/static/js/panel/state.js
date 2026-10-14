function emptyPostureValues(defaultValue) {
  return {
    roll: defaultValue,
    pitch: defaultValue,
    yaw: defaultValue,
    x: defaultValue,
    y: defaultValue,
    z: defaultValue,
  }
}

export const DUTY_CYCLES = { 1: 1 / 2, 2: 5 / 6, 3: 2 / 3 }
export const MIN_GAIT_PERIODS = { 1: 1.0, 2: 3.0, 3: 1.5 }
export const MAX_STEP_HEIGHT_MM = 60
export const POSTURE_SUFFIXES = {
  roll: '°',
  pitch: '°',
  yaw: '°',
  x: 'mm',
  y: 'mm',
  z: 'mm',
}

export const state = {
  motion: {
    controlMode: null,
    vx: 0,
    vy: 0,
    vyaw: 0,
    vxReq: 0,
    vyReq: 0,
    vyawReq: 0,
    inputX: 0,
    inputY: 0,
    yawInput: 0,
    inputSource: 'joystick',
    manual: { vx: 0, vy: 0, vyaw: 0 },
    activeMode: 0,
    activeGaitMode: 1,
    elastic: true,
    currentPeriod: 2.0,
    stepHeightMm: 30,
  },
  posture: {
    target: emptyPostureValues(0),
    current: emptyPostureValues(null),
    smoothingTime: null,
    smoothingDirty: false,
    hydrated: false,
    dirty: false,
  },
  ui: {
    calibration: null,
  },
}
