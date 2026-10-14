export const servoState = {
  servos: [],
  registers: [],
  registerResults: {},
  registerWrites: {},
  registerInputs: {},
  selectedServoId: null,
  activeRegisterRequest: null,
}

const LEG_CONFIG = [
  {
    code: 'LT',
    name: 'Izquierda delantera',
    motors: [
      { id: 1, joint: 'Coxa', nodeClass: 'servo-node-lt-coxa' },
      { id: 3, joint: 'Femur', nodeClass: 'servo-node-lt-femur' },
      { id: 5, joint: 'Tibia', nodeClass: 'servo-node-lt-tibia' },
    ],
  },
  {
    code: 'LM',
    name: 'Izquierda media',
    motors: [
      { id: 13, joint: 'Coxa', nodeClass: 'servo-node-lm-coxa' },
      { id: 15, joint: 'Femur', nodeClass: 'servo-node-lm-femur' },
      { id: 17, joint: 'Tibia', nodeClass: 'servo-node-lm-tibia' },
    ],
  },
  {
    code: 'LB',
    name: 'Izquierda trasera',
    motors: [
      { id: 7, joint: 'Coxa', nodeClass: 'servo-node-lb-coxa' },
      { id: 9, joint: 'Femur', nodeClass: 'servo-node-lb-femur' },
      { id: 11, joint: 'Tibia', nodeClass: 'servo-node-lb-tibia' },
    ],
  },
  {
    code: 'RT',
    name: 'Derecha delantera',
    motors: [
      { id: 2, joint: 'Coxa', nodeClass: 'servo-node-rt-coxa' },
      { id: 4, joint: 'Femur', nodeClass: 'servo-node-rt-femur' },
      { id: 6, joint: 'Tibia', nodeClass: 'servo-node-rt-tibia' },
    ],
  },
  {
    code: 'RM',
    name: 'Derecha media',
    motors: [
      { id: 14, joint: 'Coxa', nodeClass: 'servo-node-rm-coxa' },
      { id: 16, joint: 'Femur', nodeClass: 'servo-node-rm-femur' },
      { id: 18, joint: 'Tibia', nodeClass: 'servo-node-rm-tibia' },
    ],
  },
  {
    code: 'RB',
    name: 'Derecha trasera',
    motors: [
      { id: 8, joint: 'Coxa', nodeClass: 'servo-node-rb-coxa' },
      { id: 10, joint: 'Femur', nodeClass: 'servo-node-rb-femur' },
      { id: 12, joint: 'Tibia', nodeClass: 'servo-node-rb-tibia' },
    ],
  },
]

export const SERVO_META = new Map(
  LEG_CONFIG.flatMap((leg) =>
    leg.motors.map((motor) => [
      motor.id,
      { ...motor, legCode: leg.code, legName: leg.name },
    ]),
  ),
)

export const SERVO_ERROR_FLAGS = [
  { bit: 0x01, label: 'Voltaje fuera de rango' },
  { bit: 0x02, label: 'Límite angular' },
  { bit: 0x04, label: 'Sobrecalentamiento' },
  { bit: 0x08, label: 'Comando fuera de rango' },
  { bit: 0x10, label: 'Checksum' },
  { bit: 0x20, label: 'Sobrecarga' },
  { bit: 0x40, label: 'Instrucción inválida' },
]

export function servoEl(id) {
  return document.getElementById(id)
}

export function formatServoValue(value, suffix = '') {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return `--${suffix}`
  }
  const decimals = Math.abs(value) >= 10 ? 1 : 2
  return `${value.toFixed(decimals)}${suffix}`
}

export function hexByte(value) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return '--'
  }
  return `0x${value.toString(16).padStart(2, '0').toUpperCase()}`
}

export function getServoMeta(id) {
  return SERVO_META.get(id) || {
    id,
    joint: 'Joint',
    legCode: '--',
    legName: 'Desconocido',
    nodeClass: '',
  }
}

export function defaultServo(id) {
  return {
    id,
    valid: false,
    voltage_v: null,
    temperature_c: null,
    load_pct: null,
    load_dir: '+',
    error_raw: null,
  }
}

export function setText(id, value) {
  const node = servoEl(id)
  if (node) {
    node.textContent = value
  }
}
