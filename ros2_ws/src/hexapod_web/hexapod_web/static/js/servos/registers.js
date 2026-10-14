import { getServoMeta, hexByte, servoEl, servoState, setText } from './data.js'

function registerHex(value, size) {
  return `0x${value.toString(16).padStart(size * 2, '0').toUpperCase()}`
}

function interpretRegisterValue(register, value) {
  switch (register.format) {
    case 'return_delay':
      return `${value * 2} us`
    case 'position':
      return `${((value * 300) / 1023).toFixed(1)} grados`
    case 'temperature':
      return `${value} C`
    case 'voltage':
      return `${(value / 10).toFixed(1)} V`
    case 'torque':
      return `${((value & 0x03FF) * 100 / 1023).toFixed(1)} %`
    case 'speed': {
      const magnitude = value & 0x03FF
      const direction = value & 0x0400 ? 'CW' : 'CCW'
      return `${(magnitude * 0.111).toFixed(1)} rpm · ${direction}`
    }
    case 'load': {
      const magnitude = value & 0x03FF
      const direction = value & 0x0400 ? 'CW' : 'CCW'
      return `${(magnitude * 100 / 1023).toFixed(1)} % · ${direction}`
    }
    case 'boolean':
      return value === 0 ? 'OFF' : 'ON'
    default:
      return ''
  }
}

function registerValueMarkup(register, result) {
  if (!result) {
    return '<span class="servo-register-empty">Sin leer</span>'
  }
  if (result.pending) {
    return '<span class="servo-register-pending">Leyendo...</span>'
  }
  if (!result.valid || result.value === null) {
    return '<span class="servo-register-error">Sin respuesta</span>'
  }

  const value = result.value
  const interpreted = interpretRegisterValue(register, value)
  const matchesExpected = register.expected === null ||
    register.expected === undefined || register.expected === value
  const expectedMarkup = register.expected === null ||
    register.expected === undefined
    ? ''
    : `<span class="servo-register-check ${matchesExpected ? 'is-ok' : 'is-bad'}">
        ${matchesExpected ? 'valor esperado' : `esperado ${register.expected}`}
      </span>`

  return `
    <div class="servo-register-reading">
      <strong>${value}</strong>
      <span>${registerHex(value, register.size)}</span>
      ${interpreted ? `<span>${interpreted}</span>` : ''}
      ${expectedMarkup}
      <span>ERR ${hexByte(result.error_raw)}</span>
    </div>
  `
}

function registerWriteMarkup(result) {
  if (!result) {
    return ''
  }
  if (result.pending) {
    return '<span class="servo-register-pending">Escribiendo...</span>'
  }
  if (!result.valid) {
    return `<span class="servo-register-error">Escritura fallida · ERR ${hexByte(result.error_raw)}</span>`
  }
  return `<span class="servo-register-write-ok">Escritura aceptada · ERR ${hexByte(result.error_raw)}</span>`
}

function renderServoRegisters() {
  const groups = servoEl('servo-register-groups')
  const servoId = servoState.selectedServoId
  if (!groups || servoId === null) {
    return
  }

  if (servoState.registers.length === 0) {
    groups.innerHTML = '<div class="servo-register-empty">Cargando registros...</div>'
    return
  }

  groups.innerHTML = ['EEPROM', 'RAM'].map((area) => {
    const rows = servoState.registers
      .filter((register) => register.area === area)
      .map((register) => {
        const result = servoState.registerResults[register.key]
        const writeResult = servoState.registerWrites[register.key]
        const maxValue = register.size === 1 ? 255 : 65535
        const inputValue = servoState.registerInputs[register.key] ??
          (result?.valid ? result.value : register.expected ?? 0)
        const writeControls = register.write_enabled
          ? `<div class="servo-register-write-controls">
              <input
                class="servo-register-write-input"
                type="number"
                min="0"
                max="${maxValue}"
                step="1"
                value="${inputValue}"
                data-register-input="${register.key}"
                ${servoState.activeRegisterRequest !== null ? 'disabled' : ''}
                aria-label="Valor RAW para ${register.name}"
              >
              <button
                class="servo-register-write"
                type="button"
                data-register-write="${register.key}"
                ${servoState.activeRegisterRequest !== null ? 'disabled' : ''}
              >Escribir</button>
            </div>
            ${registerWriteMarkup(writeResult)}`
          : ''
        return `
          <tr>
            <td>
              <strong>${register.name}</strong>
              <span>${register.size} byte${register.size === 1 ? '' : 's'}</span>
            </td>
            <td>${register.address}</td>
            <td>${register.access}${register.access === 'RW' && !register.write_enabled
              ? ' · protegido'
              : ''}</td>
            <td>${registerValueMarkup(register, result)}</td>
            <td>
              <div class="servo-register-actions">
                <button
                  class="servo-register-read"
                  type="button"
                  data-register-read="${register.key}"
                  ${servoState.activeRegisterRequest !== null ? 'disabled' : ''}
                >Leer</button>
                ${writeControls}
              </div>
            </td>
          </tr>
        `
      }).join('')

    return `
      <section class="servo-register-group">
        <div class="servo-register-area">${area}</div>
        <div class="servo-register-table-wrap">
          <table class="servo-register-table">
            <thead>
              <tr>
                <th>Registro</th>
                <th>Dir.</th>
                <th>Acceso</th>
                <th>Valor</th>
                <th></th>
              </tr>
            </thead>
            <tbody>${rows}</tbody>
          </table>
        </div>
      </section>
    `
  }).join('')

  groups.querySelectorAll('[data-register-read]').forEach((button) => {
    button.addEventListener('click', () => {
      readServoRegister(button.dataset.registerRead)
    })
  })

  groups.querySelectorAll('[data-register-input]').forEach((input) => {
    input.addEventListener('input', () => {
      servoState.registerInputs[input.dataset.registerInput] = input.value
    })
  })

  groups.querySelectorAll('[data-register-write]').forEach((button) => {
    button.addEventListener('click', () => {
      writeServoRegister(button.dataset.registerWrite)
    })
  })
}

export function openServoRegisters(servoId) {
  const dialog = servoEl('servo-register-dialog')
  const meta = getServoMeta(servoId)
  if (!dialog) {
    return
  }

  servoState.selectedServoId = servoId
  servoState.registerResults = {}
  servoState.registerWrites = {}
  servoState.registerInputs = {}
  servoState.activeRegisterRequest = null
  setText('servo-register-title', `Servo ID ${servoId}`)
  setText('servo-register-subtitle', `${meta.legCode} · ${meta.legName} · ${meta.joint}`)
  setText('servo-register-status', 'Lee un registro o escribe un valor RAW en los registros habilitados.')
  renderServoRegisters()

  if (!dialog.open) {
    dialog.showModal()
  }
}

function waitForRegisterResult(servoId, registerKey, operation, attempts = 20) {
  const query = new URLSearchParams({
    servo_id: servoId,
    register: registerKey,
  })

  fetch(`/servo_register?${query.toString()}`)
    .then((response) => response.json())
    .then((data) => {
      if (data.pending && attempts > 0) {
        setTimeout(
          () => waitForRegisterResult(servoId, registerKey, operation, attempts - 1),
          100,
        )
        return
      }

      if (data.pending) {
        data.pending = false
        data.valid = false
      }
      if (servoState.selectedServoId !== servoId) {
        return
      }
      servoState.activeRegisterRequest = null
      if (operation === 'write') {
        servoState.registerWrites[registerKey] = data
        setText('servo-register-status', data.valid
          ? 'Escritura aceptada. Pulsa Leer para comprobar el valor.'
          : 'El servo no aceptó la escritura.')
      } else {
        servoState.registerResults[registerKey] = data
        setText('servo-register-status', data.valid
          ? 'Lectura completada.'
          : 'No se recibió respuesta del servo.')
      }
      renderServoRegisters()
    })
    .catch(() => {
      if (servoState.selectedServoId !== servoId) {
        return
      }
      servoState.activeRegisterRequest = null
      const failedResult = {
        pending: false,
        valid: false,
        value: null,
      }
      if (operation === 'write') {
        servoState.registerWrites[registerKey] = failedResult
      } else {
        servoState.registerResults[registerKey] = failedResult
      }
      setText('servo-register-status', 'No se pudo completar la petición.')
      renderServoRegisters()
    })
}

function readServoRegister(registerKey) {
  const servoId = servoState.selectedServoId
  if (servoId === null) {
    return
  }
  if (servoState.activeRegisterRequest !== null) {
    return
  }

  servoState.activeRegisterRequest = registerKey
  servoState.registerResults[registerKey] = {
    pending: true,
    valid: false,
    value: null,
  }
  setText('servo-register-status', 'Solicitando lectura a la ESP32...')
  renderServoRegisters()

  fetch('/servo_register', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ servo_id: servoId, register: registerKey }),
  })
    .then((response) => response.json())
    .then((data) => {
      if (!data.ok) {
        throw new Error(data.error || 'Petición rechazada')
      }
      waitForRegisterResult(servoId, registerKey, 'read')
    })
    .catch(() => {
      if (servoState.selectedServoId !== servoId) {
        return
      }
      servoState.activeRegisterRequest = null
      servoState.registerResults[registerKey] = {
        pending: false,
        valid: false,
        value: null,
      }
      setText('servo-register-status', 'No se pudo enviar la petición.')
      renderServoRegisters()
    })
}

function writeServoRegister(registerKey) {
  const servoId = servoState.selectedServoId
  const register = servoState.registers.find((item) => item.key === registerKey)
  const rawValue = servoState.registerInputs[registerKey] ??
    servoEl('servo-register-groups')
      ?.querySelector(`[data-register-input="${registerKey}"]`)?.value
  const value = Number(rawValue)
  const maxValue = register?.size === 1 ? 255 : 65535

  if (servoId === null || !register?.write_enabled ||
      !Number.isInteger(value) || value < 0 || value > maxValue ||
      servoState.activeRegisterRequest !== null) {
    setText('servo-register-status', `Introduce un valor RAW entero entre 0 y ${maxValue}.`)
    return
  }

  const persistentWarning = register.area === 'EEPROM'
    ? ' Este valor se conservará después de apagar el servo.'
    : ''
  if (!window.confirm(
    `Escribir ${value} en ${register.name} del servo ${servoId}.${persistentWarning}`,
  )) {
    return
  }

  servoState.activeRegisterRequest = registerKey
  servoState.registerInputs[registerKey] = String(value)
  servoState.registerWrites[registerKey] = {
    pending: true,
    valid: false,
    value,
    error_raw: null,
  }
  setText('servo-register-status', 'Solicitando escritura a la ESP32...')
  renderServoRegisters()

  fetch('/servo_register', {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ servo_id: servoId, register: registerKey, value }),
  })
    .then((response) => response.json())
    .then((data) => {
      if (!data.ok) {
        throw new Error(data.error || 'Petición rechazada')
      }
      waitForRegisterResult(servoId, registerKey, 'write')
    })
    .catch(() => {
      if (servoState.selectedServoId !== servoId) {
        return
      }
      servoState.activeRegisterRequest = null
      servoState.registerWrites[registerKey] = {
        pending: false,
        valid: false,
        value,
        error_raw: null,
      }
      setText('servo-register-status', 'No se pudo enviar la escritura.')
      renderServoRegisters()
    })
}

export function loadServoRegisters() {
  fetch('/servo_registers')
    .then((response) => response.json())
    .then((data) => {
      servoState.registers = data.registers || []
      renderServoRegisters()
    })
    .catch(() => {})
}

export function initRegisterDialog() {
  const dialog = servoEl('servo-register-dialog')
  const closeButton = servoEl('servo-register-close')
  closeButton.addEventListener('click', () => dialog.close())
  dialog.addEventListener('click', (event) => {
    if (event.target === dialog) {
      dialog.close()
    }
  })
}
