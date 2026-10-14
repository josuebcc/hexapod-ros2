export function el(id) {
  return document.getElementById(id)
}

export function log(message, outgoing = true) {
  const logEl = el('log')
  const row = document.createElement('p')
  row.textContent = message
  if (outgoing) {
    row.className = 'out'
  }

  logEl.appendChild(row)
  logEl.scrollTop = logEl.scrollHeight

  if (logEl.children.length > 60) {
    logEl.removeChild(logEl.firstChild)
  }
}

export function formatPostureValue(value, suffix, preferInteger = false) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return '--'
  }

  if (preferInteger && Math.abs(value - Math.round(value)) < 0.05) {
    return `${Math.round(value)}${suffix}`
  }

  return `${value.toFixed(1)}${suffix}`
}

export function hasAllValues(values) {
  return Object.values(values).every(
    (value) => value !== null && value !== undefined && !Number.isNaN(value),
  )
}

export function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value))
}

export function numberOrZero(rawValue) {
  const parsedValue = parseFloat(rawValue)
  return Number.isFinite(parsedValue) ? parsedValue : 0
}

export function quantizeToStep(value, min, step) {
  if (!step || step <= 0) {
    return value
  }

  const ticks = Math.round((value - min) / step)
  return min + (ticks * step)
}
