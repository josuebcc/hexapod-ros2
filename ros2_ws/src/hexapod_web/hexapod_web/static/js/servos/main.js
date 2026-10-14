import { pollServoHealth, renderServoDiagram } from './diagram.js'
import { initRegisterDialog, loadServoRegisters } from './registers.js'

initRegisterDialog()
renderServoDiagram()
loadServoRegisters()
pollServoHealth()
setInterval(pollServoHealth, 250)
