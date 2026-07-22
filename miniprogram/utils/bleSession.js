/**
 * 全局 BLE 会话：连接状态在各 Tab 间共享
 */
const ble = require('./ble.js')

const state = {
  scanning: false,
  connecting: false,
  connected: false,
  statusText: '未连接',
  deviceId: '',
  deviceName: '',
  serviceId: '',
  cmdCharId: '',
  statusCharId: '',
  angleText: '--',
  batteryText: '--',
  pwmText: '--',
  lastMsg: '',
  kp: 25,
  kd: 0.5,
  kv: 0.02
}

const listeners = []
let foundHandler = null
let valueHandler = null
let connHandler = null
let scanTimer = null
let suppressStatus = false

function getState() {
  return Object.assign({}, state)
}

function setState(patch) {
  Object.assign(state, patch)
  const snap = getState()
  listeners.slice().forEach((fn) => {
    try {
      fn(snap)
    } catch (e) {
      console.error(e)
    }
  })
}

function subscribe(fn) {
  if (typeof fn !== 'function') return function () {}
  listeners.push(fn)
  fn(getState())
  return function unsubscribe() {
    const i = listeners.indexOf(fn)
    if (i >= 0) listeners.splice(i, 1)
  }
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms))
}

function round(v, n) {
  const p = Math.pow(10, n)
  return Math.round(v * p) / p
}

function bindConnectionListener() {
  if (connHandler) {
    wx.offBLEConnectionStateChange(connHandler)
  }
  connHandler = (res) => {
    if (!res.connected && res.deviceId === state.deviceId) {
      setState({
        connected: false,
        statusText: '已断开',
        angleText: '--',
        batteryText: '--',
        pwmText: '--'
      })
    }
  }
  wx.onBLEConnectionStateChange(connHandler)
}

function bindValueListener() {
  if (valueHandler) {
    wx.offBLECharacteristicValueChange(valueHandler)
  }
  valueHandler = (res) => {
    if (res.deviceId !== state.deviceId) return
    const text = ble.ab2str(res.value)
    const parsed = ble.parseStatus(text)
    const patch = { lastMsg: text }

    const isTelemetry = parsed.ANG !== undefined
    if (!isTelemetry && !suppressStatus) {
      if (parsed.KP !== undefined) patch.kp = round(parsed.KP, 2)
      if (parsed.KD !== undefined) patch.kd = round(parsed.KD, 2)
      if (parsed.KV !== undefined) patch.kv = round(parsed.KV, 4)
    }
    if (parsed.ANG !== undefined) patch.angleText = parsed.ANG.toFixed(2)
    if (parsed.BAT !== undefined) patch.batteryText = parsed.BAT.toFixed(0)
    if (parsed.PWM !== undefined) patch.pwmText = parsed.PWM.toFixed(1)

    setState(patch)
  }
  wx.onBLECharacteristicValueChange(valueHandler)
}

function waitForDevice() {
  if (foundHandler) {
    wx.offBluetoothDeviceFound(foundHandler)
  }

  foundHandler = (res) => {
    const devices = res.devices || []
    const target = devices.find((d) => {
      const name = d.name || d.localName || ''
      return name === ble.DEVICE_NAME || name.indexOf('ESP32-Car') >= 0
    })
    if (!target) return

    if (scanTimer) {
      clearTimeout(scanTimer)
      scanTimer = null
    }
    wx.offBluetoothDeviceFound(foundHandler)
    foundHandler = null
    connectDevice(target)
  }

  wx.onBluetoothDeviceFound(foundHandler)

  scanTimer = setTimeout(() => {
    scanTimer = null
    if (foundHandler) {
      wx.offBluetoothDeviceFound(foundHandler)
      foundHandler = null
    }
    ble.stopDiscovery()
    setState({
      scanning: false,
      statusText: '未找到设备',
      lastMsg: '确认 ESP32 已上电并广播 ESP32-Car'
    })
  }, 12000)
}

async function connectDevice(device) {
  setState({
    scanning: false,
    connecting: true,
    statusText: '连接中…',
    deviceName: device.name || device.localName || ble.DEVICE_NAME
  })

  try {
    await ble.stopDiscovery()
    await ble.createConnection(device.deviceId)
    await sleep(400)

    try {
      await ble.setMtu(device.deviceId, 128)
    } catch (e) {
      console.warn('setBLEMTU failed', e)
    }

    const services = await ble.getServices(device.deviceId)
    const service = ble.findUuid(services, ble.SERVICE_UUID)
    if (!service) throw new Error('未找到 FFF0 服务')

    const chars = await ble.getCharacteristics(device.deviceId, service.uuid)
    const cmdChar = ble.findUuid(chars, ble.CHAR_CMD_UUID)
    const statusChar = ble.findUuid(chars, ble.CHAR_STATUS_UUID)
    if (!cmdChar || !statusChar) throw new Error('未找到指令/状态特征')

    setState({
      connected: true,
      connecting: false,
      statusText: '已连接',
      deviceId: device.deviceId,
      serviceId: service.uuid,
      cmdCharId: cmdChar.uuid,
      statusCharId: statusChar.uuid
    })

    bindValueListener()
    await ble.enableNotify(device.deviceId, service.uuid, statusChar.uuid)
    await sleep(200)
    await sendCmd('GET')
    wx.showToast({ title: '连接成功', icon: 'success' })
  } catch (err) {
    console.error(err)
    await ble.closeConnection(device.deviceId)
    setState({
      connecting: false,
      connected: false,
      statusText: '连接失败',
      deviceId: '',
      lastMsg: (err && (err.message || err.errMsg)) || '连接失败'
    })
    wx.showToast({ title: '连接失败', icon: 'none' })
  }
}

async function scanConnect() {
  if (state.connected || state.scanning || state.connecting) return

  setState({
    scanning: true,
    statusText: '扫描中…',
    lastMsg: ''
  })

  try {
    await ble.openAdapter()
    bindConnectionListener()
    await ble.startDiscovery()
    waitForDevice()
  } catch (err) {
    console.error(err)
    setState({
      scanning: false,
      statusText: '蓝牙不可用',
      lastMsg: (err && err.errMsg) || '请打开手机蓝牙并授权'
    })
    wx.showToast({ title: '请打开蓝牙', icon: 'none' })
  }
}

async function cleanup() {
  if (scanTimer) {
    clearTimeout(scanTimer)
    scanTimer = null
  }
  if (foundHandler) {
    wx.offBluetoothDeviceFound(foundHandler)
    foundHandler = null
  }
  if (valueHandler) {
    wx.offBLECharacteristicValueChange(valueHandler)
    valueHandler = null
  }
  if (connHandler) {
    wx.offBLEConnectionStateChange(connHandler)
    connHandler = null
  }
  await ble.stopDiscovery()
  await ble.closeConnection(state.deviceId)
  await ble.closeAdapter()
}

async function disconnect() {
  await cleanup()
  setState({
    connected: false,
    connecting: false,
    scanning: false,
    statusText: '未连接',
    deviceId: '',
    deviceName: '',
    serviceId: '',
    cmdCharId: '',
    statusCharId: '',
    angleText: '--',
    batteryText: '--',
    pwmText: '--',
    lastMsg: ''
  })
}

async function sendCmd(cmd) {
  const { deviceId, serviceId, cmdCharId } = state
  if (!deviceId || !serviceId || !cmdCharId) {
    throw new Error('未连接')
  }
  await ble.writeCommand(deviceId, serviceId, cmdCharId, cmd)
}

function updateParams(patch) {
  setState(patch)
}

function setSuppressStatus(v) {
  suppressStatus = !!v
}

module.exports = {
  getState,
  subscribe,
  scanConnect,
  disconnect,
  sendCmd,
  updateParams,
  setSuppressStatus,
  round
}
