/**
 * ESP32-Car BLE 协议封装
 * 设备名: ESP32-Car
 * Service: 0000FFF0-...
 * Command (Write): 0000FFF1-...
 * Status  (Notify/Read): 0000FFF2-...
 *
 * PID 参数: PID=AKP,AKI,AKD,SKP,SKI,SKD,TKP,TKI,TKD[,A0]
 * 兼容旧指令: KP= / KI= / KD= / KV= / SET=kp,kd,kv[,a0]
 * 遥测: ANG=xx,BAT=xx,PWM=xx,LSP=xx,RSP=xx
 * 应答分两帧返回角度/速度 PID 与转向 PID 参数
 * 校准: CAL=0 开始，CAL=1,GX=,GY=,GZ= 完成
 * A0: 机械零点（平衡倾角偏置，单位 °，范围 ±5）
 * SPD: 目标速度 targetSpeed，前进为正、后退为负
 * TRN: 目标偏航角速度（°/s），右转为正、左转为负
 * SLW: 急刹强度（目标速度斜坡），值越大越剧烈
 * GCAL: 陀螺仪零偏校准（保持静止）
 */

const DEVICE_NAME = 'ESP32-Car'
const SERVICE_UUID = '0000FFF0-0000-1000-8000-00805F9B34FB'
const CHAR_CMD_UUID = '0000FFF1-0000-1000-8000-00805F9B34FB'
const CHAR_STATUS_UUID = '0000FFF2-0000-1000-8000-00805F9B34FB'

function ab2str(buffer) {
  const bytes = new Uint8Array(buffer)
  let s = ''
  for (let i = 0; i < bytes.length; i++) {
    s += String.fromCharCode(bytes[i])
  }
  return s
}

function str2ab(str) {
  const buffer = new ArrayBuffer(str.length)
  const view = new Uint8Array(buffer)
  for (let i = 0; i < str.length; i++) {
    view[i] = str.charCodeAt(i)
  }
  return buffer
}

function parseStatus(text) {
  const result = {}
  const parts = String(text || '').split(',')
  parts.forEach((part) => {
    const idx = part.indexOf('=')
    if (idx < 0) return
    const key = part.slice(0, idx).trim().replace(/^(OK|SAVED)\s+/i, '')
    const val = parseFloat(part.slice(idx + 1))
    if (!Number.isNaN(val)) {
      result[key.toUpperCase()] = val
    }
  })
  return result
}

function openAdapter() {
  return new Promise((resolve, reject) => {
    wx.openBluetoothAdapter({
      success: resolve,
      fail: reject
    })
  })
}

function closeAdapter() {
  return new Promise((resolve) => {
    wx.closeBluetoothAdapter({
      complete: resolve
    })
  })
}

function startDiscovery() {
  return new Promise((resolve, reject) => {
    wx.startBluetoothDevicesDiscovery({
      allowDuplicatesKey: true,
      success: resolve,
      fail: reject
    })
  })
}

function stopDiscovery() {
  return new Promise((resolve) => {
    wx.stopBluetoothDevicesDiscovery({
      complete: resolve
    })
  })
}

function createConnection(deviceId) {
  return new Promise((resolve, reject) => {
    wx.createBLEConnection({
      deviceId,
      timeout: 10000,
      success: resolve,
      fail: reject
    })
  })
}

function closeConnection(deviceId) {
  return new Promise((resolve) => {
    if (!deviceId) {
      resolve()
      return
    }
    wx.closeBLEConnection({
      deviceId,
      complete: resolve
    })
  })
}

function setMtu(deviceId, mtu) {
  return new Promise((resolve, reject) => {
    if (typeof wx.setBLEMTU !== 'function') {
      resolve()
      return
    }
    wx.setBLEMTU({
      deviceId,
      mtu,
      success: resolve,
      fail: reject
    })
  })
}

function getServices(deviceId) {
  return new Promise((resolve, reject) => {
    wx.getBLEDeviceServices({
      deviceId,
      success: (res) => resolve(res.services || []),
      fail: reject
    })
  })
}

function getCharacteristics(deviceId, serviceId) {
  return new Promise((resolve, reject) => {
    wx.getBLEDeviceCharacteristics({
      deviceId,
      serviceId,
      success: (res) => resolve(res.characteristics || []),
      fail: reject
    })
  })
}

function writeCommand(deviceId, serviceId, characteristicId, text) {
  return new Promise((resolve, reject) => {
    wx.writeBLECharacteristicValue({
      deviceId,
      serviceId,
      characteristicId,
      value: str2ab(text),
      success: resolve,
      fail: reject
    })
  })
}

function enableNotify(deviceId, serviceId, characteristicId) {
  return new Promise((resolve, reject) => {
    wx.notifyBLECharacteristicValueChange({
      deviceId,
      serviceId,
      characteristicId,
      state: true,
      success: resolve,
      fail: reject
    })
  })
}

function findUuid(list, target) {
  const t = target.toLowerCase()
  return (list || []).find((item) => String(item.uuid || '').toLowerCase() === t)
}

module.exports = {
  DEVICE_NAME,
  SERVICE_UUID,
  CHAR_CMD_UUID,
  CHAR_STATUS_UUID,
  ab2str,
  str2ab,
  parseStatus,
  openAdapter,
  closeAdapter,
  startDiscovery,
  stopDiscovery,
  createConnection,
  closeConnection,
  setMtu,
  getServices,
  getCharacteristics,
  writeCommand,
  enableNotify,
  findUuid
}
