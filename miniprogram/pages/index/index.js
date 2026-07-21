const ble = require('../../utils/ble.js')

const PARAM_LIMITS = {
  kp: { min: 0, max: 200, digits: 2 },
  kd: { min: 0, max: 20, digits: 2 },
  kv: { min: 0, max: 2, digits: 4 },
  maxLean: { min: 0, max: 60, digits: 2 }
}

Page({
  data: {
    scanning: false,
    connecting: false,
    connected: false,
    statusText: '未连接',
    deviceId: '',
    deviceName: '',
    serviceId: '',
    cmdCharId: '',
    statusCharId: '',
    kp: 25,
    kd: 0.5,
    kv: 0.02,
    maxLean: 8,
    angleText: '--',
    targetAngleText: '--',
    pwmText: '--',
    lastMsg: ''
  },

  _foundHandler: null,
  _valueHandler: null,
  _connHandler: null,
  _scanTimer: null,
  _applyTimer: null,
  _suppressStatus: false,

  onUnload() {
    this.cleanup()
  },

  async onScanConnect() {
    if (this.data.connected || this.data.scanning || this.data.connecting) return

    this.setData({
      scanning: true,
      statusText: '扫描中…',
      lastMsg: ''
    })

    try {
      await ble.openAdapter()
      this.bindConnectionListener()
      await ble.startDiscovery()
      this.waitForDevice()
    } catch (err) {
      console.error(err)
      this.setData({
        scanning: false,
        statusText: '蓝牙不可用',
        lastMsg: (err && err.errMsg) || '请打开手机蓝牙并授权'
      })
      wx.showToast({ title: '请打开蓝牙', icon: 'none' })
    }
  },

  waitForDevice() {
    if (this._foundHandler) {
      wx.offBluetoothDeviceFound(this._foundHandler)
    }

    this._foundHandler = (res) => {
      const devices = res.devices || []
      const target = devices.find((d) => {
        const name = d.name || d.localName || ''
        return name === ble.DEVICE_NAME || name.indexOf('ESP32-Car') >= 0
      })
      if (!target) return

      if (this._scanTimer) {
        clearTimeout(this._scanTimer)
        this._scanTimer = null
      }
      wx.offBluetoothDeviceFound(this._foundHandler)
      this._foundHandler = null
      this.connectDevice(target)
    }

    wx.onBluetoothDeviceFound(this._foundHandler)

    this._scanTimer = setTimeout(() => {
      this._scanTimer = null
      if (this._foundHandler) {
        wx.offBluetoothDeviceFound(this._foundHandler)
        this._foundHandler = null
      }
      ble.stopDiscovery()
      this.setData({
        scanning: false,
        statusText: '未找到设备',
        lastMsg: '确认 ESP32 已上电并广播 ESP32-Car'
      })
    }, 12000)
  },

  async connectDevice(device) {
    this.setData({
      scanning: false,
      connecting: true,
      statusText: '连接中…',
      deviceName: device.name || device.localName || ble.DEVICE_NAME
    })

    try {
      await ble.stopDiscovery()
      await ble.createConnection(device.deviceId)

      // Android 上服务发现偶发延迟
      await this.sleep(400)

      // 提高 MTU，保证遥测 ANG/TA/PWM 与参数应答不被截断
      try {
        await ble.setMtu(device.deviceId, 128)
      } catch (e) {
        console.warn('setBLEMTU failed', e)
      }

      const services = await ble.getServices(device.deviceId)
      const service = ble.findUuid(services, ble.SERVICE_UUID)
      if (!service) {
        throw new Error('未找到 FFF0 服务')
      }

      const chars = await ble.getCharacteristics(device.deviceId, service.uuid)
      const cmdChar = ble.findUuid(chars, ble.CHAR_CMD_UUID)
      const statusChar = ble.findUuid(chars, ble.CHAR_STATUS_UUID)
      if (!cmdChar || !statusChar) {
        throw new Error('未找到指令/状态特征')
      }

      this.setData({
        connected: true,
        connecting: false,
        statusText: '已连接',
        deviceId: device.deviceId,
        serviceId: service.uuid,
        cmdCharId: cmdChar.uuid,
        statusCharId: statusChar.uuid
      })

      this.bindValueListener()
      await ble.enableNotify(device.deviceId, service.uuid, statusChar.uuid)
      await this.sleep(200)
      await this.sendCmd('GET')
      wx.showToast({ title: '连接成功', icon: 'success' })
    } catch (err) {
      console.error(err)
      await ble.closeConnection(device.deviceId)
      this.setData({
        connecting: false,
        connected: false,
        statusText: '连接失败',
        deviceId: '',
        lastMsg: (err && (err.message || err.errMsg)) || '连接失败'
      })
      wx.showToast({ title: '连接失败', icon: 'none' })
    }
  },

  bindConnectionListener() {
    if (this._connHandler) {
      wx.offBLEConnectionStateChange(this._connHandler)
    }
    this._connHandler = (res) => {
      if (!res.connected && res.deviceId === this.data.deviceId) {
        this.setData({
          connected: false,
          statusText: '已断开',
          angleText: '--',
          targetAngleText: '--',
          pwmText: '--'
        })
      }
    }
    wx.onBLEConnectionStateChange(this._connHandler)
  },

  bindValueListener() {
    if (this._valueHandler) {
      wx.offBLECharacteristicValueChange(this._valueHandler)
    }
    this._valueHandler = (res) => {
      if (res.deviceId !== this.data.deviceId) return
      const text = ble.ab2str(res.value)
      const parsed = ble.parseStatus(text)
      const patch = { lastMsg: text }

      // 含 ANG 的是周期遥测，不回写调参项，避免打断输入/滑条
      const isTelemetry = parsed.ANG !== undefined
      if (!isTelemetry && !this._suppressStatus) {
        if (parsed.KP !== undefined) patch.kp = this.round(parsed.KP, 2)
        if (parsed.KD !== undefined) patch.kd = this.round(parsed.KD, 2)
        if (parsed.KV !== undefined) patch.kv = this.round(parsed.KV, 4)
        if (parsed.ML !== undefined) patch.maxLean = this.round(parsed.ML, 2)
      }
      if (parsed.ANG !== undefined) patch.angleText = parsed.ANG.toFixed(2)
      if (parsed.TA !== undefined) patch.targetAngleText = parsed.TA.toFixed(2)
      if (parsed.PWM !== undefined) patch.pwmText = parsed.PWM.toFixed(1)

      this.setData(patch)
    }
    wx.onBLECharacteristicValueChange(this._valueHandler)
  },

  async onDisconnect() {
    await this.cleanup()
    this.setData({
      connected: false,
      connecting: false,
      scanning: false,
      statusText: '未连接',
      deviceId: '',
      deviceName: '',
      angleText: '--',
      targetAngleText: '--',
      pwmText: '--',
      lastMsg: ''
    })
  },

  onKpInput(e) {
    this.updateParamFromInput('kp', e.detail.value)
  },

  onKdInput(e) {
    this.updateParamFromInput('kd', e.detail.value)
  },

  onKvInput(e) {
    this.updateParamFromInput('kv', e.detail.value)
  },

  onMaxLeanInput(e) {
    this.updateParamFromInput('maxLean', e.detail.value)
  },

  onKpChanging(e) {
    this.setData({ kp: Number(e.detail.value) })
  },

  onKdChanging(e) {
    this.setData({ kd: Number(e.detail.value) })
  },

  onKvChanging(e) {
    this.setData({ kv: Number(e.detail.value) })
  },

  onMaxLeanChanging(e) {
    this.setData({ maxLean: Number(e.detail.value) })
  },

  onKpChange(e) {
    this.setData({ kp: Number(e.detail.value) })
    this.scheduleApply()
  },

  onKdChange(e) {
    this.setData({ kd: Number(e.detail.value) })
    this.scheduleApply()
  },

  onKvChange(e) {
    this.setData({ kv: Number(e.detail.value) })
    this.scheduleApply()
  },

  onMaxLeanChange(e) {
    this.setData({ maxLean: Number(e.detail.value) })
    this.scheduleApply()
  },

  updateParamFromInput(param, raw) {
    const v = parseFloat(raw)
    if (Number.isNaN(v)) return
    const lim = PARAM_LIMITS[param]
    this.setData({ [param]: this.clamp(v, lim.min, lim.max) })
  },

  onNudge(e) {
    if (!this.data.connected) return
    const param = e.currentTarget.dataset.param
    const delta = parseFloat(e.currentTarget.dataset.delta)
    const lim = PARAM_LIMITS[param]
    if (!lim || Number.isNaN(delta)) return

    const next = this.round(
      this.clamp(Number(this.data[param]) + delta, lim.min, lim.max),
      lim.digits
    )
    this.setData({ [param]: next })
    this.scheduleApply()
  },

  onPreset(e) {
    if (!this.data.connected) return
    const kp = this.round(this.clamp(parseFloat(e.currentTarget.dataset.kp), 0, 200), 2)
    const kd = this.round(this.clamp(parseFloat(e.currentTarget.dataset.kd), 0, 20), 2)
    if (Number.isNaN(kp) || Number.isNaN(kd)) return
    this.setData({ kp, kd })
    this.scheduleApply()
  },

  scheduleApply() {
    if (!this.data.connected) return
    if (this._applyTimer) clearTimeout(this._applyTimer)
    this._applyTimer = setTimeout(() => {
      this._applyTimer = null
      this.onApply()
    }, 180)
  },

  async onApply() {
    if (!this.data.connected) return
    const kp = this.round(Number(this.data.kp), 2)
    const kd = this.round(Number(this.data.kd), 2)
    const kv = this.round(Number(this.data.kv), 4)
    const ml = this.round(Number(this.data.maxLean), 2)
    this._suppressStatus = true
    try {
      await this.sendCmd(`SET=${kp},${kd},${kv},${ml}`)
      wx.showToast({ title: '已下发', icon: 'success', duration: 800 })
    } catch (err) {
      wx.showToast({ title: '下发失败', icon: 'none' })
    }
    setTimeout(() => {
      this._suppressStatus = false
    }, 500)
  },

  async onSave() {
    if (!this.data.connected) return
    try {
      await this.sendCmd('SAVE')
      wx.showToast({ title: '已保存到板端', icon: 'success' })
    } catch (err) {
      wx.showToast({ title: '保存失败', icon: 'none' })
    }
  },

  async sendCmd(cmd) {
    const { deviceId, serviceId, cmdCharId } = this.data
    if (!deviceId || !serviceId || !cmdCharId) {
      throw new Error('未连接')
    }
    await ble.writeCommand(deviceId, serviceId, cmdCharId, cmd)
  },

  async cleanup() {
    if (this._scanTimer) {
      clearTimeout(this._scanTimer)
      this._scanTimer = null
    }
    if (this._applyTimer) {
      clearTimeout(this._applyTimer)
      this._applyTimer = null
    }
    if (this._foundHandler) {
      wx.offBluetoothDeviceFound(this._foundHandler)
      this._foundHandler = null
    }
    if (this._valueHandler) {
      wx.offBLECharacteristicValueChange(this._valueHandler)
      this._valueHandler = null
    }
    if (this._connHandler) {
      wx.offBLEConnectionStateChange(this._connHandler)
      this._connHandler = null
    }
    await ble.stopDiscovery()
    await ble.closeConnection(this.data.deviceId)
    await ble.closeAdapter()
  },

  clamp(v, min, max) {
    return Math.min(max, Math.max(min, v))
  },

  round(v, n) {
    const p = Math.pow(10, n)
    return Math.round(v * p) / p
  },

  sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms))
  }
})
