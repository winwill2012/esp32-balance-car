const session = require('../../utils/bleSession.js')

const SPEED_MAX_KEY = 'control.speedMax'
// 速度单位是编码器脉冲，与 LEDC 位宽无关
const SPEED_MAX_DEFAULT = 20
const SPEED_MAX_LIMIT = { min: 0, max: 25 }

const TURN_MAX_KEY = 'control.turnMax'
// 转向设置和 TRN 指令均直接使用目标偏航角速度，单位为 °/s。
const TURN_MAX_DEFAULT = 200
const TURN_MAX_LIMIT = { min: 0, max: 300 }

Page({
  data: {
    connected: false,
    leftSpeedText: '--',
    angleText: '--',
    batteryText: '--',
    rightSpeedText: '--',
    speedMax: SPEED_MAX_DEFAULT,
    turnMax: TURN_MAX_DEFAULT
  },

  _unsub: null,
  _speedCmd: 0,
  _turnCmd: 0,
  _speedNorm: 0,
  _turnNorm: 0,
  _sendTimer: null,
  _heartbeatTimer: null,
  _flushInFlight: false,
  _lastSentSpeed: null,
  _lastSentTurn: null,

  onLoad() {
    this.setData({
      speedMax: this.loadNumber(SPEED_MAX_KEY, SPEED_MAX_DEFAULT, SPEED_MAX_LIMIT),
      turnMax: this.loadNumber(TURN_MAX_KEY, TURN_MAX_DEFAULT, TURN_MAX_LIMIT)
    })
  },

  onShow() {
    this.startMotionHeartbeat()
    if (this._unsub) return
    this._unsub = session.subscribe((s) => {
      const wasConnected = this.data.connected
      this.setData({
        connected: s.connected,
        leftSpeedText: s.leftSpeedText,
        angleText: s.angleText,
        batteryText: s.batteryText,
        rightSpeedText: s.rightSpeedText
      })
      if (wasConnected && !s.connected) {
        this.resetMotion()
      }
    })
  },

  onHide() {
    this.stopMotionHeartbeat()
    this.resetMotion(true)
  },

  onUnload() {
    this.stopMotionHeartbeat()
    this.resetMotion(true)
    if (this._unsub) {
      this._unsub()
      this._unsub = null
    }
  },

  loadNumber(key, def, limit) {
    try {
      const saved = Number(wx.getStorageSync(key))
      if (!Number.isNaN(saved) && saved > 0) {
        return this.clamp(saved, limit.min, limit.max)
      }
    } catch (e) {
      console.warn('load setting failed', key, e)
    }
    return def
  },

  saveNumber(key, v) {
    try {
      wx.setStorageSync(key, v)
    } catch (e) {
      console.warn('save setting failed', key, e)
    }
  },

  applySpeedMax(raw) {
    const next = this.round(
      this.clamp(Number(raw), SPEED_MAX_LIMIT.min, SPEED_MAX_LIMIT.max),
      1
    )
    this.setData({ speedMax: next })
    this.saveNumber(SPEED_MAX_KEY, next)
    if (this._speedNorm !== 0) {
      this.applySpeedNorm(this._speedNorm)
    }
  },

  applyTurnMax(raw) {
    const next = this.round(
      this.clamp(Number(raw), TURN_MAX_LIMIT.min, TURN_MAX_LIMIT.max),
      1
    )
    this.setData({ turnMax: next })
    this.saveNumber(TURN_MAX_KEY, next)
    if (this._turnNorm !== 0) {
      this.applyTurnNorm(this._turnNorm)
    }
  },

  onSpeedMaxChanging(e) {
    this.setData({ speedMax: Number(e.detail.value) })
  },

  onSpeedMaxChange(e) {
    this.applySpeedMax(e.detail.value)
  },

  onSpeedMaxInput(e) {
    const v = parseFloat(e.detail.value)
    if (Number.isNaN(v)) return
    this.applySpeedMax(v)
  },

  onSpeedMaxNudge(e) {
    const delta = parseFloat(e.currentTarget.dataset.delta)
    if (Number.isNaN(delta)) return
    this.applySpeedMax(Number(this.data.speedMax) + delta)
  },

  onSpeedMaxReset() {
    this.applySpeedMax(SPEED_MAX_DEFAULT)
  },

  onSettingHelp(e) {
    const tip = e.currentTarget.dataset.tip
    if (!tip) return
    wx.showModal({
      title: '说明',
      content: String(tip),
      showCancel: false,
      confirmText: '知道了'
    })
  },

  onTurnMaxChanging(e) {
    this.setData({ turnMax: Number(e.detail.value) })
  },

  onTurnMaxChange(e) {
    this.applyTurnMax(e.detail.value)
  },

  onTurnMaxInput(e) {
    const v = parseFloat(e.detail.value)
    if (Number.isNaN(v)) return
    this.applyTurnMax(v)
  },

  onTurnMaxNudge(e) {
    const delta = parseFloat(e.currentTarget.dataset.delta)
    if (Number.isNaN(delta)) return
    this.applyTurnMax(Number(this.data.turnMax) + delta)
  },

  onTurnMaxReset() {
    this.applyTurnMax(TURN_MAX_DEFAULT)
  },

  onSpeedStick(e) {
    if (!this.data.connected) return
    // 屏幕向上为负 y，映射为前进（正 targetSpeed）
    const y = Number(e.detail.y) || 0
    this.applySpeedNorm(-y)
  },

  onTurnStick(e) {
    if (!this.data.connected) return
    // 右推为正目标偏航角速度（右转）
    const x = Number(e.detail.x) || 0
    this.applyTurnNorm(x)
  },

  applySpeedNorm(norm) {
    const n = this.clamp(Number(norm) || 0, -1, 1)
    this._speedNorm = n
    const speed = this.round(n * Number(this.data.speedMax), 3)
    this._speedCmd = speed
    this.scheduleSend()
  },

  applyTurnNorm(norm) {
    const n = this.clamp(Number(norm) || 0, -1, 1)
    this._turnNorm = n
    const turn = this.round(n * Number(this.data.turnMax), 3)
    this._turnCmd = turn
    this.scheduleSend()
  },

  scheduleSend() {
    if (this._sendTimer) return
    this._sendTimer = setTimeout(() => {
      this._sendTimer = null
      this.flushSend()
    }, 50)
  },

  startMotionHeartbeat() {
    if (this._heartbeatTimer) return
    this._heartbeatTimer = setInterval(() => {
      if (this.data.connected) {
        this.flushSend(true)
      }
    }, 150)
  },

  stopMotionHeartbeat() {
    if (!this._heartbeatTimer) return
    clearInterval(this._heartbeatTimer)
    this._heartbeatTimer = null
  },

  async flushSend(force) {
    if (!this.data.connected || this._flushInFlight) return

    const speed = this._speedCmd
    const turn = this._turnCmd
    const commands = []

    if (force || this._lastSentSpeed === null || Math.abs(this._lastSentSpeed - speed) >= 0.01) {
      this._lastSentSpeed = speed
      commands.push(`SPD=${speed.toFixed(3)}`)
    }

    if (force || this._lastSentTurn === null || Math.abs(this._lastSentTurn - turn) >= 0.01) {
      this._lastSentTurn = turn
      commands.push(`TRN=${turn.toFixed(3)}`)
    }

    if (!commands.length) return

    this._flushInFlight = true
    try {
      // 微信 BLE 在部分手机上不允许并发写特征值，心跳命令必须依次发送。
      for (const command of commands) {
        await session.sendCmd(command)
      }
    } catch (err) {
      console.error('motion command send failed', err)
    } finally {
      this._flushInFlight = false
    }
  },

  resetMotion(sendZero) {
    this._speedCmd = 0
    this._turnCmd = 0
    this._speedNorm = 0
    this._turnNorm = 0
    this._lastSentSpeed = null
    this._lastSentTurn = null
    if (this._sendTimer) {
      clearTimeout(this._sendTimer)
      this._sendTimer = null
    }
    if (sendZero && this.data.connected) {
      Promise.all([
        session.sendCmd('SPD=0'),
        session.sendCmd('TRN=0')
      ]).catch(() => {})
    }
  },

  clamp(v, min, max) {
    return Math.min(max, Math.max(min, v))
  },

  round(v, n) {
    const p = Math.pow(10, n)
    return Math.round(v * p) / p
  }
})
