const session = require('../../utils/bleSession.js')

const PARAM_LIMITS = {
  kp: { min: 0, max: 200, digits: 2 },
  kd: { min: 0, max: 20, digits: 2 },
  kv: { min: 0, max: 2, digits: 4 }
}

Page({
  data: {
    connected: false,
    kp: 25,
    kd: 0.5,
    kv: 0.02,
    angleText: '--',
    batteryText: '--',
    pwmText: '--',
    lastMsg: ''
  },

  _unsub: null,
  _applyTimer: null,

  onShow() {
    if (this._unsub) return
    this._unsub = session.subscribe((s) => {
      this.setData({
        connected: s.connected,
        kp: s.kp,
        kd: s.kd,
        kv: s.kv,
        angleText: s.angleText,
        batteryText: s.batteryText,
        pwmText: s.pwmText,
        lastMsg: s.lastMsg
      })
    })
  },

  onHide() {
    this.clearApplyTimer()
  },

  onUnload() {
    this.clearApplyTimer()
    if (this._unsub) {
      this._unsub()
      this._unsub = null
    }
  },

  clearApplyTimer() {
    if (this._applyTimer) {
      clearTimeout(this._applyTimer)
      this._applyTimer = null
    }
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

  onKpChanging(e) {
    session.updateParams({ kp: Number(e.detail.value) })
  },

  onKdChanging(e) {
    session.updateParams({ kd: Number(e.detail.value) })
  },

  onKvChanging(e) {
    session.updateParams({ kv: Number(e.detail.value) })
  },

  onKpChange(e) {
    session.updateParams({ kp: Number(e.detail.value) })
    this.scheduleApply()
  },

  onKdChange(e) {
    session.updateParams({ kd: Number(e.detail.value) })
    this.scheduleApply()
  },

  onKvChange(e) {
    session.updateParams({ kv: Number(e.detail.value) })
    this.scheduleApply()
  },

  updateParamFromInput(param, raw) {
    const v = parseFloat(raw)
    if (Number.isNaN(v)) return
    const lim = PARAM_LIMITS[param]
    session.updateParams({ [param]: this.clamp(v, lim.min, lim.max) })
    this.scheduleApply()
  },

  onNudge(e) {
    if (!this.data.connected) return
    const param = e.currentTarget.dataset.param
    const delta = parseFloat(e.currentTarget.dataset.delta)
    const lim = PARAM_LIMITS[param]
    if (!lim || Number.isNaN(delta)) return

    const next = session.round(
      this.clamp(Number(this.data[param]) + delta, lim.min, lim.max),
      lim.digits
    )
    session.updateParams({ [param]: next })
    this.scheduleApply()
  },

  scheduleApply() {
    if (!this.data.connected) return
    this.clearApplyTimer()
    this._applyTimer = setTimeout(() => {
      this._applyTimer = null
      this.onApply()
    }, 180)
  },

  async onApply() {
    if (!this.data.connected) return
    const s = session.getState()
    const kp = session.round(Number(s.kp), 2)
    const kd = session.round(Number(s.kd), 2)
    const kv = session.round(Number(s.kv), 4)
    session.setSuppressStatus(true)
    try {
      await session.sendCmd(`SET=${kp},${kd},${kv}`)
    } catch (err) {
      wx.showToast({ title: '下发失败', icon: 'none' })
    }
    setTimeout(() => {
      session.setSuppressStatus(false)
    }, 500)
  },

  async onSave() {
    if (!this.data.connected) return
    try {
      await session.sendCmd('SAVE')
      wx.showToast({ title: '已持久化到芯片', icon: 'success' })
    } catch (err) {
      wx.showToast({ title: '保存失败', icon: 'none' })
    }
  },

  clamp(v, min, max) {
    return Math.min(max, Math.max(min, v))
  }
})
