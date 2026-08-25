const session = require('../../utils/bleSession.js')

const PARAM_LIMITS = {
  a0: { min: -5, max: 5, digits: 2 },
  // 10 位 PWM（0–1023）约为原 8 位的 4 倍，Kp/Kd 同步放大
  kp: { min: 0, max: 300, digits: 2 },
  ki: { min: 0, max: 5, digits: 4 },
  kd: { min: 0, max: 20, digits: 2 },
  skp: { min: 0, max: 5, digits: 4 },
  ski: { min: 0, max: 5, digits: 4 },
  skd: { min: 0, max: 10, digits: 4 },
  tkp: { min: 0, max: 10, digits: 3 },
  tki: { min: 0, max: 5, digits: 4 },
  tkd: { min: 0, max: 10, digits: 4 }
}

const PARAM_DEFAULTS = {
  a0: 0,
  kp: 90,
  ki: 0,
  kd: 4,
  skp: 0.77,
  ski: 0,
  skd: 0,
  tkp: 2.3,
  tki: 0.2,
  tkd: 0
}

Page({
  data: {
    connected: false,
    a0: PARAM_DEFAULTS.a0,
    kp: PARAM_DEFAULTS.kp,
    ki: PARAM_DEFAULTS.ki,
    kd: PARAM_DEFAULTS.kd,
    skp: PARAM_DEFAULTS.skp,
    ski: PARAM_DEFAULTS.ski,
    skd: PARAM_DEFAULTS.skd,
    tkp: PARAM_DEFAULTS.tkp,
    tki: PARAM_DEFAULTS.tki,
    tkd: PARAM_DEFAULTS.tkd,
    angleText: '--',
    batteryText: '--',
    pwmText: '--',
    outerLoopTab: 'speed',
    lastMsg: '',
    calibrating: false,
    calPhase: '',
    calCount: 0,
    calTip: ''
  },

  _unsub: null,
  _applyTimer: null,
  _calDone: false,
  _lastCalDoneSeq: 0,

  onShow() {
    if (this._unsub) return
    this._unsub = session.subscribe((s) => {
      this.setData({
        connected: s.connected,
        a0: s.a0,
        kp: s.kp,
        ki: s.ki,
        kd: s.kd,
        skp: s.skp,
        ski: s.ski,
        skd: s.skd,
        tkp: s.tkp,
        tki: s.tki,
        tkd: s.tkd,
        angleText: s.angleText,
        batteryText: s.batteryText,
        pwmText: s.pwmText,
        lastMsg: s.lastMsg
      })
      if (s.calDoneSeq && s.calDoneSeq !== this._lastCalDoneSeq) {
        this._lastCalDoneSeq = s.calDoneSeq
        this._calDone = true
      }
      if (!s.connected && this.data.calibrating) {
        this.finishBusy(false, '已断开，操作中断')
      }
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

  sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms))
  },

  onA0Input(e) {
    this.updateParamFromInput('a0', e.detail.value)
  },

  onKpInput(e) {
    this.updateParamFromInput('kp', e.detail.value)
  },

  onKdInput(e) {
    this.updateParamFromInput('kd', e.detail.value)
  },

  onParamInput(e) {
    this.updateParamFromInput(e.currentTarget.dataset.param, e.detail.value)
  },

  onA0Changing(e) {
    session.updateParams({ a0: Number(e.detail.value) })
  },

  onKpChanging(e) {
    session.updateParams({ kp: Number(e.detail.value) })
  },

  onKdChanging(e) {
    session.updateParams({ kd: Number(e.detail.value) })
  },

  onParamChanging(e) {
    const param = e.currentTarget.dataset.param
    session.updateParams({ [param]: Number(e.detail.value) })
  },

  onA0Change(e) {
    session.updateParams({ a0: Number(e.detail.value) })
    this.scheduleApply()
  },

  onKpChange(e) {
    session.updateParams({ kp: Number(e.detail.value) })
    this.scheduleApply()
  },

  onKdChange(e) {
    session.updateParams({ kd: Number(e.detail.value) })
    this.scheduleApply()
  },

  onParamChange(e) {
    const param = e.currentTarget.dataset.param
    session.updateParams({ [param]: Number(e.detail.value) })
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
    if (!this.data.connected || this.data.calibrating) return
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

  onResetParam(e) {
    if (!this.data.connected || this.data.calibrating) return
    const param = e.currentTarget.dataset.param
    if (PARAM_DEFAULTS[param] === undefined) return
    session.updateParams({ [param]: PARAM_DEFAULTS[param] })
    this.scheduleApply()
  },

  onTuneHelp(e) {
    const tip = String(e.currentTarget.dataset.tip || '').replace(/\\n/g, '\n')
    if (!tip) return
    wx.showModal({
      title: '调参说明',
      content: tip,
      showCancel: false,
      confirmText: '知道了'
    })
  },

  onOuterLoopTab(e) {
    const tab = e.currentTarget.dataset.tab
    if (tab !== 'speed' && tab !== 'turn') return
    this.setData({ outerLoopTab: tab })
  },

  scheduleApply() {
    if (!this.data.connected || this.data.calibrating) return
    this.clearApplyTimer()
    this._applyTimer = setTimeout(() => {
      this._applyTimer = null
      this.onApply()
    }, 180)
  },

  async onApply() {
    if (!this.data.connected || this.data.calibrating) return
    const s = session.getState()
    const a0 = session.round(Number(s.a0), 2)
    const kp = session.round(Number(s.kp), 2)
    const ki = session.round(Number(s.ki), 4)
    const kd = session.round(Number(s.kd), 2)
    const skp = session.round(Number(s.skp), 4)
    const ski = session.round(Number(s.ski), 4)
    const skd = session.round(Number(s.skd), 4)
    const tkp = session.round(Number(s.tkp), 4)
    const tki = session.round(Number(s.tki), 4)
    const tkd = session.round(Number(s.tkd), 4)
    session.setSuppressStatus(true)
    try {
      await session.sendCmd(
        `PID=${kp},${ki},${kd},${skp},${ski},${skd},${tkp},${tki},${tkd},${a0}`
      )
    } catch (err) {
      wx.showToast({ title: '下发失败', icon: 'none' })
    }
    setTimeout(() => {
      session.setSuppressStatus(false)
    }, 500)
  },

  async onSave() {
    if (!this.data.connected || this.data.calibrating) return
    try {
      await session.sendCmd('SAVE')
      wx.showToast({ title: '已持久化到芯片', icon: 'success' })
    } catch (err) {
      wx.showToast({ title: '保存失败', icon: 'none' })
    }
  },

  finishBusy(ok, failToast, okTitle, okContent) {
    this.setData({
      calibrating: false,
      calPhase: '',
      calCount: 0,
      calTip: ''
    })
    if (ok) {
      wx.showModal({
        title: okTitle || '完成',
        content: okContent || '操作已完成。',
        showCancel: false,
        confirmText: '知道了'
      })
    } else if (failToast) {
      wx.showToast({ title: failToast, icon: 'none' })
    }
  },

  finishCalib(ok, failToast) {
    this.finishBusy(
      ok,
      failToast,
      '校准完成',
      '请保持静止片刻后继续调参或遥控。'
    )
  },

  async onGyroCalib() {
    if (!this.data.connected || this.data.calibrating) return

    this._calDone = false
    this.setData({
      calibrating: true,
      calPhase: 'prepare',
      calTip: '请保持小车静止',
      calCount: 3
    })

    for (let i = 3; i >= 1; i--) {
      if (!this.data.calibrating) return
      this.setData({ calCount: i })
      await this.sleep(1000)
      if (!this.data.connected) {
        this.finishBusy(false, '已断开，校准中断')
        return
      }
    }

    this.setData({
      calPhase: 'running',
      calTip: '校准中，请保持静止',
      calCount: 5
    })

    try {
      await session.sendCmd('GCAL')
    } catch (err) {
      this.finishBusy(false, '校准指令发送失败')
      return
    }

    for (let i = 5; i >= 1; i--) {
      if (!this.data.calibrating) return
      this.setData({ calCount: i })
      if (this._calDone) break
      await this.sleep(1000)
      if (!this.data.connected) {
        this.finishBusy(false, '已断开，校准中断')
        return
      }
    }

    const waitUntil = Date.now() + 4000
    while (!this._calDone && Date.now() < waitUntil) {
      if (!this.data.calibrating) return
      await this.sleep(200)
      if (!this.data.connected) {
        this.finishBusy(false, '已断开，校准中断')
        return
      }
    }

    if (this._calDone) {
      this.setData({
        calPhase: 'done',
        calTip: '校准完成',
        calCount: 0
      })
      await this.sleep(600)
      this.finishCalib(true)
    } else {
      this.finishBusy(false, '校准超时，请重试')
    }
  },

  clamp(v, min, max) {
    return Math.min(max, Math.max(min, v))
  }
})
