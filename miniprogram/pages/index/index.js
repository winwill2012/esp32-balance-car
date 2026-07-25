const session = require('../../utils/bleSession.js')

const PARAM_LIMITS = {
  a0: { min: -5, max: 5, digits: 2 },
  kp: { min: 0, max: 200, digits: 2 },
  kd: { min: 0, max: 20, digits: 2 },
  kv: { min: 0, max: 2, digits: 4 }
}

const PARAM_DEFAULTS = {
  a0: 0,
  kp: 25,
  kd: 0.5,
  kv: 0.7
}

Page({
  data: {
    connected: false,
    a0: PARAM_DEFAULTS.a0,
    kp: PARAM_DEFAULTS.kp,
    kd: PARAM_DEFAULTS.kd,
    kv: PARAM_DEFAULTS.kv,
    angleText: '--',
    batteryText: '--',
    pwmText: '--',
    leftDzText: '--',
    rightDzText: '--',
    lastMsg: '',
    calibrating: false,
    calPhase: '',
    calCount: 0,
    calTip: ''
  },

  _unsub: null,
  _applyTimer: null,
  _calDone: false,
  _dzDone: false,
  _lastCalDoneSeq: 0,
  _lastDzDoneSeq: 0,

  onShow() {
    if (this._unsub) return
    this._unsub = session.subscribe((s) => {
      this.setData({
        connected: s.connected,
        a0: s.a0,
        kp: s.kp,
        kd: s.kd,
        kv: s.kv,
        angleText: s.angleText,
        batteryText: s.batteryText,
        pwmText: s.pwmText,
        leftDzText: s.leftDzText,
        rightDzText: s.rightDzText,
        lastMsg: s.lastMsg
      })
      if (s.calDoneSeq && s.calDoneSeq !== this._lastCalDoneSeq) {
        this._lastCalDoneSeq = s.calDoneSeq
        this._calDone = true
      }
      if (s.dzDoneSeq && s.dzDoneSeq !== this._lastDzDoneSeq) {
        this._lastDzDoneSeq = s.dzDoneSeq
        this._dzDone = true
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

  onKvInput(e) {
    this.updateParamFromInput('kv', e.detail.value)
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

  onKvChanging(e) {
    session.updateParams({ kv: Number(e.detail.value) })
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
    const kd = session.round(Number(s.kd), 2)
    const kv = session.round(Number(s.kv), 4)
    session.setSuppressStatus(true)
    try {
      await session.sendCmd(`SET=${kp},${kd},${kv},${a0}`)
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

  async onDeadZoneDetect() {
    if (!this.data.connected || this.data.calibrating) return

    this._dzDone = false
    this.setData({
      calibrating: true,
      calPhase: 'prepare',
      calTip: '请让小车轮子悬空',
      calCount: 3
    })

    for (let i = 3; i >= 1; i--) {
      if (!this.data.calibrating) return
      this.setData({ calCount: i })
      await this.sleep(1000)
      if (!this.data.connected) {
        this.finishBusy(false, '已断开，检测中断')
        return
      }
    }

    this.setData({
      calPhase: 'running',
      calTip: '检测中，请保持轮子悬空',
      calCount: 30
    })

    try {
      await session.sendCmd('DZCAL')
    } catch (err) {
      this.finishBusy(false, '检测指令发送失败')
      return
    }

    for (let i = 30; i >= 1; i--) {
      if (!this.data.calibrating) return
      this.setData({ calCount: i })
      if (this._dzDone) break
      await this.sleep(1000)
      if (!this.data.connected) {
        this.finishBusy(false, '已断开，检测中断')
        return
      }
    }

    const waitUntil = Date.now() + 5000
    while (!this._dzDone && Date.now() < waitUntil) {
      if (!this.data.calibrating) return
      await this.sleep(200)
      if (!this.data.connected) {
        this.finishBusy(false, '已断开，检测中断')
        return
      }
    }

    if (this._dzDone) {
      const s = session.getState()
      this.setData({
        calPhase: 'done',
        calTip: '检测完成',
        calCount: 0,
        leftDzText: s.leftDzText,
        rightDzText: s.rightDzText
      })
      await this.sleep(600)
      this.finishBusy(
        true,
        '',
        '死区检测完成',
        `左电机死区 PWM：${s.leftDzText}\n右电机死区 PWM：${s.rightDzText}\n已自动保存到芯片。`
      )
    } else {
      this.finishBusy(false, '检测超时，请重试')
    }
  },

  clamp(v, min, max) {
    return Math.min(max, Math.max(min, v))
  }
})
