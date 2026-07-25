Component({
  properties: {
    // y: 仅前后; x: 仅左右; both: 全向
    axis: {
      type: String,
      value: 'y'
    },
    label: {
      type: String,
      value: ''
    },
    disabled: {
      type: Boolean,
      value: false
    }
  },

  data: {
    active: false,
    dx: 0,
    dy: 0
  },

  lifetimes: {
    ready() {
      this._radius = 0
      this._centerX = 0
      this._centerY = 0
      this._nx = 0
      this._ny = 0
      this._touchId = null
      this.measurePad()
    }
  },

  methods: {
    measurePad() {
      return new Promise((resolve) => {
        const q = this.createSelectorQuery()
        q.select('.joy-pad').boundingClientRect((rect) => {
          if (!rect) {
            resolve(false)
            return
          }
          this._radius = Math.min(rect.width, rect.height) / 2
          this._centerX = rect.left + rect.width / 2
          this._centerY = rect.top + rect.height / 2
          resolve(true)
        }).exec()
      })
    },

    async ensureMeasured() {
      if (this._radius > 0) return true
      return this.measurePad()
    },

    findTouch(list) {
      if (!list || this._touchId === null || this._touchId === undefined) {
        return null
      }
      for (let i = 0; i < list.length; i++) {
        if (list[i].identifier === this._touchId) {
          return list[i]
        }
      }
      return null
    },

    async onTouchStart(e) {
      if (this.properties.disabled) return
      // 同一摇杆只跟踪一根手指，避免串点
      if (this._touchId !== null) return

      const touch = (e.changedTouches && e.changedTouches[0]) || (e.touches && e.touches[0])
      if (!touch) return

      this._touchId = touch.identifier
      await this.ensureMeasured()
      await this.measurePad()
      this.setData({ active: true })
      this.updateFromTouch(touch)
    },

    onTouchMove(e) {
      if (this.properties.disabled || !this.data.active) return
      const touch = this.findTouch(e.touches) || this.findTouch(e.changedTouches)
      if (!touch) return
      this.updateFromTouch(touch)
    },

    onTouchEnd(e) {
      if (this.properties.disabled) return
      if (this._touchId === null) return

      // 只有当前绑定的手指抬起时才回中
      const ended = this.findTouch(e.changedTouches)
      if (!ended) {
        // 若当前触点已不在 touches 中，也视为结束
        if (this.findTouch(e.touches)) return
      }

      this.releaseStick()
    },

    releaseStick() {
      this._touchId = null
      this._nx = 0
      this._ny = 0
      this.setData({
        active: false,
        dx: 0,
        dy: 0
      })
      this.emitChange(0, 0)
    },

    updateFromTouch(touch) {
      if (!touch || !this._radius) return

      let dx = touch.clientX - this._centerX
      let dy = touch.clientY - this._centerY
      const axis = this.properties.axis

      if (axis === 'y') dx = 0
      if (axis === 'x') dy = 0

      const dist = Math.sqrt(dx * dx + dy * dy)
      const maxR = this._radius * 0.62
      if (dist > maxR && dist > 0) {
        const s = maxR / dist
        dx *= s
        dy *= s
      }

      const nx = maxR > 0 ? dx / maxR : 0
      const ny = maxR > 0 ? dy / maxR : 0
      this._nx = nx
      this._ny = ny

      this.setData({ dx, dy })
      this.emitChange(nx, ny)
    },

    emitChange(x, y) {
      this.triggerEvent('change', { x, y })
    }
  }
})
