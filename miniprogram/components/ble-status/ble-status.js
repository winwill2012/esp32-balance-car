const session = require('../../utils/bleSession.js')

Component({
  data: {
    scanning: false,
    connecting: false,
    connected: false,
    statusText: '未连接',
    deviceName: ''
  },

  lifetimes: {
    attached() {
      this._unsub = session.subscribe((s) => {
        this.setData({
          scanning: s.scanning,
          connecting: s.connecting,
          connected: s.connected,
          statusText: s.statusText,
          deviceName: s.deviceName
        })
      })
    },
    detached() {
      if (this._unsub) {
        this._unsub()
        this._unsub = null
      }
    }
  },

  methods: {
    onScanConnect() {
      session.scanConnect()
    },
    onDisconnect() {
      session.disconnect()
    }
  }
})
