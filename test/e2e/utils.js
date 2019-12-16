import electron from 'electron'
import { Application } from 'spectron'

const TIMEOUT = 20000

export default {
  afterEach () {
    this.timeout(TIMEOUT)

    if (this.app && this.app.isRunning()) {
      return this.app.stop()
    }
  },
  beforeEach () {
    this.timeout(TIMEOUT)
    this.app = new Application({
      path: electron,
      args: ['dist/electron/main.js'],
      startTimeout: TIMEOUT,
      waitTimeout: TIMEOUT
    })

    return this.app.start()
  }
}
