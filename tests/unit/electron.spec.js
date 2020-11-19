import testWithSpectron from 'vue-cli-plugin-electron-builder/lib/testWithSpectron'
import chai from 'chai'
import chaiAsPromised from 'chai-as-promised'
// eslint-disable-next-line no-undef
const spectron = __non_webpack_require__('spectron')

chai.should()
chai.use(chaiAsPromised)

describe('Application launch', function () {
  this.timeout(30000)

  before(function () {
    return testWithSpectron(spectron).then(instance => {
      this.app = instance.app
      this.stopServe = instance.stopServe
    })
  })

  before(function () {
    chaiAsPromised.transferPromiseness = this.app.transferPromiseness
  })

  after(function () {
    if (this.app && this.app.isRunning()) {
      return this.stopServe()
    }
  })

  it('Window count is one', function () {
    return this.app.client.getWindowCount().should.eventually.be.equal(1)
  })

  it('Window is not minimized', function () {
    return this.app.client.browserWindow.isMinimized().should.eventually.be.false
  })

  it('Window is visible', function () {
    return this.app.client.browserWindow.isVisible().should.eventually.be.true
  })

  it('Correct window bounds', function () {
    return Promise.all([
      this.app.client.browserWindow.getBounds()
        .should.eventually.have.property('width').and.be.equal(1024),
      this.app.client.browserWindow.getBounds()
        .should.eventually.have.property('height').and.be.equal(600)
    ])
  })

  it('Correct window title', function () {
    return this.app.client.browserWindow.getTitle().should.eventually.be.equal('hero-display')
  })
})
