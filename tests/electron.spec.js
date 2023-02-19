import { testWithPlaywright } from 'vue-cli-plugin-electron-builder'
import { expect, test } from '@playwright/test'
test.setTimeout(60000)

test.describe('Window Loads Properly', async () => {
  let app, stop, win, browserWindow

  test.beforeAll(async () => {
    // Wait for dev server to start
    const { app: _app, stop: _stop } = await testWithPlaywright({ launchOptions: { args: ['dist_electron/index.js', 'random_host.local'] } })
    app = _app
    stop = _stop
    win = await app.firstWindow()
    browserWindow = await app.browserWindow(win)
  })

  test.afterAll(async () => {
    if (app) {
      return await stop()
    }
  })

  test('Window count is one', () => {
    expect(app.windows().length).toBe(1)
  })

  test('Window is not minimized', async () => {
    const isMinimized = await browserWindow.evaluate((browserWindow) => { return browserWindow.isMinimized() })
    expect(isMinimized).toBe(false)
  })

  test('Window is visible', async () => {
    const isVisible = await browserWindow.evaluate((browserWindow) => { return browserWindow.isVisible() })
    expect(isVisible).toBe(true)
  })

  test('Window bounds are correct', async () => {
    const { height, width } = await browserWindow.evaluate((browserWindow) => { return { ...browserWindow.getBounds() } })
    expect(width).toBe(1024)
    expect(height).toBe(600)
  })

  test('Window title is correct', async () => {
    expect(await win.title()).toBe('hero-display')
  })

  test('Argument passing works', async () => {
    const argv = await app.evaluate(async () => process.argv)
    expect(argv).toContain('random_host.local')
  })
})
