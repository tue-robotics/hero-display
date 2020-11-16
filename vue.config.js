module.exports = {
  lintOnSave: true,
  pluginOptions: {
    electronBuilder: {
      builderOptions: {
        productName: 'hero-display',
        appId: 'com.hero-display.display-app',
        // eslint-disable-next-line no-template-curly-in-string
        artifactName: 'hero-display.${ext}',
        linux: {
          category: 'Utility',
          target: 'AppImage'
        }
      },
      nodeIntegration: true
    }
  }
}
