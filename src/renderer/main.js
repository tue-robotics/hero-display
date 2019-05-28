import Vue from 'vue'
import Icon from 'vue-awesome/components/Icon'
import 'vue-awesome/icons'

import App from './App'

Vue.component('v-icon', Icon)

if (!process.env.IS_WEB) Vue.use(require('vue-electron'))
Vue.config.productionTip = false

/* eslint-disable no-new */
new Vue({
  components: { App },
  template: '<App/>'
}).$mount('#app')
