import Vue from 'vue'
import Icon from 'vue-awesome/components/Icon'
import 'vue-awesome/icons'

/* eslint-disable no-unused-vars */
import $ from 'jquery'
import 'bootstrap/dist/css/bootstrap.css'
import 'bootstrap-vue/dist/bootstrap-vue.css'

import { BContainer, BCol, BRow, BProgress, BProgressBar } from 'bootstrap-vue'

import App from './App'

Vue.component('v-icon', Icon)
Vue.component('b-container', BContainer)
Vue.component('b-col', BCol)
Vue.component('b-row', BRow)
Vue.component('b-progress', BProgress)
Vue.component('b-progress-bar', BProgressBar)

if (!process.env.IS_WEB) Vue.use(require('vue-electron'))
Vue.config.productionTip = false

/* eslint-disable no-new */
new Vue({
  components: { App },
  template: '<App/>'
}).$mount('#app')
