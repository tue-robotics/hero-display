import Vue from 'vue'

import { BContainer, BCol, BRow, BProgress, BProgressBar } from 'bootstrap-vue'
import 'bootstrap/dist/css/bootstrap.css'
import 'bootstrap-vue/dist/bootstrap-vue.css'

import Icon from 'vue-awesome/components/Icon'
import 'vue-awesome/icons/bolt'

import App from './App.vue'

Vue.component('v-icon', Icon)
Vue.component('b-container', BContainer)
Vue.component('b-col', BCol)
Vue.component('b-row', BRow)
Vue.component('b-progress', BProgress)
Vue.component('b-progress-bar', BProgressBar)

new Vue({
  render: h => h(App)
}).$mount('#app')
