import Vue from 'vue'

import App from './App.vue'

import { Battery } from 'hero-vue'

Vue.component('Battery', Battery)

new Vue({
  render: h => h(App)
}).$mount('#app')
