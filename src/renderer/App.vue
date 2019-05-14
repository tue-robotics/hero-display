<template>
  <div id="app">
    <vue-typer :text='text' :erase-on-complete='true' :pre-erase-delay='2000' :repeat='0'></vue-typer>
  </div>
</template>

<script>
  import {remote} from 'electron'
  import { VueTyper } from 'vue-typer'
  import AutoRos from './services/ros'
  import ROSLIB from 'roslib'

  export default {
    name: 'hero-display',
    components: {
      VueTyper
    },
    data () {
      return {
        topic: new ROSLIB.Topic({
          ros: AutoRos.ros,
          name: 'string',
          messageType: 'std_msgs/String'
        }),
        text: ''
      }
    },
    mounted () {
      remote.getCurrentWindow().setFullScreen(true)
      AutoRos.connect('ws://localhost:9090')
      this.topic.subscribe((msg) => {
        this.text = msg.data
      })
    }
  }
</script>

<style>
.vue-typer {
  text-align: center;
  font-size: 140px;
}
</style>
