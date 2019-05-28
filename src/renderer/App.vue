<template>
  <div id="app">
    <div v-if="hmiGoalActive" id="hmiGoalActive">
      <v-icon name="spinner" pulse scale="5" />
    </div>
    <div v-else>
      <vue-typer :text='text' :erase-on-complete='true' :pre-erase-delay='2000' :repeat='0' @completed='text = " "'></vue-typer>
      <img v-if="imageSrc" :src="`${imageSrc}`" />
    </div>
  </div>
</template>

<script>
  import { remote } from 'electron'
  import { VueTyper } from 'vue-typer'
  import AutoRos from './services/ros'
  import ROSLIB from 'roslib'
  import jpeg from 'jpeg-js'

  function imageToBase64JpegString (msg) {
    var raw = atob(msg.data)
    var array = new Uint8Array(new ArrayBuffer(raw.length))
    for (let i = 0; i < raw.length; i++) {
      array[i] = raw.charCodeAt(i)
    }

    var frameData = Buffer.alloc(msg.width * msg.height * 4)
    for (let i = 0; i < msg.width * msg.height; i++) {
      if (msg.encoding === 'rgb8') {
        frameData[4 * i + 0] = array[3 * i + 0]
        frameData[4 * i + 2] = array[3 * i + 2]
      } else if (msg.encoding === 'bgr8') {
        frameData[4 * i + 0] = array[3 * i + 2]
        frameData[4 * i + 2] = array[3 * i + 0]
      } else {
        console.error('invalid encoding', msg.encofing)
      }
      frameData[4 * i + 1] = array[3 * i + 1]
      frameData[4 * i + 3] = 0
    }
    var rawImageData = {
      data: frameData,
      width: msg.width,
      height: msg.height
    }
    return 'data:image/jpeg;base64,' + jpeg.encode(rawImageData, 50).data.toString('base64')
  }

  export default {
    name: 'hero-display',
    components: {
      VueTyper
    },
    data () {
      return {
        textTopic: new ROSLIB.Topic({
          ros: AutoRos.ros,
          name: 'text_to_speech/output',
          messageType: 'std_msgs/String'
        }),
        imageTopic: new ROSLIB.Topic({
          ros: AutoRos.ros,
          name: 'image_from_ros',
          messageType: 'sensor_msgs/Image'
        }),
        hmiStatusTopic: new ROSLIB.Topic({
          ros: AutoRos.ros,
          name: 'hmi/status',
          messageType: 'actionlib_msgs/GoalStatusArray'
        }),
        text: ' ',
        imageSrc: null,
        imageShowSeconds: 4,
        hmiGoalActive: false,
        endPoint: 'ws://localhost:9090'
      }
    },
    mounted () {
      remote.getCurrentWindow().setFullScreen(true)
      AutoRos.connect(this.endPoint)
      this.textTopic.subscribe((msg) => {
        this.text = msg.data
      })
      this.imageTopic.subscribe((msg) => {
        this.imageSrc = imageToBase64JpegString(msg)
        setTimeout(() => {
          this.imageSrc = null
        }, this.imageShowSeconds * 1000)
      })
      this.hmiStatusTopic.subscribe((msg) => {
        this.hmiGoalActive = msg.status_list.length > 0
      })
    }
  }
</script>

<style>
#app {
  position: relative;
  text-align: center;
  font-size: 40px;
}
</style>
