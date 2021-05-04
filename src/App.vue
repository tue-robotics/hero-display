<template>
  <div id="app">
    <div
      v-if="hmiGoalActive"
      id="hmiGoalActive"
    >
      <div id="mic_spinner">
        <font-awesome-icon
          id="mic"
          icon="microphone"
          class="fa-icon"
        />
        <font-awesome-icon
          id="spinner"
          icon="spinner"
          pulse
          class="fa-icon"
        />
      </div>
      <small id="hmiGoalActiveText">I'm listening ...</small>
    </div>
    <div
      v-else-if="text || imageSrc"
      id="contentBox"
    >
      <div
        v-if="imageSrc"
        id="image"
        :style="{backgroundImage: `url(${imageSrc})`}"
      />
      <span
        id="text"
        :style="{fontSize: imageSrc ? '50px' : '90px'}"
        v-text="text"
      />
    </div>

    <!-- logo -->
    <div
      v-if="hmiGoalActive || text || imageSrc"
      id="logoSmall"
    >
      <img src="./assets/logo.png">
    </div>
    <div
      v-else
      id="logoBig"
    >
      <img src="./assets/logo.png">
    </div>

    <!-- battery -->
    <div
      v-show="!hmiGoalActive && !text && !imageSrc"
      id="battery"
    >
      <Battery
        :ros="ros"
      />
    </div>

    <div class="backgroundArea">
      <ul class="circles">
        <li
          v-for="index in 10"
          :key="index"
        />
      </ul>
    </div>
  </div>
</template>

<script>
/* eslint new-cap: ["error", { "properties": false }] */
/* eslint node/prefer-global/buffer: [error, never] */

import AutoRos from 'auto-ros'
import { Battery } from 'hero-vue'
import ROSLIB from 'roslib'
import jpeg from 'jpeg-js'

import { library } from '@fortawesome/fontawesome-svg-core'
import { faMicrophone, faSpinner } from '@fortawesome/free-solid-svg-icons'
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
library.add(faMicrophone)
library.add(faSpinner)

const remote = require('@electron/remote')

const { ArrayBuffer, Buffer, Uint8Array } = require('buffer')

function imageToBase64JpegString (msg) {
  const raw = atob(msg.data)
  const array = new Uint8Array.from(new ArrayBuffer.alloc(raw.length))
  for (let i = 0; i < raw.length; i++) {
    array[i] = raw.charCodeAt(i)
  }

  const frameData = Buffer.alloc(msg.width * msg.height * 4)
  for (let i = 0; i < msg.width * msg.height; i++) {
    if (msg.encoding === 'rgb8') {
      frameData[4 * i + 0] = array[3 * i + 0]
      frameData[4 * i + 2] = array[3 * i + 2]
    } else if (msg.encoding === 'bgr8') {
      frameData[4 * i + 0] = array[3 * i + 2]
      frameData[4 * i + 2] = array[3 * i + 0]
    } else {
      console.error('invalid encoding', msg.encoding)
    }
    frameData[4 * i + 1] = array[3 * i + 1]
    frameData[4 * i + 3] = 0
  }
  const rawImageData = {
    data: frameData,
    width: msg.width,
    height: msg.height
  }
  return 'data:image/jpeg;base64,' + jpeg.encode(rawImageData, 50).data.toString('base64')
}

export default {
  name: 'HeroDisplay',
  components: {
    Battery,
    'font-awesome-icon': FontAwesomeIcon
  },
  data () {
    return {
      ros: AutoRos.ros,
      textTopic: new ROSLIB.Topic({
        ros: AutoRos.ros,
        name: '/talking_sentence',
        messageType: 'std_msgs/String'
      }),
      imageTopic: new ROSLIB.Topic({
        ros: AutoRos.ros,
        name: 'image_from_ros',
        messageType: 'sensor_msgs/Image'
      }),
      compressedImageTopic: new ROSLIB.Topic({
        ros: AutoRos.ros,
        name: 'hmi/image',
        messageType: 'sensor_msgs/CompressedImage'
      }),
      hmiStatusTopic: new ROSLIB.Topic({
        ros: AutoRos.ros,
        name: 'hmi/status',
        messageType: 'actionlib_msgs/GoalStatusArray'
      }),
      text: '',
      msPerChar: 100,
      imageSrc: null,
      imageShowSeconds: 4,
      hmiGoalActive: false,
      endPoint: 'ws://localhost:9090',
      textTimeout: null,
      imageTimeout: null
    }
  },
  mounted () {
    if (!remote.process.env.NO_FULLSCREEN) {
      remote.getCurrentWindow().setFullScreen(true)
    }
    const argv = remote.process.argv
    const index = argv.length - 1
    let url = this.endPoint
    if (index > 0) {
      const host = argv[index]
      url = `ws://${host}:9090`
    }

    AutoRos.ros.on('connection', this.OnConnection.bind(this))
    AutoRos.ros.on('close', this.OnClose.bind(this))
    AutoRos.connect(url)
    this.textTopic.subscribe((msg) => {
      this.setText(msg.data)
    })
    this.imageTopic.subscribe((msg) => {
      this.imageSrc = imageToBase64JpegString(msg)
      this.setupClearImage(msg.header.stamp)
    })
    this.compressedImageTopic.subscribe((msg) => {
      this.imageSrc = 'data:image/jpeg;base64,' + msg.data
      this.setupClearImage(msg.header.stamp)
    })
    this.hmiStatusTopic.subscribe((msg) => {
      let active = false
      msg.status_list.forEach((status) => {
        if (status.status === 1) {
          active = true
        }
      })
      this.hmiGoalActive = active
    })
  },
  beforeUnmount () {
    this.textTopic.unsubscribe()
    this.imageTopic.unsubscribe()
    this.compressedImageTopic.unsubscribe()
    this.hmiStatusTopic.unsubscribe()
  },
  methods: {
    setupClearImage (stamp) {
      if (this.imageTimeout) {
        clearTimeout(this.imageTimeout)
      }
      const seconds = stamp.secs + 1e-9 * stamp.nsecs
      this.imageTimeout = setTimeout(() => {
        this.imageSrc = null
      }, seconds <= 0.0 ? this.imageShowSeconds * 1000 : seconds * 1000)
    },
    setupClearText (seconds) {
      if (this.textTimeout) {
        clearTimeout(this.textTimeout)
      }
      this.textTimeout = setTimeout(() => {
        this.text = ''
      }, seconds <= 0.0 ? this.msPerChar * this.text.length + 2000 : seconds * 1000)
    },
    setText (data, seconds = 0) {
      this.text = data
      this.setupClearText(seconds)
    },
    OnConnection () {
      this.setText('Connection established', 1)
    },
    OnClose () {
      this.setText('Connection lost', 1e5)
    }
  }
}
</script>

<style>
#contentBox {
  height: 600px;
  width: 1024px;
  display: flex;
  justify-content: center;
  align-items: center;
}
#mic_spinner svg {
  height: 320px;
  vertical-align: -0.125em;
  font-size: 20em;
}
#mic {
  color: #067340;
  width: 240px;
}
#spinner {
  color: #df9101;
  width: 340px;
}
#image {
  width: 100%;
  height: 100%;
  background-size: contain;
  background-repeat: no-repeat;
  background-position: center; /* Center the image */
}
body {
  background-color: #171717;
  width: 1024px;
  height: 600px;
  position: absolute;
  top: 0px;
  left: 0px;
  margin: 0;
  padding: 0;
  overflow: hidden;
}
#app {
  position: relative;
  text-align: center;
  font-size: 90px;
  color: white;
  font-family: Ubuntu;
  width: 1024px;
  height: 600px;
  position: absolute;
  top: 0px;
  left: 0px;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
}
#hmiGoalActive {
  padding: 70px;
}
#hmiGoalActiveText {
  color: #a2a2a2;
  font-size: 60px;
}
#logoBig {
  margin-top: 75px;
}
#logoSmall {
  margin-top: 55px;
  position: fixed;
  bottom: 0px;
  right: 0px;
  margin: 20px;
}
#logoSmall img {
  width: 120px;
}

#battery {
  position: fixed;
  bottom: 0px;
  left: 0px;
  margin: 20px;
}
#batteryProgress {
  height: 2rem;
}

#battery_col {
  width: 100px;
  padding-left: 5px;
  padding-right: 5px;
  margin-left: 0px;
  margin-right: 0px;
}

.backgroundArea {
  z-index: -100;
  width: 1024px;
  height: 600px;
  position: absolute;
  top: 0px;
  left: 0px;
}

.circles{
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  overflow: hidden;
  margin: 0px;
  padding: 0px;
}

.circles li{
  position: absolute;
  display: block;
  list-style: none;
  width: 20px;
  height: 20px;
  background: rgba(255, 255, 255, 0.2);
  animation: animate 25s linear infinite;
  bottom: -150px;

}

.circles li:nth-child(1){
  left: 25%;
  width: 80px;
  height: 80px;
  animation-delay: 0s;
}

.circles li:nth-child(2){
  left: 10%;
  width: 20px;
  height: 20px;
  animation-delay: 2s;
  animation-duration: 12s;
}

.circles li:nth-child(3){
  left: 70%;
  width: 20px;
  height: 20px;
  animation-delay: 4s;
}

.circles li:nth-child(4){
  left: 40%;
  width: 60px;
  height: 60px;
  animation-delay: 0s;
  animation-duration: 18s;
}

.circles li:nth-child(5){
  left: 65%;
  width: 20px;
  height: 20px;
  animation-delay: 0s;
}

.circles li:nth-child(6){
  left: 75%;
  width: 110px;
  height: 110px;
  animation-delay: 3s;
}

.circles li:nth-child(7){
  left: 35%;
  width: 150px;
  height: 150px;
  animation-delay: 7s;
}

.circles li:nth-child(8){
  left: 50%;
  width: 25px;
  height: 25px;
  animation-delay: 15s;
  animation-duration: 45s;
}

.circles li:nth-child(9){
  left: 20%;
  width: 15px;
  height: 15px;
  animation-delay: 2s;
  animation-duration: 35s;
}

.circles li:nth-child(10){
  left: 85%;
  width: 150px;
  height: 150px;
  animation-delay: 0s;
  animation-duration: 11s;
}

@keyframes animate {
  0%{
    transform: translateY(0) rotate(0deg);
    opacity: 1;
    border-radius: 0;
  }

  100%{
    transform: translateY(-1000px) rotate(720deg);
    opacity: 0;
    border-radius: 50%;
  }
}
</style>
