<template>
  <div id="app">
    <div v-if="hmiGoalActive" id="hmiGoalActive">
      <div>
        <v-icon name="microphone" scale="20" id="mic" />
        <v-icon name="spinner" scale="20" pulse id="spinner" />
      </div>
      <small id="hmiGoalActiveText">I'm listening ...</small>
    </div>
    <div v-else-if="text || imageSrc" id="contentBox">
      <div v-if="imageSrc" id="image" :style="{backgroundImage: `url(${imageSrc})`}" />
      <span v-text='text' id="text" :style="{fontSize: imageSrc ? '50px' : '90px'}" />
    </div>

    <!-- logo -->
    <div v-if="hmiGoalActive || text || imageSrc" id="logoSmall">
      <img src="static/logo.png" />
    </div>
    <div v-else id="logoBig">
      <img src="static/logo.png" />
    </div>

    <!-- battery -->
    <div v-if="!hmiGoalActive && !text && !imageSrc" id="battery">
      <b-container fluid class="p-0 m-0">
        <b-row>
          <b-col v-for="(v, key) in batteries" :key="key" align="center" id="battery_col" class="pr-0">
            <span>
                <h5 vertical-align="text-bottom">
                  {{key}}
                  <v-icon v-if="v.charging" name="bolt" id="bolt" />
                </h5>
            </span>
            <b-progress class="w-100" id="batteryProgress" :style="batteryProgressStyle">
              <b-progress-bar :value="v.percentage" :animated="v.charging" :variant="v.type"">
                <span class="position-absolute w-100 d-block"><b>{{v.percentage}}%</b></span>
              </b-progress-bar>
            </b-progress>
          </b-col>
        </b-row>
      </b-container>
    </div>

    <div class="backgroundArea" >
      <ul class="circles">
        <li v-for="index in 10" :key="index" />
      </ul>
    </div>
  </div>
</template>

<script>
  import { remote } from 'electron'
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
        console.error('invalid encoding', msg.encoding)
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
        batteryTopic: new ROSLIB.Topic({
          ros: AutoRos.ros,
          name: 'battery',
          messageType: 'sensor_msgs/BatteryState'
        }),
        text: '',
        msPerChar: 100,
        imageSrc: null,
        imageShowSeconds: 4,
        hmiGoalActive: false,
        endPoint: 'ws://localhost:9090',
        textTimeout: null,
        imageTimeout: null,
        batteries: {},
        batteryProgressStyle: {
          'background-color': '#d0d0d0'
        }
      }
    },
    methods: {
      setupClearImage (stamp) {
        if (this.imageTimeout) {
          clearTimeout(this.imageTimeout)
        }
        var seconds = stamp.secs + 1e-9 * stamp.nsecs
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
      setupClearBatteryType (key, seconds = 10) {
        if (this.batteries[key].TypeTimeOut) {
          clearTimeout(this.batteries[key].TypeTimeOut)
        }
        this.batteries[key].TypeTimeOut = setTimeout(() => {
          this.batteries[key].type = 'dark'
          this.batteries[key].charging = false
        }, seconds * 1000)
      },
      setupRemoveBattery (key, seconds = 60) {
        if (this.batteries[key].RemoveTimeOut) {
          clearTimeout(this.batteries[key].RemoveTimeOut)
        }
        this.batteries[key].RemoveTimeOut = setTimeout(() => {
          console.log('deleting battery', key)
          this.$delete(this.batteries, key)
        }, seconds * 1000)
      },
      OnConnection () {
        this.setText('Connection established', 1)
      },
      OnClose () {
        this.setText('Connection lost', 1e5)
      },
      handleBatteryMsg (msg) {
        var type = 'info'
        const percentage = Math.round(msg.percentage * 100)
        if (percentage > 40) {
          type = 'success'
        } else if (percentage > 20) {
          type = 'warning'
        } else {
          type = 'danger'
        }

        const batteries = this.batteries
        const key = msg.location

        // Get battery or create new one
        var battery
        if (!batteries.hasOwnProperty(key)) {
          battery = {
            percentage: null,
            type: null,
            charging: null,
            TypeTimeOut: null,
            RemoveTimeOut: null
          }
        } else {
          battery = batteries[key]
        }
        // Only update the state, not the timeouts, which are done
        // at the end
        battery.percentage = percentage
        battery.type = type
        battery.charging = msg.power_supply_status === 1 // POWER_SUPPLY_STATUS_CHARGING = 1

        // Update current battery
        batteries[key] = battery

        // Order batteries, so it shown on alphabetical order
        const ordered = {}
        Object.keys(batteries).sort().forEach(function (key) {
          ordered[key] = batteries[key]
        })

        // Update batteries with ordered
        this.batteries = ordered

        // Setup Timeouts for this battery
        this.setupClearBatteryType(key, 10)
        this.setupRemoveBattery(key, 60)
      }
    },
    mounted () {
      if (!process.env.NO_FULLSCREEN) {
        remote.getCurrentWindow().setFullScreen(true)
      }
      const remote2 = window.require('electron').remote
      const argv = remote2.process.argv
      const index = argv.length - 1
      var url = this.endPoint
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
        var active = false
        msg.status_list.forEach((status) => {
          if (status.status === 1) {
            active = true
          }
        })
        this.hmiGoalActive = active
      })
      this.batteryTopic.subscribe(this.handleBatteryMsg)
    },
    beforeDestroy () {
      this.textTopic.unsubscribe({})
      this.imageTopic.unsubscribe({})
      this.compressedImageTopic.unsubscribe({})
      this.hmiStatusTopic.unsubscribe({})
      this.batteryTopic.unsubscribe({})
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
#mic {
  color: #067340;
}
#image {
  width: 100%;
  height: 100%;
  background-size: contain;
  background-repeat: no-repeat;
  background-position: center; /* Center the image */
}
#spinner {
  color: #df9101;
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
  width: 120px;
}

#bolt {
  color: #FFFF00;
  height: 1rem;
  width: auto;
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
