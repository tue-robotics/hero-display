<template>
  <div id="app">
    <div v-if="hmiGoalActive || pvGetIntentGoalActive" id="hmiGoalActive">
      <div id="mic_spinner">
        <font-awesome-icon id="mic" icon="microphone" class="fa-icon" />
        <font-awesome-icon id="spinner" icon="spinner" pulse class="fa-icon" />
      </div>
      <small id="hmiGoalActiveText">I'm listening ...</small>
    </div>
    <div v-else-if="text || imageSrc" id="contentBox">
      <div v-if="imageSrc" id="image" :style="{ backgroundImage: `url(${imageSrc})` }" />
      <span id="text" :style="{ fontSize: imageSrc ? '50px' : '90px' }" v-text="text" />
    </div>

    <!-- logo -->
    <div v-if="hmiGoalActive || text || imageSrc" id="logoSmall">
      <img src="./assets/logo.png" />
    </div>
    <div v-else id="logoBig">
      <img src="./assets/logo.png" />
    </div>

    <!-- battery -->
    <div v-show="!hmiGoalActive && !text && !imageSrc" id="battery">
      <Battery :ros="autoRos.ros" />
    </div>

    <div class="backgroundArea">
      <ul class="circles">
        <li v-for="index in 10" :key="index" />
      </ul>
    </div>
  </div>
</template>

<script setup lang="ts">
  import { ref, onMounted, onBeforeUnmount } from "vue";
  import { AutoRos } from "auto-ros";
  import { Battery } from "hero-vue";
  import * as ROSLIB from "roslib";
  import { library } from "@fortawesome/fontawesome-svg-core";
  import { faMicrophone, faSpinner } from "@fortawesome/free-solid-svg-icons";
  import { FontAwesomeIcon } from "@fortawesome/vue-fontawesome";

  interface Stamp1 {
    secs: number;
    nsecs: number;
  }

  interface Stamp2 {
    sec: number;
    nanosec: number;
  }

  type Stamp = Stamp1 | Stamp2;

  interface HeaderMsg {
    seq: number;
    stamp: Stamp;
    frame_id: string;
  }

  interface ImageMsg {
    header: HeaderMsg;
    data: string;
    width: number;
    height: number;
    encoding: string;
  }

  interface StringMsg {
    data: string;
  }

  interface UUIDMsg {
    uuid: {
      data: Array<number>;
    };
  }

  interface GoalInfoMsg {
    goal_id: UUIDMsg;
    stamp: Stamp;
  }

  interface GoalStatusMsg {
    goal_info: GoalInfoMsg;
    status: number;
  }

  interface GoalStatusArrayMsg {
    status_list: Array<GoalStatusMsg>;
  }

  library.add(faMicrophone, faSpinner);

  // Buffer cache for reusing frameData buffers across images with different dimensions
  // Key: dimension string (e.g., "640x480"), Value: Uint8ClampedArray buffer
  const frameBufferCache = new Map<string, Uint8ClampedArray>();
  const MAX_CACHED_BUFFERS = 5; // Limit cache size to prevent unbounded memory growth

  function getOrCreateFrameBuffer(width: number, height: number): Uint8ClampedArray {
    const key = `${width}x${height}`;
    const size = width * height * 4;

    const cachedBuffer = frameBufferCache.get(key);
    if (cachedBuffer) {
      return cachedBuffer;
    }

    // If cache is full, remove the oldest entry (first entry in Map)
    if (frameBufferCache.size >= MAX_CACHED_BUFFERS) {
      const firstKey = frameBufferCache.keys().next().value;
      frameBufferCache.delete(firstKey);
    }

    const buffer = new Uint8ClampedArray(size);
    frameBufferCache.set(key, buffer);
    return buffer;
  }

  function imageToBase64JpegString(msg: ImageMsg): string {
    const raw = atob(msg.data);
    const rawArray = new Uint8Array(raw.length);

    for (let i = 0; i < raw.length; i++) {
      rawArray[i] = raw.charCodeAt(i);
    }

    const frameData = getOrCreateFrameBuffer(msg.width, msg.height);

    for (let i = 0; i < msg.width * msg.height; i++) {
      if (msg.encoding === "rgb8") {
        frameData[4 * i + 0] = rawArray[3 * i + 0]; // R
        frameData[4 * i + 1] = rawArray[3 * i + 1]; // G
        frameData[4 * i + 2] = rawArray[3 * i + 2]; // B
      } else if (msg.encoding === "bgr8") {
        frameData[4 * i + 0] = rawArray[3 * i + 2]; // R
        frameData[4 * i + 1] = rawArray[3 * i + 1]; // G
        frameData[4 * i + 2] = rawArray[3 * i + 0]; // B
      } else {
        console.error("Invalid encoding", msg.encoding);
      }
      frameData[4 * i + 3] = 255; // Alpha channel (fully opaque)
    }

    // Create canvas to encode as JPEG
    const canvas = document.createElement("canvas");
    canvas.width = msg.width;
    canvas.height = msg.height;
    const ctx = canvas.getContext("2d");

    if (!ctx) {
      throw new Error("Failed to get canvas context");
    }

    const imageData = new ImageData(frameData, msg.width, msg.height);
    ctx.putImageData(imageData, 0, 0);

    // Convert to JPEG with quality 0.5 (equivalent to quality 50)
    return canvas.toDataURL("image/jpeg", 0.5);
  }

  const autoRos = new AutoRos();
  const text = ref("");
  const msPerChar = 100;
  const imageSrc = ref<string | null>(null);
  const imageShowSeconds = 4;
  const hmiGoalActive = ref(false);
  const pvGetIntentGoalActive = ref(false);
  const endPoint = "ws://localhost:9090";
  let textTimeout: ReturnType<typeof setTimeout> | null = null;
  let imageTimeout: ReturnType<typeof setTimeout> | null = null;

  const textTopic = new ROSLIB.Topic<StringMsg>({
    ros: autoRos.ros,
    name: "text_to_speech/output",
    messageType: "std_msgs/String",
  });
  const imageTopic = new ROSLIB.Topic<ImageMsg>({
    ros: autoRos.ros,
    name: "image_from_ros",
    messageType: "sensor_msgs/Image",
  });
  const compressedImageTopic = new ROSLIB.Topic<ImageMsg>({
    ros: autoRos.ros,
    name: "hmi/image",
    messageType: "sensor_msgs/CompressedImage",
  });
  const hmiStatusTopic = new ROSLIB.Topic<GoalStatusArrayMsg>({
    ros: autoRos.ros,
    name: "hmi/status",
    messageType: "actionlib_msgs/GoalStatusArray",
  });
  const pvGetIntentStatusTopic = new ROSLIB.Topic<GoalStatusArrayMsg>({
    ros: autoRos.ros,
    name: "get_intent/status",
    messageType: "actionlib_msgs/GoalStatusArray",
  });

  function calculateSeconds(stamp: Stamp): number {
    if ("secs" in stamp && "nsecs" in stamp) {
      // ROS1
      return stamp.secs + 1e-9 * stamp.nsecs;
    } else if ("sec" in stamp && "nanosec" in stamp) {
      // ROS2
      return stamp.sec + 1e-9 * stamp.nanosec;
    } else {
      throw new TypeError("Invalid stamp format: " + JSON.stringify(stamp));
    }
  }

  function setupClearImage(stamp: Stamp) {
    if (imageTimeout) clearTimeout(imageTimeout);
    let seconds = calculateSeconds(stamp);
    seconds = seconds > 0.0 ? seconds : imageShowSeconds;
    console.debug(`Setting up image timeout of ${seconds.toFixed(2)} seconds`);
    imageTimeout = setTimeout(() => {
      console.debug("Clearing image");
      imageSrc.value = null;
    }, seconds * 1000);
  }

  function setupClearText(seconds: number) {
    if (textTimeout) clearTimeout(textTimeout);
    const ms = seconds > 0.0 ? seconds * 1000 : msPerChar * text.value.length + 2000;
    console.debug(`Setting text timeout of ${(ms / 1000).toFixed(2)} seconds`);
    textTimeout = setTimeout(() => {
      console.debug("Clearing text");
      text.value = "";
    }, ms);
  }

  function setText(data: string, seconds = 0) {
    text.value = data;
    setupClearText(seconds);
  }

  function OnConnection() {
    setText("Connection established", 1);
  }

  function OnClose() {
    setText("Connection lost", 1e5);
  }

  onMounted(async () => {
    const host = (await window?.args?.host()) || null;
    let url = endPoint;
    if (host) {
      url = `ws://${host}:9090`;
    }

    autoRos.ros.on("connection", OnConnection);
    autoRos.ros.on("close", OnClose);
    autoRos.connect(url);

    textTopic.subscribe((msg: StringMsg) => {
      setText(msg.data);
    });
    imageTopic.subscribe((msg: ImageMsg) => {
      try {
        imageSrc.value = imageToBase64JpegString(msg);
      } catch (error) {
        console.error("Failed to convert image:", error);
      }
      setupClearImage(msg.header.stamp);
    });
    compressedImageTopic.subscribe((msg: ImageMsg) => {
      imageSrc.value = "data:image/jpeg;base64," + msg.data;
      setupClearImage(msg.header.stamp);
    });
    hmiStatusTopic.subscribe((msg: GoalStatusArrayMsg) => {
      let active = false;
      msg.status_list.forEach((status: GoalStatusMsg) => {
        if (status.status === 1) active = true;
      });
      hmiGoalActive.value = active;
    });
    pvGetIntentStatusTopic.subscribe((msg: GoalStatusArrayMsg) => {
      let active = false;
      msg.status_list.forEach((status: GoalStatusMsg) => {
        if (status.status === 1) active = true;
      });
      pvGetIntentGoalActive.value = active;
    });
  });

  onBeforeUnmount(() => {
    textTopic.unsubscribe();
    imageTopic.unsubscribe();
    compressedImageTopic.unsubscribe();
    hmiStatusTopic.unsubscribe();
    pvGetIntentStatusTopic.unsubscribe();
  });
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

  #battery .progress {
    height: 2rem;
  }

  #battery .col {
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

  .circles {
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    overflow: hidden;
    margin: 0px;
    padding: 0px;
  }

  .circles li {
    position: absolute;
    display: block;
    list-style: none;
    width: 20px;
    height: 20px;
    background: rgba(255, 255, 255, 0.2);
    animation: animate 25s linear infinite;
    bottom: -150px;
  }

  .circles li:nth-child(1) {
    left: 25%;
    width: 80px;
    height: 80px;
    animation-delay: 0s;
  }

  .circles li:nth-child(2) {
    left: 10%;
    width: 20px;
    height: 20px;
    animation-delay: 2s;
    animation-duration: 12s;
  }

  .circles li:nth-child(3) {
    left: 70%;
    width: 20px;
    height: 20px;
    animation-delay: 4s;
  }

  .circles li:nth-child(4) {
    left: 40%;
    width: 60px;
    height: 60px;
    animation-delay: 0s;
    animation-duration: 18s;
  }

  .circles li:nth-child(5) {
    left: 65%;
    width: 20px;
    height: 20px;
    animation-delay: 0s;
  }

  .circles li:nth-child(6) {
    left: 75%;
    width: 110px;
    height: 110px;
    animation-delay: 3s;
  }

  .circles li:nth-child(7) {
    left: 35%;
    width: 150px;
    height: 150px;
    animation-delay: 7s;
  }

  .circles li:nth-child(8) {
    left: 50%;
    width: 25px;
    height: 25px;
    animation-delay: 15s;
    animation-duration: 45s;
  }

  .circles li:nth-child(9) {
    left: 20%;
    width: 15px;
    height: 15px;
    animation-delay: 2s;
    animation-duration: 35s;
  }

  .circles li:nth-child(10) {
    left: 85%;
    width: 150px;
    height: 150px;
    animation-delay: 0s;
    animation-duration: 11s;
  }

  @keyframes animate {
    0% {
      transform: translateY(0) rotate(0deg);
      opacity: 1;
      border-radius: 0;
    }

    100% {
      transform: translateY(-1000px) rotate(720deg);
      opacity: 0;
      border-radius: 50%;
    }
  }
</style>
