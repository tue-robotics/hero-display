# hero-display

Display sink for the hero display that presents information to the user.

# Build Setup

```bash
# install dependencies
npm install

# serve with hot reload at localhost:9080
npm run dev

# build electron application for production
npm run build

# run unit & end-to-end tests
npm test

# lint all JS/Vue component files in `src/`
npm run lint

```

# Test hero display

## Dependencies

```bash
sudo apt-get install ros-$ROS_DISTRO-rosbridge-server ros-$ROS_DISTRO-rostopic
```

## Run

```bash
# Launch rosbridge server
roslaunch rosbridge_server rosbridge_websocket.launch

# Launch example string publisher
rostopic pub /text_to_speech/output std_msgs/String -- "I am hero, an awesome robot!"

# Run hero-display executable
./build/linux-unpacked/hero-display
```
