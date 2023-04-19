# hero-display

[![CI](https://github.com/tue-robotics/hero-display/actions/workflows/main.yml/badge.svg)](https://github.com/tue-robotics/hero-display/actions/workflows/main.yml)

Display sink for the hero display that presents information to the user.

## Build Setup

```bash
# install dependencies
npm install

# Compiles and hot-reloads for development
npm run electron:serve

# Compiles and minifies for production
npm run electron:build

# Run your unit tests
NO_FULLSCREEN=1 npm run test:unit

# Lints and fixes files
npm run lint
```

## Test hero display

### Dependencies

```bash
sudo apt-get install ros-$ROS_DISTRO-rosbridge-server ros-$ROS_DISTRO-rostopic
```

### Run

```bash
# Launch rosbridge server
roslaunch rosbridge_server rosbridge_websocket.launch

# Launch example string publisher
rostopic pub /text_to_speech/output std_msgs/String -- "I am hero, an awesome robot!"

# Run hero-display executable
NO_FULLSCREEN=1 ./dist_electron/hero-display.AppImage
```

To connect to a different rosbridge webserver,
add the desired hostname or ip-address as final argument.
