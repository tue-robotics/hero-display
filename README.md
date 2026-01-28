# hero-display

[![CI](https://github.com/tue-robotics/hero-display/actions/workflows/main.yml/badge.svg)](https://github.com/tue-robotics/hero-display/actions/workflows/main.yml)

Display sink for the hero display that presents information to the user.

## Build Setup

```bash
# install dependencies
npm install

# Compiles and hot-reloads for development
npm run dev

# Compiles and minifies for production
npm run build

# Run your unit tests
NO_FULLSCREEN=1 npm run test

# Lints and fixes files
npm run format
npm run lint
```

## Test hero display

### Dependencies

#### ROS1

```bash
sudo apt-get install ros-${ROS_DISTRO}-rosbridge-server ros-${ROS_DISTRO}-rostopic
```

#### ROS2

```bash
sudo apt-get install ros-${ROS_DISTRO}-rosbridge-server ros-${ROS_DISTRO}-ros2topic
```

### Run

#### ROS1

```bash
# Launch rosbridge server
roslaunch rosbridge_server rosbridge_websocket.launch

# Launch example string publisher
rostopic pub /text_to_speech/output std_msgs/String -- "I am hero, an awesome robot!"
```

#### ROS2

```bash
# Launch rosbridge server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Launch example string publisher
ros2 topic pub /text_to_speech/output std_msgs/msg/String "data: 'I am hero, an awesome robot!'"
```

#### Start hero-display

```bash
# Run hero-display executable
NO_FULLSCREEN=1 ./dist_electron/hero-display.AppImage
```

To connect to a different rosbridge webserver,
add the desired hostname or ip-address as final argument.

```bash
NO_FULLSCREEN=1 ./dist_electron/hero-display.AppImage --host=other_machine.local
```
