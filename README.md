# hero-display

[![CI][gh-actions-image]][gh-actions-url] [![Dependency Status][daviddm-image]][daviddm-url] [![devDependency Status][daviddm-image-dev]][daviddm-url-dev]

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

[gh-actions-image]: https://github.com/tue-robotics/hero-display/workflows/CI/badge.svg
[gh-actions-url]: https://github.com/tue-robotics/hero-display/actions

[daviddm-image]: https://david-dm.org/tue-robotics/hero-display/status.svg
[daviddm-url]: https://david-dm.org/tue-robotics/hero-display
[daviddm-image-dev]: https://david-dm.org/tue-robotics/hero-display/dev-status.svg
[daviddm-url-dev]: https://david-dm.org/tue-robotics/hero-display?type=dev
