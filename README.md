# Baybot

!!!THIS IS ABSOLUTELY UNDER CONSTRUCTION!!!

## Set up workspace

Install dependencies:

```bash
sudo apt-get update
sudo apt-get install python-catkin-tools curl
sudo apt-get install libusb-dev libspnav-dev libx11-dev libbluetooth-dev libcurl4-openssl-dev libcwiid-dev libbullet-dev libeigen3-dev  libyaml-cpp-dev
curl -sSL http://get.gazebosim.org | sh
```

```bash

BAYBOT_REPO=~/repos/baybot #or wherever
BAYBOT_WS=~/baybot_ws #or wherever

# create repo directory and clone repo
mkdir -p BAYBOT_REPO
cd BAYBOT_REPO
git clone git@github.com:payneio/baybot.git

# create catkin-tools workspace
mkdir -p BAYBOT_WS
cd BAYBOT_WS
catkin init
mkdir src # temp to get the devel set up correctly
catkin build
rmdir src
source devel/setup.sh

# link repo as workspace src
ln -sT BAYBOT_REPO BAYBOT_WS/src

# build for real
catkin build
```

# Build

```bash
cd src && git submodule update --init && cd ..
catkin build
source devel/setup.bash
```

# Simulation

```bash
roslaunch baybot_gazebo baybot_empty_world.launch
BAYBOT_LOGITECH=1 roslaunch baybot_control teleop.launch
```
