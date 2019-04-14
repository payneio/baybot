# Baybot

## Set up workspace

Install dependencies:

```bash
sudo apt-get update
sudo apt-get install python-catkin-tools
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers
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
catkin build
```
