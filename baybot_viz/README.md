# baybot_viz

## Overview

This package allows for launching robot visualization within `rviz`.

Currently, there are two launch files:

* `view_model`: Allows for visualization of the model
* `view_robot`: Launches the robot's vizualization. `roscore`, `gazebo`,
  etc. must be running.

## Usage

```bash

roslaunch baybot_viz view_model.launch

```
