#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
# roscore&
# sleep 5
# rosbag play /data/bag.bag --loop&
dt-exec roslaunch led_controller led_controller_node.launch veh:="$VEHICLE_NAME"
dt-exec roslaunch deadreckoning deadreckoning_node.launch veh:="$VEHICLE_NAME"
dt-exec roslaunch augmented_reality augmented_reality_node.launch veh:="$VEHICLE_NAME"
# dt-exec roslaunch lane_following lane_following.launch veh:="$VEHICLE_NAME"


# ----------------------------------------------------------------------------

# wait for app to end
dt-launchfile-join
