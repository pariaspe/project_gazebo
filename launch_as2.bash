#!/bin/bash

usage() {
    echo "  options:"
    echo "      -b: launch behavior tree"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
}

# Arg parser
while getopts "bmrt" opt; do
  case ${opt} in
    b )
      behavior_tree="true"
      ;;
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[wrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# Shift optional args
shift $((OPTIND -1))

# HOW TO INCLUDE WORLDS OR MODELS FROM THE PROJECT
export IGN_GAZEBO_RESOURCE_PATH=$PWD/worlds:$IGN_GAZEBO_RESOURCE_PATH

## DEFAULTS
behavior_tree=${behavior_tree:="false"}
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}

# CHOOSE SIMULATION CONFIG FILE
echo "Choose simulation config file to open:"
cat -n <(ls -1 sim_config/*.json) # list json files
simulation_config=$(python utils/choose_sim_config.py | tail -n 1)
if [[ ${simulation_config} == "Invalid" ]]; then
    exit 1
fi

# Get drone namespaces from swarm config file
drones=$(python utils/get_drones.py ${simulation_config})

drones_arr=(${drones//:/ })
for drone in "${drones_arr[@]}"
do
    tmuxinator start -p tmuxinator/session.yml \
        drone_namespace=${drone} \
        simulation_config=${simulation_config} \
        behavior_tree=${behavior_tree} &
    wait
done

if [[ ${record_rosbag} == "true" ]]; then
    tmuxinator start -p tmuxinator/rosbag.yml \
        drone_namespace=${drones} &
    wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
    # TODO: Keyboard Teleop uses ',' as separator for drone namespaces
    drones_sep=$(python utils/get_drones.py ${config_file} --sep ",")
    tmuxinator start -n keyboard_teleop -p tmuxinator/keyboard_teleop.yml \
        simulation=true \
        drone_namespace=${drones_sep} &
    wait
fi

tmuxinator start -p tmuxinator/gazebo.yml simulation_config=${simulation_config} &
wait

# Attach to tmux session ${drone_arr[@]}, window mission
tmux attach-session -t ${drones_arr[0]}:mission
