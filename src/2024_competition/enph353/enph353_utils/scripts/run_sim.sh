#!/bin/bash

spawn_vehicles='false'
spawn_pedestrians='false'
generate_plates='false'
label_plates='false'
wind_blowing='false'

print_usage() {
  echo "Usage:"
  echo "-v to spawn vehicles"
  echo "-p to spawn pedestrians"
  echo "-g to generate new license plates"
  echo "-l to spawn QR code labels"
  echo "-w to enable wind"
}

while getopts 'vpglw' flag; do
  case "${flag}" in
    v) spawn_vehicles='true' ;;
    p) spawn_pedestrians='true' ;;
    g) generate_plates='true' ;;
    l) label_plates='true' ;;
    w) wind_blowing='true' ;;
    *) print_usage
       exit 1 ;;
  esac
done

# display the last commit log and status then wait two seconds (this is to check that
# no changes were made to the competition package
echo -e "\n\n################## COMPETITION PACKAGE #########################\n"
git --no-pager log | head
echo -e "\n************************ GIT STATUS ****************************"
git status | head
echo -e "\n################################################################\n"
#sleep 5s

# generate new plates if necessary
if [[ "$generate_plates" = "true" ]]
then
	SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
	SCRIPT_PATH="$SCRIPT_DIR/../../enph353_gazebo/scripts"
  echo "Generating new clues..."
	python3 "$SCRIPT_PATH/clue_generator.py"
  
  echo "Generating new plates from new clues..."
	python3 "$SCRIPT_PATH/plate_generator.py"
else
  echo "Loading previous clues..."
fi

# display plates with or without QR code
if [[ "$label_plates" = "true" ]]
then
  ln -sfn labelled ../../enph353_gazebo/media/materials/textures/license_plates
else
  ln -sfn unlabelled ../../enph353_gazebo/media/materials/textures/license_plates
fi

# start the ROS environment
roslaunch enph353_utils sim.launch spawn_pedestrians:=$spawn_pedestrians spawn_vehicles:=$spawn_vehicles wind_blowing:=$wind_blowing