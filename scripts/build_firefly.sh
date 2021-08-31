#!/bin/bash
cd ..

rotors_description_dir=$(pwd)/models/rotors_description
scripts_dir=$(pwd)/scripts

echo $rotors_description_dir
echo $scripts_dir

enable_mavlink_interface=1
enable_ground_truth=1
enable_wind=0
send_vision_estimation=0
send_odometry=1


python3 ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/firefly_base.urdf  ${rotors_description_dir}/urdf/firefly_base.xacro enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_wind:=${enable_wind} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir} send_vision_estimation:=${send_vision_estimation} send_odometry:=${send_odometry}

gz sdf -p  ${rotors_description_dir}/urdf/firefly_base.urdf >> $(pwd)/models/firefly/firefly.sdf

rm ${rotors_description_dir}/urdf/firefly_base.urdf
