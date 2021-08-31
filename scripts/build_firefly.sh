#!/bin/bash
cd ..

rotors_description_dir=$(pwd)/models/rotors_description
scripts_dir=$(pwd)/scripts

echo $rotors_description_dir
echo $scripts_dir

# build non-hil model
enable_mavlink_interface="true"
enable_ground_truth="true"
enable_wind="false"
send_vision_estimation="false"
send_odometry="true"
serial_device='/tmp/ttyACM0'
serial_enabled="false"
hil_mode="false"

args="enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_wind:=${enable_wind} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir} send_vision_estimation:=${send_vision_estimation} send_odometry:=${send_odometry} hil_mode:=${hil_mode} serial_device:=${serial_device} serial_enabled:=${serial_enabled}"

echo "Args: ${args}"

python3 ${scripts_dir}/xacro.py -o ${rotors_description_dir}/urdf/firefly_base.urdf ${rotors_description_dir}/urdf/firefly_base.xacro ${args}

[[ -e $(pwd)/models/firefly/firefly.sdf ]] && rm $(pwd)/models/firefly/firefly.sdf
gz sdf -p  ${rotors_description_dir}/urdf/firefly_base.urdf >> $(pwd)/models/firefly/firefly.sdf

rm ${rotors_description_dir}/urdf/firefly_base.urdf

# build HIL model
serial_device='/tmp/ttyACM0'
serial_enabled="true"
hil_mode="true"
args="enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_wind:=${enable_wind} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir} send_vision_estimation:=${send_vision_estimation} send_odometry:=${send_odometry} hil_mode:=${hil_mode} serial_device:=${serial_device} serial_enabled:=${serial_enabled}"
python3 ${scripts_dir}/xacro.py -o ${rotors_description_dir}/urdf/firefly_base.urdf ${rotors_description_dir}/urdf/firefly_base.xacro ${args}

[[ -e $(pwd)/models/firefly_hil/firefly_hil.sdf ]] && rm $(pwd)/models/firefly_hil/firefly_hil.sdf
gz sdf -p  ${rotors_description_dir}/urdf/firefly_base.urdf >> $(pwd)/models/firefly_hil/firefly_hil.sdf

rm ${rotors_description_dir}/urdf/firefly_base.urdf
