#! /bin/bash
# sudo ethercatctl stop
#rosrun ethercat_device_configurator 6G-life src/ethercat_device_configurator/example_config/endoWrist.yaml
# sudo bash -c 'source devel/setup.bash; devel/lib/ethercat_device_configurator/6G-life src/ethercat_device_configurator/example_config/endoWrist.yaml'
executable="standalone"
config_file="endoWrist.yaml"
echo "Sourcing devel/setup.bash..."
source devel/setup.bash
echo "Running ethercat_device_configurator executable..."
devel/lib/ethercat_device_configurator/$executable src/ethercat_device_configurator/example_config/$config_file