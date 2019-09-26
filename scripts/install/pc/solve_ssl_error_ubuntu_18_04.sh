#!/bin/bash

# replace text ignitionfuel by ignitionrobotics on ignition config file
sed -i 's/ignitionfuel/ignitionrobotics/g' /home/$USER/.ignition/fuel/config.yaml