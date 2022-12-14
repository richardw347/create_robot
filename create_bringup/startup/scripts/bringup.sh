#!/bin/zsh

# might want to change fixed username to something else later
ENV_LOCATION=/home/jetson/.zshrc
source ${ENV_LOCATION}

roslaunch create_bringup shoebot.launch

bash -i
