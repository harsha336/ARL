#!/bin/bash
clear
sudo chmod 777 /dev/tty*
source ~/ARL/devel/setup.bash

# Determine if autonomous navigation will be used or manual control
exit_condition="false"
while [ "$exit_condition" = "false" ]; do
	read -p "Enable autonomous navigation? (y|n): " answer
	case $answer in
		[Yy]* )
			export PIONEER_AUTON_ENABLED="true"
			exit_condition="true";;
		[Nn]* )
			export PIONEER_AUTON_ENABLED="false"
			exit_condition="true";;
		* )
			echo "Please answer yes or no.";;
	esac
done

# Determine if this session will be bagged
if [ "$PIONEER_AUTON_ENABLED" = "true" ]; then
echo
exit_condition="false"
while [ "$exit_condition" = "false" ]; do
        read -p "Should this session be saved? (y|n): " answer
        case $answer in
                [Yy]* )
                        export PIONEER_BAG_ENABLED="true"
                        exit_condition="true";;
                [Nn]* )
                        export PIONEER_BAG_ENABLED="false"
                        exit_condition="true";;
                * )
                        echo "Please answer yes or no.";;
        esac
done
fi

roslaunch pioneer_gmap pioneerStart.launch
