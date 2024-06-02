##
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch mavros px4.launch  fcu_url:=/dev/ttyACM0:921600; exec bash"' \
--tab -e 'bash -c "sleep 6; source ~/projects/Autoland_ws/devel/setup.sh;roslaunch px4_command px4_s.launch; exec bash"' \
--tab -e 'bash -c "sleep 7; source ~/projects/Autoland_ws/devel/setup.sh;roslaunch px4_command p450_indoor_landing_static_target.launch; exec bash"' \
