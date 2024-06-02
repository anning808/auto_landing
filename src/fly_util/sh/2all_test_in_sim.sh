##
gnome-terminal --window -e 'bash -c "cd ~/projects/Firmware && make px4_sitl_default gazebo; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/projects/Autoland_ws/devel/setup.sh;roslaunch px4_command px4_s.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/projects/Autoland_ws/devel/setup.sh;roslaunch px4_command p450_indoor_landing_static_target.launch; exec bash"' \
