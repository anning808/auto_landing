##
gnome-terminal --window -e 'bash -c " roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"; exec bash"' \
--tab -e 'bash -c "sleep 1; source ~/projects/Autoland_ws/devel/setup.sh;roslaunch px4_command px4_s.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; source ~/projects/Autoland_ws/devel/setup.sh;roslaunch px4_command p450_indoor_landing_static_target.launch; exec bash"' \
