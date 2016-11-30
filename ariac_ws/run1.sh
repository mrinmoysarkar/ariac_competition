source ./devel/setup.bash

rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/comp_conf2.yaml ~/ariac_competition/ariac_ws/src/ariac_example/config/sample_gear_conf.yaml -o /tmp

#sleep 10s
#gnome-terminal -e rosrun ariac_example ariac_example_node
