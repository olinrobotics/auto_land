roslaunch auto_land apm2.launch fcu_url:=$1 &

sleep 5

rosservice call /mavros/set_stream_rate 0 10 1

roslaunch auto_land vehicle.launch
