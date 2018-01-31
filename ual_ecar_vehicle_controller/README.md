ual_ecar_vehicle_controller
==================

This package contains a ROS node for UAL eCAR's steer driver.

Examples of use & demos:
  * Launch:
```
       roscore     # In one terminal
       rosrun ual_ecar_vehicle_controller ual_ecar_vehicle_controller_node _CURRENT_LIMIT_THRESHOLD_VOLT:=1.5 _CURRENT_LIMIT_TIME:=0.75   # In another terminal
```

  * Getting the controller / encoder status / motor current:

       rostopic echo /steer_controller_status

  * Publishing a PWM value:

      rostopic pub -v -r 10 /steer_controller_pwm std_msgs/Float64 0.5

## How to program the uC via Atmel JTAG ICE programmer from Ubuntu

	avrdude -p m164p -c jtag2 -U flash:w:"ual_ecar_vehicle_controller/firmware/Release/vehicle_controller_firmware.hex":i
	
JL: Please, check if the "-c XXX" is another one, depending on the specific programmer model. 
See this: http://www.ladyada.net/learn/avr/avrdude.html
