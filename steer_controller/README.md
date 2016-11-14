steer_controller
==================

This package contains a ROS node for UAL eCAR's steer driver ("caja azul").

Examples of use & demos:
  * Launch:

       roscore     # In one terminal
       rosrun steer_controller steer_controller_node    # In another terminal

  * Getting the controller / encoder status:

       rostopic echo /steer_controller_status

  * Publishing a PWM value:

      rostopic pub -v -r 10 /steer_controller_pwm std_msgs/Float64 0.5
