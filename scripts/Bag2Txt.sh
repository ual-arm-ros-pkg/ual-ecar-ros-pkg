#!/bin/bash
echo "-------------------"
echo "CONVERSIÓN BAG->TXT"
echo "-------------------"
echo " "
echo "DAQ:Ammeter_value ..."
rostopic echo -b ecarm_* -p /amm/ammeter_value > rostopic_Amm_Ammeter_value.txt
echo "DAQ:Battery_voltaje ..."
rostopic echo -b ecarm_* -p /amm/battery_voltaje > rostopic_Amm_Battery_voltaje.txt
echo "BAT:Ammeter_value ..."
rostopic echo -b ecarm_* -p /bat/ammeter_value > rostopic_Bat_Ammeter_value.txt
echo "BAT:Battery_voltaje ..."
rostopic echo -b ecarm_* -p /bat/battery_voltaje > rostopic_Bat_Battery_voltaje.txt
echo "Claraquino_abs_encoder ..."
rostopic echo -b ecarm_* -p /claraquino_abs_encoder > rostopic_Encoder_Abs.txt
echo "Claraquino_speedcruise_adc ..."
rostopic echo -b ecarm_* -p /claraquino_speedcruise_adc > rostopic_Speedcruise_adc.txt
echo "Claraquino_speedcruise_control_signal ..."
rostopic echo -b ecarm_* -p /claraquino_speedcruise_control_signal > rostopic_Speedcruise_control_signal.txt
echo "Claraquino_speedcruise_encoders ..."
rostopic echo -b ecarm_* -p /claraquino_speedcruise_encoders > rostopic_Speedcruise_encoders.txt
echo "Claraquino_steer_adc ..."
rostopic echo -b ecarm_* -p /claraquino_steer_adc > rostopic_Steer_adc.txt
echo "Claraquino_steer_control_signal ..."
rostopic echo -b ecarm_* -p /claraquino_steer_control_signal > rostopic_Steer_control_signal.txt
echo "Claraquino_steer_encoders ..."
rostopic echo -b ecarm_* -p /claraquino_steer_encoders > rostopic_Steer_encoders.txt
echo "Enc_phidget ..."
rostopic echo -b ecarm_* -p /joint_states > rostopic_Enc_phidget.txt
rostopic echo -b ecarm_* -p /joint_states_ch0_decim_speed > rostopic_Enc_phidget_0.txt
rostopic echo -b ecarm_* -p /joint_states_ch1_decim_speed > rostopic_Enc_phidget_1.txt
rostopic echo -b ecarm_* -p /joint_states_ch2_decim_speed > rostopic_Enc_phidget_2.txt
rostopic echo -b ecarm_* -p /joint_states_ch3_decim_speed > rostopic_Enc_phidget_3.txt
rostopic echo -b ecarm_* -p /joint_states_ch4_decim_speed > rostopic_Enc_phidget_4.txt
rostopic echo -b ecarm_* -p /joint_states_ch5_decim_speed > rostopic_Enc_phidget_5.txt
echo "Joystick_eje_x ..."
rostopic echo -b ecarm_* -p /joystick_eje_x > rostopic_Eje_x.txt
echo "Joystick_eje_y ..."
rostopic echo -b ecarm_* -p /joystick_eje_y > rostopic_Eje_y.txt
echo "Joystick_eje_z ..."
rostopic echo -b ecarm_* -p /joystick_eje_z > rostopic_Eje_z.txt
echo "Odom ..."
rostopic echo -b ecarm_* -p /odom > rostopic_Odom.txt
echo "Tf ..."
rostopic echo -b ecarm_* -p /tf > rostopic_Tf.txt
echo "Vehicle_autonomous_mode ..."
rostopic echo -b ecarm_* -p /vehicle_autonomous_mode > rostopic_Autonomous_mode.txt
echo "Vehicle_brake_enable ..."
rostopic echo -b ecarm_* -p /vehicle_brake_enable > rostopic_Brake_enable.txt
echo "Vehicle_openloop_mode_steering ..."
rostopic echo -b ecarm_* -p /vehicle_openloop_mode_steering > rostopic_OL_Steering.txt
echo "Vehicle_openloop_mode_throttle ..."
rostopic echo -b ecarm_* -p /vehicle_openloop_mode_throttle > rostopic_OL_Throttle.txt
echo " "
echo "------------------"
echo "Proceso finalizado"
echo "------------------"
