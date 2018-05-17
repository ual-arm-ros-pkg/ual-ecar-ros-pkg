
# Timestap of the microcontroller, in millisecs since boot
uint32  timestamp_ms
# PWM
int32   Steer_controller_signal
# Encoder Signal
float64   Encoder_controller_signal
# Encoder Absolute
int32     Encoder_Absoluto
# Encoder Incremental
float64   Encoder_Incremental
# ADC value for steer motor current sensor (Volts)
float64   Steer_ADC_current_sense
# Mechanical limit protection activated
bool steer_mech_limit_reached
#
float64 enc_offset_correction