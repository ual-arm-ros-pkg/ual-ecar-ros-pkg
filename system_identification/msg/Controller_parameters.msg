# This message holds the readings from (software-based) quadrature encoder
# decoders for arduino_daq.

uint32    timestamp_ms		# Timestap of the microcontroller, in millisecs since boot
uint32    period_ms			# Time, in milliseconds, between consecutive messages. Can be used to estimate velocity
float64[] controller_values	#
