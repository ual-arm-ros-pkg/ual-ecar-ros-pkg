# This message holds the ADC output for one sampling period
# for claraquino. E.g. if only one ADC channel is
# active, the actual reading will be in adc_data[0].

uint32    timestamp_ms   # Timestap of the microcontroller, in millisecs since boot
uint16[]  adc_data       # ADC raw reading. Multiply by VOLT_REF/1023 to obtain volts
