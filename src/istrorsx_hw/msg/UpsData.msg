# Waveshare UPS Power Module (C) battery telemetry (INA219 over I2C) --
# published by ups_node. No legacy equivalent -- this hardware isn't part of
# istro_rt2025.cpp, see ups_node's own comment.
float64 bus_voltage      # volts, load-side (INA219 BUS VOLTAGE register)
float64 shunt_voltage    # volts, across the shunt resistor (SHUNT VOLTAGE register)
float64 current          # amps (CURRENT register)
float64 power             # watts (POWER register)
float64 percentage       # 0-100, estimated from bus_voltage -- (bus_voltage-9)/3.6*100,
                          # clamped, same formula as ina219.py's own __main__ (9V=empty,
                          # 12.6V=full for this pack)
