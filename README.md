# car-performance-display
esp32 + elm327 + bluetooth + display (ssd1283a) + freertos

* esp32 communicating via Bluetooth to an elm327 plugged into an obd ii (can) port of a vehicle

* the display is controlled by a ssd1283a which communicates with the esp32 via spi

* shows data like oil and coolant temperatures, air pressure and temperature in the intake manifold, timing advance and high-pressure fuel pump pressure

* the maximum values can be reset through a touch sensitive pad

* programmed in c/c++ with freertos