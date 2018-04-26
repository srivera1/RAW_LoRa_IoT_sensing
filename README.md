# RAW_LoRa_IoT_sensing


  IoT raw Lora sender: Take and send readings from sensors
#
  By: Sergio Rivera Lavado
#
  Date: April 2018
#
  https://github.com/
#
  Hardware (always at 3.3V)
#
  -STM32 board (Blue pill)
#
  -Lora module (RF96)
#
  -MAX30105            (will also run at 3.3V)
#
  -Adafruit_VEML6070
#
  -BME280I2C
#
  -Capacitive Soil Moisture Sensor
#
  Hardware Connections
#
  -5V input power from USB (from power bank)
#
  -3.3V ouput power from STM32 board to LoRa module
#
  -GND = GND
#
  -SDA = A4 (or SDA)     // i2c for sensors
#
  -SCL = A5 (or SCL)     // i2c for sensors
#
  -MOSI = PA7            // LoRa radio
#
  -MISO = PA6            // LoRa radio
#
  -csPin = PB5;          // LoRa radio chip select
#
  -resetPin = PB4;       // LoRa radio reset
#
  -irqPin = PB3;         // LoRa hardware interrupt pin
#
  -Analog pin PA0        // Capacitive Soil Moisture Sensor
#
  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
