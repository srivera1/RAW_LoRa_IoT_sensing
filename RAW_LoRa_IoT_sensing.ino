/*
  IoT raw Lora sender: Take and send readings from sensors
  By: Sergio Rivera Lavado
  Date: April 2018
  https://github.com/srivera1/RAW_LoRa_IoT_sensing

  Hardware (always at 3.3V)
  -STM32 board (Blue pill)
  -Lora module (RF96)
  -MAX30105            (will also run at 3.3V)
  -Adafruit_VEML6070
  -BME280I2C
  -Capacitive Soil Moisture Sensor

  Hardware Connections
  -5V input power from USB (from power bank)
  -3.3V ouput power from STM32 board to LoRa module
  -GND = GND
  -SDA = A4 (or SDA)     // i2c for sensors
  -SCL = A5 (or SCL)     // i2c for sensors
  -MOSI = PA7            // LoRa radio
  -MISO = PA6            // LoRa radio
  -csPin = PB5;          // LoRa radio chip select
  -resetPin = PB4;       // LoRa radio reset
  -irqPin = PB3;         // LoRa hardware interrupt pin
  -Analog pin PA0        // Capacitive Soil Moisture Sensor

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
*/

#include <Wire.h>
#include "MAX30105.h"
#include "Adafruit_VEML6070.h"
#include "SparkFunHTU21D.h"
#include <BME280I2C.h>
#include <SPI.h>                // include libraries
#include <LoRa.h>

int samplesToAverage = 128;     // a few extra bits

const int csPin = PB5;          // LoRa radio chip select
const int resetPin = PB4;       // LoRa radio reset
const int irqPin = PB3;         // change for your board; must be a hardware interrupt pin
String outgoing;                // outgoing message
byte msgCount = 0;              // count of outgoing messages
byte localAddress = 0xAC;       // address of this device
byte destination = 0xAD;        // destination to send to
char endLine = '\n';


Adafruit_VEML6070 uv = Adafruit_VEML6070();
HTU21D myHumidity;
BME280I2C::Settings settings(
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::Mode_Forced,
  BME280::StandbyTime_20ms,
  BME280::Filter_Off,
  BME280::SpiEnable_False,
  0x76 // I2C address. I2C specific.
);
BME280I2C bme(settings);
MAX30105 particleSensor;

long startTime;
long samplesTaken = 0;            //Counter for calculating the Hz or read rate
bool selectExp = true;



void setup() {
  Serial2.begin(115200);
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
  if (selectExp) {
    setupWeather();
  } else {
    setupParticles();
  }
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
  if (!LoRa.begin(868E6)) {             // initialize ratio at 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  /**/
  // TODO: implement more modes
  int Mode = -2;
  if (Mode == -1) { // XX bps
    LoRa.setSignalBandwidth(62.5E3);
    LoRa.setSpreadingFactor(7);           // ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(8);
    LoRa.setPreambleLength(17);
  }
  else if (Mode == -2) { // 43 bps
    LoRa.setSignalBandwidth(41.7E3);
    LoRa.setSpreadingFactor(8);           // ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(8);
    LoRa.setPreambleLength(17);
  }
  else if (Mode == -3) { // 43 bps
    LoRa.setSignalBandwidth(20.8E3);
    LoRa.setSpreadingFactor(9);           // 7,8,9  ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(8);
    LoRa.setPreambleLength(17);
  }
  else if (Mode == -4) { // 43 bps
    LoRa.setSignalBandwidth(15.6E3);
    LoRa.setSpreadingFactor(9);           // 7,8,9  ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(17);
  }
  else if (Mode == 0) { // 43 bps
    LoRa.setSignalBandwidth(20.8E3);
    LoRa.setSpreadingFactor(11);           // ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(8);
    LoRa.setPreambleLength(17);
  }
  else if (Mode == 1) { // 1476 bps
    LoRa.setSignalBandwidth(20.8E3);
    LoRa.setSpreadingFactor(6);           // ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(17);
  }
  else if (Mode == 2) { // 915 bps
    LoRa.setSignalBandwidth(62.5E3);
    LoRa.setSpreadingFactor(8);           // ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(8);
    LoRa.setPreambleLength(17);
  }
  else if (Mode == 3) { // 8509 bps
    LoRa.setSignalBandwidth(250.0E3);
    LoRa.setSpreadingFactor(7);           // ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(5); // check this value
    LoRa.setPreambleLength(17);
  }
  else if (Mode == 4) { // 17738 bps
    LoRa.setSignalBandwidth(250.0E3);
    LoRa.setSpreadingFactor(6);           // ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(17);
  }
  else if (Mode == 5) { // 104 bps
    LoRa.setSignalBandwidth(41.7E3);
    LoRa.setSpreadingFactor(11);           // ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(8);
    LoRa.setPreambleLength(17);
  }
  else if (Mode == 6) { // 2959 bps
    LoRa.setSignalBandwidth(41.7E3);
    LoRa.setSpreadingFactor(6);           // ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(17);
  }
  else if (Mode == 7) { // 841 bps
    LoRa.setSignalBandwidth(20.8E3);
    LoRa.setSpreadingFactor(7);           // ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(17);
  }
  else if (Mode == 8) { // 4435 bps
    LoRa.setSignalBandwidth(62.5E3);
    LoRa.setSpreadingFactor(6);           // ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(17);
  }
  else if (Mode == 9) {
    // not supported
    LoRa.setSignalBandwidth(500.0E3);
    LoRa.setSpreadingFactor(6);           // ranges from 6-12,default 7 see API docs
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(17);
  }
  /**/
  Serial.println("LoRa init succeeded.");
}
int i = 0;
void loop() {
  if (selectExp) {
    loopWeather();
  } else {
    loopParticles();
  }
  i++;
  if (i == -1) {
    Serial.print(selectExp);
    selectExp = not selectExp;
    setup();
    i = 0;
  }
}

void setupWeather() {
  Serial.begin(115200);           // open serial port, set the baud rate to 9600 bps
  delay(110);
  Serial.println("Setup init");
  delay(110);
  //uv.begin(VEML6070_1_T);       // pass in the integration time constant
  myHumidity.begin(Wire);

  //Wire.begin();                 // pass in the integration time constant
  delay(110);
  //  particleSensor.setup();     //Configure sensor. Use 6.4mA for LED drive
  delay(110);
  bme.begin();
  settings.tempOSR = BME280::OSR_X4;
  delay(110);

  bme.setSettings(settings);
  delay(110);
}

void setupParticles()
{
  Serial.begin(115200);           // open serial port, set the baud rate
  delay(110);
  Serial.println("Initializing...");



  //Setup to sense up to 18 inches, max LED brightness

  byte ledBrightness = 0xFF;      //Options: 0=Off to 255=50mA
  byte sampleAverage = 4;         //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;               //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 400;           //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;           //Options: 69, 118, 215, 411
  int adcRange = 2048;            //Options: 2048, 4096, 8192, 16384

  //particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  //Configure sensor with these settings
  particleSensor.setup();        //Configure sensor. Use 6.4mA for LED drive

  Serial.println("Initializing...2");
  startTime = millis();
}

void loopWeather() {
  float temp(NAN), hum(NAN), pres(NAN);

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pres, temp, hum, tempUnit, presUnit);

  int i;
  float soilMoistureValue = 0;
  int uvValue = 0;
  float tempValue = 0;
  float rhValue = 0;
  float temp2Value = 0;
  float pressValue = 0;
  for (i = 1 ; i < samplesToAverage ; i++)
  {
    soilMoistureValue += analogRead(PA0);  //put Sensor insert into soil
    delay(3);
    uvValue += uv.readUV();
    delay(3);
    tempValue += myHumidity.readTemperature();
    delay(3);
    rhValue += myHumidity.readHumidity();
    delay(3);
    bme.read(pres, temp, hum, tempUnit, presUnit);
    temp2Value += temp;
    pressValue += pres;
  }
  //soilMoistureValue/=i;



  String content = "";

  //content.concatln("UV light level, soilMoistureValue, Relative Humidity, Temperature1, Temperature1, Pressure kPa");
  content.concat(String(uvValue / (i - 1), 4));
  content.concat(",");
  content.concat(String(soilMoistureValue  / (i - 1), 4));
  content.concat(",");
  content.concat(String(rhValue / (i - 1), 4));
  content.concat(",");
  content.concat(String(tempValue / (i - 1), 4));
  content.concat(",");
  content.concat(String(temp2Value / (i - 1), 4));
  content.concat(",");
  content.concat(String(pressValue / (i - 1), 4));
  content.concat(";");/*
  //content.concat(particleSensor.getRed());
  content.concat(",");
  //content.concat(particleSensor.getIR());
  content.concat(",");
  //content.concat(particleSensor.getGreen());
  content.concat(",");*/
  int contentSize = content.length();
  // FEC, error correction "best 2 out of 3"

  content = String(contentSize) + ';' + content + content + content;
  // TODO: implementation of Hamming codes or Reed-Solomon codes

  while (content.length() < 256)
    content.concat(".");


  content.concat("\n");

  Serial.print(content);
  sendMessage(content);      // manda el string gps

}

void loopParticles()
{
  particleSensor.check(); //Check the sensor, read up to 3 samples

  while (particleSensor.available()) //do we have new data?
  {
    samplesTaken++;

    Serial.print(" R[");
    Serial.print(particleSensor.getFIFORed());
    Serial.print("] IR[");
    Serial.print(particleSensor.getFIFOIR());
    Serial.print("] G[");
    Serial.print(particleSensor.getFIFOGreen());
    Serial.print("] Hz[");
    Serial.print((float)samplesTaken / ((millis() - startTime) / 1000.0), 2);
    Serial.print("]");

    Serial.println();

    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }
}

void sendMessage(String outgoing) {
  LoRa.beginPacket(true);                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

