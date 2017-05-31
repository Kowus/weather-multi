#include <Wire.h> //I2C needed for sensors
#include "SparkFunMPL3115A2.h" //Pressure sensor - Search "SparkFun MPL3115" and install from Library Manager
#include "SparkFunHTU21D.h" //Humidity sensor - Search "SparkFun HTU21D" and install from Library Manager
MPL3115A2 myPressure; //Create an instance of the pressure sensor
HTU21D myHumidity; //Create an instance of the humidity sensor
#include <ArduinoJson.h>

// Hardware pin definitions
const byte STAT_BLUE = 7;
const byte STAT_GREEN = 8;
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WDIR = A0;
const byte RAIN = 2;
const byte WSPEED = 3;
const long interval = 1000;
const long postInterval = 120000;

unsigned long previousMillis = 0;
unsigned long postMillis = 0;
volatile unsigned long rainlast = 0;
volatile unsigned long lastWindIRQ = 0;
volatile unsigned long lastWindCheck = 0;
volatile float dailyrainin = 0, thisrainin = 0;
volatile int wind_clicks = 0;

float lightRead = 0, speedRead = 0, pressRead = 0, humRead = 0, tempRead = 0;
int dirRead = 0, iterations = 0;



// SD Library
#include <SPI.h>
#include <SD.h>
// Mega:  MOSI - pin 51, MISO - pin 50, CLK - pin 52, CS - pin 4 (CS pin can be changed)  and pin #52 (SS) must be an output
const int chipSelect = 53;
const int fileLed = 48;

// Create an instance of File
File myFile;

void setup() {
  Serial.begin(115200);
  Serial.print("Initializing Weather Shield:\t");
  pinMode(LIGHT, INPUT);
  pinMode(SS, OUTPUT);
  pinMode(fileLed, OUTPUT);
  pinMode(RAIN, INPUT_PULLUP);
  pinMode(WSPEED, INPUT_PULLUP
  
  attachInterrupt(0, rainIRQ, FALLING);
  attachInterrupt(1, wspeedIRQ, FALLING);
  
  myPressure.begin();
  myPressure.setModeBarometer();
  myPressure.setOversampleRate(7);
  myPressure.enableEventFlags();

  myHumidity.begin();
  Serial.print("success!\n\nInitializing SD Card:\t");

  if (!SD.begin(chipSelect)) {
    Serial.println("failed!");
    return;
  }
  Serial.println("success!");

}

void loop() {
  unsigned long currentMillis = millis();
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  if (currentMillis - previousMillis >= interval) {


    float humidity = myHumidity.readHumidity();

    if (humidity == 998) //Humidty sensor failed to respond
    {
      Serial.println("I2C communication to sensors is not working. Check solder connections.");

      //Try re-initializing the I2C comm and the sensors
      myPressure.begin();
      myPressure.setModeBarometer();
      myPressure.setOversampleRate(7);
      myPressure.enableEventFlags();
      myHumidity.begin();
    } else {
      //      Cummulative addition of readings per turn
      lightRead += get_light_level();
      humRead += humidity;
      dirRead += get_wind_direction();
      pressRead += myPressure.readPressure();
      tempRead += myHumidity.readTemperature();
      digitalWrite(fileLed, LOW);
      //      Number of turns
      iterations += 1;
      Serial.println("Step:\t" + String(iterations));
      //      Reset timer
      previousMillis = currentMillis;
    }
  } else if (currentMillis - postMillis >= postInterval) {
    char output[300] = "";
    root.set<float>("light", float(lightRead) / iterations); // Light
    root.set<int>("wind_dir", dirRead / iterations); // Wind Direction
    //    root.set<float>("wind_spd", 0.00);  // Wind Speed
    root.set<float>("pressure", float(pressRead) / iterations);  // pressure
    root.set<float>("temperature", float(tempRead) / iterations);    // temperature
    root.set<float>("totalrain", float(dailyrainin));      // total rainfall
    root.set<float>("lastrain", float(thisrainin));      // rainfall
    root.set<float>("humidity", float(humRead) / iterations);    // humidity
    root.set<float>("battery", get_battery_level());
    root.printTo(output);
    Serial.println(output);
    
    Serial.print("write_to_file:\t");
    digitalWrite(fileLed, HIGH);
    myFile = SD.open("hive00.log", FILE_WRITE);
    if (myFile) {
      myFile.println(output);
      myFile.close();
      Serial.println("success!");

    } else {
      Serial.println("error!");
    }
    //    Reset values
    lightRead = humRead = dirRead = pressRead = tempRead = iterations =thisrainin= 0;
    postMillis = currentMillis;
  }

}


// Get Light Level function
float get_light_level()
{
  volatile float operatingVoltage = analogRead(REFERENCE_3V3);

  volatile float lightSensor = analogRead(LIGHT);

  operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V

  lightSensor = operatingVoltage * lightSensor;

  return (lightSensor);
}


// Get Wind Direction
int get_wind_direction()
{
  unsigned int adc;

  adc = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  if (adc < 380) return (113);
  else if (adc < 393) return (68);
  else if (adc < 414) return (90);
  else if (adc < 456) return (158);
  else if (adc < 508) return (135);
  else if (adc < 551) return (203);
  else if (adc < 615) return (180);
  else if (adc < 680) return (23);
  else if (adc < 746) return (45);
  else if (adc < 801) return (248);
  else if (adc < 833) return (225);
  else if (adc < 878) return (338);
  else if (adc < 913) return (0);
  else if (adc < 940) return (293);
  else if (adc < 967) return (315);
  else if (adc < 990) return (270);
  else return (-1); // error, disconnected?
}

float get_battery_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);

  float rawVoltage = analogRead(BATT);

  operatingVoltage = 3.30 / operatingVoltage; //The reference voltage is 3.3V

  rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin

  rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage

  return (rawVoltage);
}







// Interrupt Handlers

void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  volatile unsigned long raintime = millis(); // grab current time
  if (raintime - rainlast > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainin += 0.011; //Each dump is 0.011" of water
    thisrainin += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }
}


void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }
}

