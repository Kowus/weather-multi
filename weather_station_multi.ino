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
const long interval = 1000;
const long postInterval = 20000;

unsigned long previousMillis = 0;
unsigned long postMillis = 0;
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
  Serial.println("Initializing Weather Shield");
  pinMode(LIGHT, INPUT);
  pinMode(SS, OUTPUT);
  pinMode(fileLed, OUTPUT);

  myPressure.begin();
  myPressure.setModeBarometer();
  myPressure.setOversampleRate(7);
  myPressure.enableEventFlags();

  myHumidity.begin();
  Serial.println("\n\nWeather Shield Initialized\n\nInitializing SD Card");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("\n\nInitialization done.");

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
      /*





        Serial.println("Writing to File");
        Serial.println(output);
        digitalWrite(fileLed, HIGH);
        myFile = SD.open("hive00.log", FILE_WRITE);
        if (myFile) {
        myFile.println(output);
        myFile.close();
        Serial.println("Success");

        } else {
        Serial.println("error opening log file.");
        }*/

      lightRead += get_light_level();
      humRead += humidity;
      dirRead += get_wind_direction();
      pressRead += myPressure.readPressure();
      tempRead += myHumidity.readTemperature();
      digitalWrite(fileLed, LOW);
      iterations += 1;
      Serial.println("Step:\t"+String(iterations));
      previousMillis = currentMillis;
    }
  } else if (currentMillis - postMillis >= postInterval) {
    char output[300] = "";
    root.set<float>("light", float(lightRead) / iterations); // Light
    root.set<int>("wind_dir", dirRead / iterations); // Wind Direction
//    root.set<float>("wind_spd", 0.00);  // Wind Speed
    root.set<float>("pressure", float(pressRead)/iterations);    // pressure
    root.set<float>("temperature", float(tempRead)/iterations);      // temperature
//    root.set<float>("rain", 0.00);      // rainfall
    root.set<float>("humidity", float(humRead)/iterations);      // humidity
    root.printTo(output);
        Serial.println();
    lightRead = humRead = dirRead = pressRead = tempRead = iterations = 0;
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
