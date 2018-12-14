#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <DHT.h>

#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"

#include "Math.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred



float beatsPerMinute;
int beatAvg;
int contact;

byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
int sampleRate = 1000; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411; //Options: 69, 118, 215, 411
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384


float FIFORedValue;
float FIFOIRValue;
float FIFOGreenValue;




#if SIMULATED_DATA

void initSensor()
{
    // use SIMULATED_DATA, no sensor need to be inited
}

float readTemperature()
{
    return random(20, 30);
}

float readHumidity()
{
    return random(30, 40);
}

#else

static DHT dht(DHT_PIN, DHT_TYPE);
void initSensor()
{
    dht.begin();
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
    
}

float readTemperature()
{
    return dht.readTemperature();
}

float readHumidity()
{
    return dht.readHumidity();
}

float readHeartRate()
{
    long irValue = particleSensor.getIR();

  if (irValue > 5000)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / ((delta/4) / 1000.0);
    Serial.print(beatsPerMinute);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
        Serial.print(beatAvg);
      beatAvg /= RATE_SIZE;
    }

    
  }


  

  if (irValue < 50000)
    {
    Serial.print(" The sensor is not in contact ");
    contact = 1;
     
    }
  Serial.println();
  return beatsPerMinute;
}

float readSmoke()
{     int samplesTaken = 0;
      if (particleSensor.available()) 
      {
        
        
        FIFORedValue = particleSensor.getFIFORed();
            
        
        
        FIFOIRValue = particleSensor.getFIFOIR();
        
      
        
        FIFOGreenValue = particleSensor.getFIFOGreen();
        
        

        

        samplesTaken++;

      }

        float rootmeansquared = FIFORedValue*FIFORedValue + FIFOIRValue*FIFOIRValue + FIFOGreenValue*FIFOGreenValue;
        Serial.println("rootmeansquared: ");
        Serial.println(sqrt(rootmeansquared));

        return sqrt(rootmeansquared/3.0);

        
}

#endif

bool readMessage(int messageId, char *payload)
{
    float temperature = readTemperature();
    float humidity = readHumidity();

    contact = 0;
    float HeartRate = readHeartRate();
    float SmokeRate = readSmoke();
    
    StaticJsonBuffer<MESSAGE_MAX_LEN> jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root["deviceId"] = DEVICE_ID;
    root["messageId"] = messageId;
    bool temperatureAlert = false;

    // NAN is not the valid json, change it to NULL
    if (std::isnan(temperature))
    {
        root["temperature"] = NULL;
    }
    else
    {
        root["temperature"] = temperature;
        if (temperature > TEMPERATURE_ALERT)
        {
            temperatureAlert = true;
        }
    }

    if (std::isnan(humidity))
    {
        root["humidity"] = NULL;
    }
    else
    {
        root["humidity"] = humidity;
    }

       if (std::isnan(HeartRate))
    {
        root["heartRate"] = NULL;
    }
    else
    {
        root["heartRate"] = HeartRate;
    }

      if (std::isnan(contact))
    {
        root["contact"] = NULL;
    }
    else
    {
        root["contact"] = contact;
    }
    

       if (std::isnan(SmokeRate))
    {
        root["smokeRate"] = NULL;
    }
    else
    {
        root["smokeRate"] = SmokeRate;
    }
   
    root.printTo(payload, MESSAGE_MAX_LEN);
    return temperatureAlert;
}

void parseTwinMessage(char *message)
{
    StaticJsonBuffer<MESSAGE_MAX_LEN> jsonBuffer;
    JsonObject &root = jsonBuffer.parseObject(message);
    if (!root.success())
    {
        Serial.printf("Parse %s failed.\r\n", message);
        return;
    }

    if (root["desired"]["interval"].success())
    {
        interval = root["desired"]["interval"];
    }
    else if (root.containsKey("interval"))
    {
        interval = root["interval"];
    }
}
