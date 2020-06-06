#include <WiFi.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#define SEALEVELPRESSURE_HPA (1013.25)


Adafruit_BME280 bme; // I2C

// replace with your wifi ssid and wpa2 key
const char *ssid =  "xxxxx";                                    
const char *pass =  "xxxxx";

//To get Network time
String formattedDate;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// mqtt_server IP
  const char* mqtt_server = "192.168.0.102  ";


WiFiClient espClient;
PubSubClient client(espClient);

void setup() 
{
    Serial.begin(115200);
    delay(10);
    if (!bme.begin(0x76)) {
          Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
    }
    Serial.println("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    timeClient.begin();
    timeClient.setTimeOffset(19080);
    client.setServer(mqtt_server, 1883);
}


//connecting to MQTT server
void reconnect() {
    // Loop until we're reconnected     
    while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
      if (client.connect("Esp32Client")) 
      {
        Serial.println("connected");
     
      } else {
        Serial.print("Failed Satus");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
  }
}


void loop() 
{
  float  TemperatureC = 0;
  float  Humidity =0;
  float  Pressure = 0;
  float  Altitude = 0;
  StaticJsonBuffer<300> JSONbuffer;


  while(!timeClient.update()) {
      timeClient.forceUpdate();
    }
         
    formattedDate = timeClient.getFormattedDate();
    //Serial.println(formattedDate);
  
    if (!client.connected()) {
        reconnect();
      }
    client.loop();             
    
    JsonObject& JSONencoder = JSONbuffer.createObject();
    
    TemperatureC = bme.readTemperature();
    Humidity = bme.readHumidity(); 
    
    JSONencoder["ID"] = "Esp32";
    JSONencoder["Date"] = formattedDate;
    JSONencoder["Tem"] = bme.readTemperature();
    JSONencoder["Hum"] = bme.readHumidity();
    JSONencoder["Per"] = bme.readPressure();
    JSONencoder["Alt"]= bme.readAltitude(SEALEVELPRESSURE_HPA);
    char JSONmessageBuffer[290];
    
    JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
     
     Serial.println("%. Sending to MQTT Broker........");
     client.publish("Weather/ESP32",JSONmessageBuffer);
     Serial.println(JSONmessageBuffer);
    delay(5000);

}
