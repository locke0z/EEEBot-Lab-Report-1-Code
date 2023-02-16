//****************************************//
//* ESP32 MQTT EEEBot Template           *//
//* Modified from Rui Santos'            *//
//* https://randomnerdtutorials.com      *//
//*                                      *//
//* ESP32 Code                           *//
//*                                      *//
//* UoN 2022 - ND                        *//
//****************************************//

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>


// Add your required sensor/component libraries here
// --
#include <MPU6050_tockn.h>
#define SDA_PIN  21  
#define SCL_PIN  22  

#define ledPin 5
// --
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
// Replace the next variables with your SSID/Password combination
const char* ssid = "c14cry";                      //CHANGE ME
const char* password = "rtfmrtfm1";              //CHANGE ME     

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";                      
const char* mqtt_server = "192.168.137.3";          //CHANGE ME

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
float dis;

void setup() 
{
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(ledPin,OUTPUT);
}

void setup_wifi() 
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) 
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageZGyro;
  
  for (int i = 0; i < length; i++) 
  {
    Serial.print((char)message[i]);
    messageZGyro += (char)message[i];
  }
  Serial.println();

  // Add your subscribed topics here i.e. statements to control GPIOs with MQTT
  // --
   if (String(topic) == "esp32/output") 
   {
    Serial.print("Changing output to ");
    if(messageZGyro == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageZGyro == "off")
    {
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
  // --
}

void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) 
    {
      Serial.println("connected");
      
      // Add your subscribe topics here
      // --
      client.subscribe("esp32/output");
      // --
         
    } else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() 
{
  if (!client.connected()) 
  {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 50) 
  {
    lastMsg = now;
    
    // Add your own code here i.e. sensor measurements, publish topics & subscribe topics for GPIO control
    // --
    Z=mpu6050.getAngleZ();
    char ZString[8];
    dtostrf(Z, 1, 2, ZString);
    Serial.print("Orientation: ");
    Serial.println(ZString);
    client.publish("esp32/GyroZ", ZString);
    // --
  }
}
