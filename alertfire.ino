// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

#include "DHT.h"
#include <UIPEthernet.h>
#include <PubSubClient.h>


#define DHTPIN 2     // what digital pin we're connected to
#define PIRPIN 3     // what digital pin we're connected to
#define SMKPIN 4     // what digital pin we're connected to
#define FLMPIN 5
#define SMKPINA 0    // what analogic pin we're connected to
#define FLMPINA 1    // what analogic pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);


// Update these with values suitable for your network.
byte mac[] = {  0x75, 0x75, 0xBA, 0xFE, 0xFE, 0xED };

//byte server[] = {52, 58, 157, 180}; this is server broker.hivemq.com
byte server[] = {192, 168, 0, 3};

EthernetClient ethClient;
PubSubClient client(server, 1883, 0, ethClient);

char g_achBuffer[5];
char g_achMenssage[15];


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // client.publish("raimundo-oliveira","hello world");
      // ... and resubscribe
      // client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void setup() {
  Serial.begin(9600);

  pinMode(PIRPIN, INPUT);
  pinMode(SMKPIN, INPUT);
  pinMode(SMKPINA, INPUT);
  pinMode(FLMPINA, INPUT);
  pinMode(FLMPIN, INPUT);
  
  dht.begin();
  
  Ethernet.begin(mac);
  delay(1500);

}

void loop() {


  // Reading smok
  int iSmok = digitalRead( SMKPIN );
  int iSmoka = analogRead( SMKPINA );

  // Reading flame 
  int iFlamea = analogRead( FLMPINA );
  int iFlame = digitalRead( FLMPIN );
  
  //Serial.println( iSmoka );
  //Serial.println( iSmok );
  //Serial.println( iFlame );
  //Serial.println( iFlamea );
  
  // Reading the presence 
  int iPir = digitalRead( PIRPIN );
  
   // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  int iBuffLen = sizeof(g_achBuffer);
  
  // protocolo
  // temperatur
  dtostrf( t, sizeof(float), 2, g_achBuffer );
  memcpy( g_achMenssage, g_achBuffer, iBuffLen );

  // humidi
  memset( g_achBuffer, 0, iBuffLen );
  dtostrf( h, sizeof(float), 2, g_achBuffer );
  memcpy( &g_achMenssage[iBuffLen], g_achBuffer, iBuffLen );

 // presença
  memset( g_achBuffer, 0, iBuffLen );
  dtostrf( iPir, 1, 0, g_achBuffer );
  g_achMenssage[10] = g_achBuffer[0];
  //memcpy( &g_achMenssage[2*iBuffLen], g_achBuffer, 1 );

  // flame 
  memset( g_achBuffer, 0, iBuffLen );
  dtostrf( iFlame, 1, 0, g_achBuffer );
  g_achMenssage[11] = g_achBuffer[0];
 //memcpy( &g_achMenssage[(2*iBuffLen)], g_achBuffer, 1 );

  // smoke
  memset( g_achBuffer, 0, iBuffLen );
  dtostrf( iSmok, 1, 0, g_achBuffer );
  g_achMenssage[12] = g_achBuffer[0];
  
  g_achMenssage[13] = '\0';

  Serial.print( "data:\n" );
  Serial.print( g_achMenssage );
  Serial.print( "\n" );
  Serial.print( "\n" );
  
  if (!client.connected()) {
    //reconnect();
  }
  client.publish("data",g_achMenssage);
  client.loop();

  // Wait a few seconds between measurements.
  delay(10000);

}
