#include <Wire.h>
#include <SPI.h>
#include <WiFiManager.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <ArduCAM.h>
//MQTT
#include <PubSubClient.h>


#define MQTT_AUTH false
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""

int IN1 = 12;
int IN2 = 2;
int IN3 = 25;
int IN4 = 26;
bool stopEngine = true;

WiFiClient wclient;
const char* MQTT_SERVER = "10.20.228.220";
PubSubClient client(MQTT_SERVER,1883,wclient);
const String MQTT_CONTROL_TOPIC = "car/control/set";
const char* HOSTNAME  = "MQTTCAR";

const int CS = 16;

WebServer server(80);
ArduCAM myCAM(OV2640, CS);

static const size_t bufferSize = 4096;
static uint8_t buffer[bufferSize] = {0xFF};
uint8_t temp = 0, temp_last = 0;
int i = 0;
bool is_header = false;


void start_capture(){
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
}


void serverStream(){
WiFiClient client = server.client();

String response = "HTTP/1.1 200 OK\r\n";
response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
server.sendContent(response);

while (1){
start_capture();
while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
size_t len = myCAM.read_fifo_length();
if (len >= MAX_FIFO_SIZE) //8M
{
Serial.println(F("Over size."));
continue;
}
if (len == 0 ) //0 kb
{
Serial.println(F("Size is 0."));
continue;
} 
myCAM.CS_LOW();
myCAM.set_fifo_burst();
if (!client.connected()) break;
response = "--frame\r\n";
response += "Content-Type: image/jpeg\r\n\r\n";
server.sendContent(response); 
while ( len-- )
{
temp_last = temp;
temp =  SPI.transfer(0x00);

//Read JPEG data from FIFO
if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
{
buffer[i++] = temp;  //save the last  0XD9     
//Write the remain bytes in the buffer
myCAM.CS_HIGH();; 
if (!client.connected()) break;
client.write(&buffer[0], i);
is_header = false;
i = 0;
}  
if (is_header == true)
{ 
//Write image data to buffer if not full
if (i < bufferSize)
buffer[i++] = temp;
else
{
//Write bufferSize bytes image data to file
myCAM.CS_HIGH(); 
if (!client.connected()) break;
client.write(&buffer[0], bufferSize);
i = 0;
buffer[i++] = temp;
myCAM.CS_LOW();
myCAM.set_fifo_burst();
}        
}
else if ((temp == 0xD8) & (temp_last == 0xFF))
{
is_header = true;
buffer[i++] = temp_last;
buffer[i++] = temp;   
} 
}
if (!client.connected()){
  Serial.println("CLOSE");
  break;
  } 
}
}

void handleNotFound(){
String message = "Server is running!\n\n";
message += "URI: ";
message += server.uri();
message += "\nMethod: ";
message += (server.method() == HTTP_GET)?"GET":"POST";
message += "\nArguments: ";
message += server.args();
message += "\n";
server.send(200, "text/plain", message);
}
TaskHandle_t Mqtt;
TaskHandle_t Streaming;
void mqtt( void * parameter )
{
  for (;;){
     if (WiFi.status() == WL_CONNECTED) {
    if (checkMqttConnection()){
      client.loop();
    }
     }
    }
}
void streaming( void * parameter )
{
  for (;;){
if (WiFi.status() == WL_CONNECTED) {
    server.handleClient(); 
    }
  }
}
void setup() {
pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
   pinMode(IN4, OUTPUT);
  
    //MOTORS
  xTaskCreatePinnedToCore(
    mqtt,            /* Task function. */
    "MQTT",                 /* name of task. */
    3000,                    /* Stack size of task */
    NULL,                     /* parameter of the task */
    1,                        /* priority of the task */
    &Mqtt,                   /* Task handle to keep track of created task */
    1);                    /* Core */
  delay(500);
  xTaskCreatePinnedToCore(
    streaming,            /* Task function. */
    "STREAM",                 /* name of task. */
    3000,                    /* Stack size of task */
    NULL,                     /* parameter of the task */
    1,                        /* priority of the task */
    &Streaming,                   /* Task handle to keep track of created task */
    0);                    /* Core */
  delay(500);

uint8_t vid, pid;
uint8_t temp;
pinMode(CS,OUTPUT);

//I2C START SDA, SCL
  Wire.begin(4,5);
  //display.init();
  //display.flipScreenVertically();
  //display.setFont(ArialMT_Plain_10);
Serial.begin(115200);

// initialize SPI: SCK, MISO, MOSI, SS
SPI.begin(14,0,13,16);
SPI.setFrequency(4000000); //4MHz

//Check if the ArduCAM SPI bus is OK
myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
temp = myCAM.read_reg(ARDUCHIP_TEST1);
if (temp != 0x55){
Serial.println(F("SPI1 interface Error!"));
while(1);
}

//Check if the camera module type is OV2640
myCAM.wrSensorReg8_8(0xff, 0x01);
myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 )))
Serial.println(F("Can't find OV2640 module!"));
else
Serial.println(F("OV2640 detected."));


//Change to JPEG capture mode and initialize the OV2640 module
myCAM.set_format(JPEG);
myCAM.InitCAM();

myCAM.OV2640_set_JPEG_size(OV2640_320x240);
myCAM.clear_fifo_flag();
 //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(180);
  
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect("ESP32_CAM")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
  } 
// Start the server
server.on("/stream", HTTP_GET, serverStream);
server.onNotFound(handleNotFound);
server.begin();
Serial.println(F("Server started"));
 client.setCallback(callback);
}

//Chamada de recepção de mensagem 
void callback(char* topic, byte* payload, unsigned int length) {
 
  String payloadStr = "";
  for (int i=0; i<length; i++) {
    payloadStr += (char)payload[i];
  }
Serial.println(payloadStr);
  if(payloadStr.equals("STOP")){
    stopEngine = true;
    stopMotors();
    return;
  }else   if(payloadStr.equals("START")){
    stopEngine = false;
    return;
  }else if(payloadStr.equals("RIGHT")){
     frontRight();
    }else if(payloadStr.equals("LEFT")){
      frontLeft();
    }else if(payloadStr.equals("FRONT")){
      front();
    }
}
bool checkMqttConnection(){
  if (!client.connected()) {
    if (MQTT_AUTH ? client.connect(HOSTNAME,MQTT_USERNAME, MQTT_PASSWORD) : client.connect(HOSTNAME)) {
      //SUBSCRIÇÃO DE TOPICOS
      Serial.println("CONNECTED");
      client.subscribe(MQTT_CONTROL_TOPIC.c_str());
    }
  }
  return client.connected();
}

void loop() {
   
  delay(100);
  }

void front(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3,  LOW);
  digitalWrite(IN4, HIGH);
  showDirection("Front");
 
}
void rear(){
 digitalWrite(IN1,  LOW);
 digitalWrite(IN2, HIGH);
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4,  LOW);
  showDirection("Rear");   
} 

void stopMotors(){
 //Para o motor A
 digitalWrite(IN1,  LOW);
 digitalWrite(IN2,  LOW);
  //Para o motor B
 digitalWrite(IN3,  LOW);
 digitalWrite(IN4,  LOW);
 showDirection("Stop");
}
void frontLeft(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,  LOW);
  digitalWrite(IN3,  LOW);
  digitalWrite(IN4,  LOW);
  showDirection("Front Left");
}
void frontRight(){
  digitalWrite(IN1,  LOW);
  digitalWrite(IN2,  LOW);
  digitalWrite(IN3,  LOW);
 digitalWrite(IN4, HIGH);
  showDirection("Front Right");
}
