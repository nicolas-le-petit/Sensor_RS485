/**
   This example demonstrates how to combine reading inputs and writing
   outputs, whether they are digital or analog
   It assumes there are 2 push buttons with pullup resistors connected
   to channels C0 and C1, 2 potentiometers connected to channels C2
   and C3, and 4 LEDs+resistors with the positive lead of the LEDs
   connected to channels C4 to C7
   PIN A0 is used for reading the inputs and should connect to the SIG pin
   PIN 3 is used for writing the outputs and should connect to the SIG pin
   Connecting both PIN A0 and PIN 3 to the SIG pin won't create a problem
   because when the pins are not used, they are declared as inputs which
   puts them in a high impedance state

   For more about the interface of the library go to
   https://github.com/pAIgn10/MUX74HC4067

  MUX74HC4067 KEYWORD1// declare the MUX 4-16

  setChannel  KEYWORD2//set channel
  enable  KEYWORD2//enable module
  disable KEYWORD2//disable module
  signalPin KEYWORD2//select the pin read/write signal
  read  KEYWORD2
  write KEYWORD2

  SPI
  MOSI 23  13
  MISO 19  12
  CLK  18  14
  CS   5   15

  Strapping Pins
  The ESP32 chip has the following strapping pins:

  GPIO 0
  GPIO 2
  GPIO 4
  GPIO 5 (must be HIGH during boot)
  GPIO 12 (must be LOW during boot)
  GPIO 15 (must be HIGH during boot)

  DAC(not use): 25,26
  Not use: 0, 6-12, 25, 26, 2, 4, 5, 12, 15, 20, 24, 28-31, 37-38
  //  UART0: 1,3
  UART1(not use):
  //  UART2 (RS485): 17,16
  Input only pins: 34,35,36,39 (can not use PWM)
  SPI flash (not use): 6-11
  I2C: 21,22
  VSPI: 23,18,19,5
  //  HSPI: 13,12,14,15
  All GPIOs can be configured as interrupts.
  Pins HIGH at Boot: 1,3,5,6-11,14,15
  test
*/

//All library here
#include <WiFi.h>
#include "MUX74HC4067.h"
#include "RS485_protocol.h"//support RS485 protocol
#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


#define _DEBUG_//debug mode via Serial
#define _LCD_//defien to use LCD
//All define here
//Serial0 is Debug UART by default

//LCD - I2C
#define LCD_SDA 21
#define LCD_SCL 22
const int lcdColumns = 20;
const int lcdRows = 4;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  //init LCD

//SD Card SPI (HSPI)
#define CS_PIN_SDcard 5
#define CLK_PIN_SDcard 18
#define MOSI_PIN_SDcard 23
#define MISO_PIN_SDcard 19


//74HC4067
#define MUX_EN_PIN 4
#define MUX_PIN_S0 25
#define MUX_PIN_S1 26
#define MUX_PIN_S2 27
//MUX_PIN_S3 ---> GND
MUX74HC4067 mux(MUX_EN_PIN, MUX_PIN_S0, MUX_PIN_S1, MUX_PIN_S2);

//RS485
#define RS485_Port Serial2// TX:17, RX:16
#define RS485_PIN_DE 32//control pin RS485, HIGH to transmit - LOW to receive
#define RS485_TX 17//TX2
#define RS485_RX 16//RX2
#define RS485_TX_MODE() digitalWrite (RS485_PIN_DE, HIGH)
#define RS485_RX_MODE() digitalWrite (RS485_PIN_DE, LOW)

//SIM_Port
#define TX3_PIN 33
#define RX3_PIN 34
#define SIM_Port_PORT SoftSerial//32, 33
SoftwareSerial SoftSerial(RX3_PIN, TX3_PIN);//RX TX

//
#define PIN_RESET_MCU
#define PIN_SELECT_INIT_MODE 35
//Wifi
const char* _ssid     = "HNN_IoT";
const char* _password = "honeynet.vn";
const char* _host = "ais.honeynet.vn";
const int _httpPort = 80;

struct Http_Url_Frame {
  int DevID;
  String Protocol;
  int value1;
  int value2;
  int value3;
  int value4;
  int value5;
  int value6;
  int value7;
  int value8;
};

void setup()
{
  Serial.begin(9600);
  // initialize LCD
  Wire.begin(21, 22);
  #ifdef
  Serial.println("I2C is ready now!");
  #endif

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Hello world!");
  delay(1000);
  lcd.clear();

  #ifdef _LCD_
    lcd.setCursor(3, 0);
    lcd.print("I2C is ready now");
    delay(500);
    lcd.clear();
  #endif 

  /*Serial.begin(9600);*/
  #ifdef _DEBUG_
  Serial.println("Serial 0 is ready now!");
  #endif

  #ifdef _LCD_
    lcd.setCursor(0, 0);//row, col
    lcd.print("Serial0 is ready now");
  #endif
  delay(500);

  Serial2.begin(9600);
  Serial.println("Serial 2 is ready now!");

  #ifdef _LCD_
    lcd.setCursor(1, 0);
    lcd.print("Serial2 is ready now");
  #endif
  delay(500);

  SoftSerial.begin(9600);
  Serial.println("Software Serial is ready now!");

  #ifdef _LCD_
    lcd.setCursor(2, 0);
    lcd.print("Serial3 is ready now");
  #endif
  delay(500);

  pinMode(RS485_PIN_DE, OUTPUT);
  pinMode(MUX_EN_PIN, OUTPUT);
  pinMode(PIN_SELECT_INIT_MODE, INPUT_PULLUP, DIGITAL);

  int _STATE_INIT = digitalRead(PIN_SELECT_INIT_MODE); 
  if (_STATE_INIT == HIGH)//config mode
  {
    //wait Serial
    //if config_wifi
    //if config_device  
  }
  else

  //init Wifi
  Wifi_Init();
  //init HTTP
  //init EEPROM
  //init RTC

  //init timer clock
  //init watchdog
}

void loop()
{
  if (Serial.available()) {
    String rev = Serial.readStringUntil('\n');
    write_serial_by_channel(1, rev);
  }
  read_serial_by_channel(1);
}

/*------------------------------WIFI HANDLE---------------------------*/
void Wifi_Init() {
#ifdef _DEBUG_
  Serial.print("Connecting to ");
  Serial.println(_ssid);
#endif

  WiFi.begin(_ssid, _password);

#ifdef _LCD_
  lcd.setCursor(0, 0);
  lcd.print("Connecting");
#endif

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
#ifdef _LCD_
    lcd.print(".");
#endif
  }
#ifdef _LCD_
  lcd.setCursor(1, 0);
  lcd.print("WiFi connected!");
  lcd.setCursor(2, 0);
  lcd.print("IP address: ");
  lcd.setCursor(3, 0);
  lcd.print((String)WiFi.localIP());
  delay(1000);
#endif

#ifdef _DEBUG_
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
#endif
}

void send_data_to_cloud_via_WIFI(String sensor_data)
{
#ifdef _DEBUG_
  Serial.print("connecting to ");
  Serial.println(_host);
#endif

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  if (!client.connect(_host, _httpPort)) {
    Serial.println("connection failed");
    return;
  }

  // We now create a URI for the request
  String url = "/input/";
//  url += streamId;
//  url += "?private_key=";
//  url += privateKey;
//  url += "&value=";
//  url += value;

#ifdef _DEBUG_
  Serial.print("Requesting URL: ");
  Serial.println(url);
#endif

  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + _host + "\r\n" +
               "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

  // Read all the lines of the reply from server and print them to Serial
  while (client.available()) {
    String line = client.readStringUntil('\r');
  #ifdef _DEBUG_
      Serial.print(line);
  #endif
  }

  #ifdef _DEBUG_
    Serial.println();
    Serial.println("closing connection");
  #endif
}
/*----------------------------------------------------------------------*/

/*----------------------SERIAL COMMUNICATION HANDLE---------------------*/
//channel will be 0-7
void read_serial_by_channel(uint8_t channel) {
  //Select the pin
  uint8_t tx_pin_slave = channel;

  mux.setChannel(tx_pin_slave);//select the pin
  mux.enable();
  mux.signalPin(RS485_RX, INPUT_PULLUP, DIGITAL);//select pin to read data

  RS485_RX_MODE();
  if (RS485_Port.available()) {
    char incoming_data = RS485_Port.read();
    Serial.println(incoming_data);
  }
  mux.disable();
}

void write_serial_by_channel(uint8_t channel, String data_frame) {
  //Select the pin
  uint8_t rx_pin_slave = channel;

  mux.setChannel(rx_pin_slave);
  mux.enable();
  mux.signalPin(RS485_TX, OUTPUT, DIGITAL);//select pin to write data

  RS485_TX_MODE();
  RS485_Port.println(data_frame);
  RS485_RX_MODE();
  mux.disable();
}

//RS485 lib
//void fWrite (const byte what)
//{
//  RS485_Port.write (what);
//}
//
//int fAvailable ()
//{
//  return RS485_Port.available ();
//}
//
//int fRead ()
//{
//  return RS485_Port.read ();
//}
//
//void transmit_RS485(byte msg[]) {
//  // gửi đến slave
//  digitalWrite (RS485_PIN_DE, RS485_Transmit);  // cho phép gửi
//  sendMsg (fWrite, msg, sizeof msg);
//  digitalWrite (RS485_PIN_DE, RS485_Receive);  // không cho phép gửi
//}
//
//String receive_RS485(){
//
//}
/*------------------------------------------------------------------------*/
//frame = byte Add + byte Function + byte Length + n x byte Data +  2 x byte CRC
//String get_data_from_frame(byte[] data_frame) {
//  Modbus_Frame data_rev;
//  data_rev.DevID = data_frame[0];
//  data_rev.Fcode = data_frame[1];
//  data_rev.Length = data_frame[2];
//  for (int i = 0; i < data_rev.Length; i++) {
//    data_rev[i] = data_frame[i + 3];
//  }
//  data_rev.CRC = data_frame[];
//}

int Scan_I2C_Addr() {
  byte error, address;
  int nDevices[2];
  #ifdef _DEBUG_
    Serial.println("Scanning...");
  #endif
  nDevices = 0;
  for (address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      return nDevices;
    }
    else if (error == 4) {
      Serial.print("Unknow error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      return nDevices;
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  delay(50);
}

void display_data_receive() {

}

void display_time() {

}

void display_wifi_infor() {

}
/*--------------------------------UTILITIES------------------------------*/
unsigned int hexToDec(String hexString) {

  unsigned int decValue = 0;
  int nextInt;

  for (int i = 0; i < hexString.length(); i++) {

    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);

    decValue = (decValue * 16) + nextInt;
  }

  return decValue;
}

//void get_data_ep520_02(float &moisture, float &temperature, float &ec)
//{
//  byte array_rs485[12] = {0x00};
//
//  rs485_ttl.listen();
//  // Clear data
//  delay(10);
//  while (rs485_ttl.available())
//  {
//    rs485_ttl.read();
//  }
//
//  rs485_ttl.write(code_ep520_02, 8);
//  delay(500);
//  int index_array_rs485 = 0;
//  while (rs485_ttl.available())
//  {
//    byte read_rs485 = rs485_ttl.read();
//    array_rs485[index_array_rs485] = read_rs485;
//    index_array_rs485++;
//  }
//
//  // Serial.print("HEX array_rs485:");
//  // for (int i = 0; i < index_array_rs485; i++)
//  // {
//  //   if (array_rs485[i] < 0x10) {
//  //     Serial.print("0");
//  //   }
//  //   Serial.print(array_rs485[i], HEX); Serial.print(" ");
//  // }
//  // Serial.println("");
//
//  String str_hex_mois1 = String(array_rs485[3], HEX);
//  String str_hex_mois2 = String(array_rs485[4], HEX);
//  String str_hex_mois = str_hex_mois1 + str_hex_mois2;
//  moisture = hexToDec(str_hex_mois) / 100.00;
//
//  String str_hex_temp1 = String(array_rs485[5], HEX);
//  String str_hex_temp2 = String(array_rs485[6], HEX);
//  String str_hex_temp = str_hex_temp1 + str_hex_temp2;
//  if (hexToDec(str_hex_temp) >= 32768)
//  {
//    temperature = (hexToDec(str_hex_temp) - 32768 - 1) / 100.00;
//  }
//  else
//  {
//    temperature = hexToDec(str_hex_temp) / 100.00;
//  }
//
//  String str_hex_ec1 = String(array_rs485[7], HEX);
//  String str_hex_ec2 = String(array_rs485[8], HEX);
//  String str_hex_ec = str_hex_ec1 + str_hex_ec2;
//  ec = hexToDec(str_hex_ec) / 1000.00;
//
//}




void tinh_sai_so(float &last_data, float input_data, float saiso)
{
  if (( input_data  < (last_data - saiso)) || (input_data > (last_data + saiso)))
  {
    last_data = input_data;
  }
}
/*------------------------------------------------------------------------*/
//void user_task_send_data()
//{
//  arduino_timeout = 0;
//
//  float float_avgValue;  //Store the average value of the sensor feedback
//  float buf_sensor_data1[10], buf_sensor_data2[10], buf_sensor_data3[10], float_tmp_swap;
//
//  float moisture, temp_c_soil, ec;
//  // float sum_moisture, sum_temp_c_soil, sum_ec;
//  // active sensor ep520_02
//  get_data_ep520_02(moisture, temp_c_soil, ec);
//  while (temp_c_soil < 2.0)
//  {
//    delay(10);
//    get_data_ep520_02(moisture, temp_c_soil, ec);
//  }
//  tinh_sai_so(last_moisture, moisture, 1);
//  tinh_sai_so(last_temp_c_soil, temp_c_soil, 0.5);
//  tinh_sai_so(last_ec, ec, 0);
//
//  // Read values from the sensor
//  delay(10);
//
//  // String s_data = "\"humi1\":\"" + (String)last_humidity_air + "\",\"temp1\":\"" + (String)last_temp_c_air + "\",\"mois1\":\"" + (String)last_moisture + "\",\"stemp1\":\"" + (String)last_temp_c_soil + "\",\"ec1\":\"" + (String)last_ec + "\",\"ph1\":\"" + (String)last_phvalue + "\",\"light1\":\"" + (String)last_light + "\"";
//  String s_data = "\"mois1\":\"" + (String)last_moisture + "\",\"stemp1\":\"" + (String)last_temp_c_soil + "\",\"ec1\":\"" + (String)last_ec + "\"";
//  // Serial.println(s_data);
//  send_data_to_cloud(s_data);
//}
//
////    "humi1":"54.49","temp1":"55.55","mois1":"345"
//void send_data_to_cloud_via_SIM_Port(String sensor_data)
//{
//  arduino_timeout = 0;
//  SIM_Port.listen();
//  SIM_Port.println(cmd_connect);
//
//  while (!SIM_Port.find("CONNECT OK\r\n"))
//  {
//    // Serial.println(F("send_data_to_cloud SIM_Port Don't Connect !!!"));
//    SIM_Port.println(cmd_connect);
//  }
//
//  String s_data = MAC + "[]Insertsensor-{\"sensor\":{" + sensor_data + "}}";
//  Serial.println(s_data);
//  int length_s_data = s_data.length();
//  String cmd_CIPSEND = "AT+CIPSEND=" + String(length_s_data);
//  SIM_Port.println(cmd_CIPSEND);
//  SIM_Port.find("> ");
//  // Serial.println(F("Ready Send data"));
//
//  //  Serial.println(s_data);
//  SIM_Port.println(s_data);
//  SIM_Port.find("SEND OK\r\n");
//  // Serial.println(F("s_data SEND OK"));
//
//  SIM_Port.println("AT+CIPSHUT");
//  SIM_Port.find("SHUT OK\r\n");
//  // Serial.println(F("SHUT OK"));
//}
//
////void reset_MCU()
////{
////  pinMode(PIN_RESET_MCU, OUTPUT);
////  digitalWrite(PIN_RESET_MCU, LOW);
////  delay(500);
////  asm volatile ( "jmp 0");
////}
