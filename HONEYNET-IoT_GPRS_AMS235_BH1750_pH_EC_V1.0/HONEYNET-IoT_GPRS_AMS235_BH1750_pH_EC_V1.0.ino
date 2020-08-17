//Arduino Pro Mini 3.3V 8MHz
#include <ArduinoJson.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// Connect RED of the AM2315 sensor to 5.0V
// Connect BLACK to Ground
// Connect WHITE to i2c clock
// Connect YELLOW to i2c data
#include <Wire.h>
#include "Adafruit_AM2315.h"
Adafruit_AM2315 am2315;


#include "BH1750FVI.h"
BH1750FVI LightSensor;


#include <SoftwareSerial.h>
SoftwareSerial sim800a(3, 5); //RX:3, TX:5
SoftwareSerial rs485_ttl(9, 6); //RX:9, TX:6

#define power_sim800a 4
#define buttonPin A7
#define ledPin 8
#define pin_reset_arduino 7
#define AnalogPin 6


String cmd_connect = "AT+CIPSTART=\"TCP\",\"ais.honeynet.vn\",5100";
String MAC;

float last_humidity_air, last_temp_c_air, last_moisture, last_temp_c_soil, last_phvalue, last_ec, last_light;

byte code_ep520_02[8] = {  0x51, 0x03, 0x00, 0x00, 0x00, 0x03, 0x09, 0x9B};

unsigned long INTERVAL = 5;
unsigned long DELAY_RUNNING = 0;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

volatile boolean f_wdt = 1;

unsigned long count_timeout = 0;
unsigned long arduino_timeout = 0;
unsigned long getinfo_timeout = 0;
volatile boolean f_getinfo_timeout = 0;


void reset_arduino()
{
  pinMode(pin_reset_arduino, OUTPUT);
  digitalWrite(pin_reset_arduino, LOW);
  delay(500);
  asm volatile ( "jmp 0");
}

void get_mac_address()
{
  sim800a.listen();
  sim800a.setTimeout(3000);
  sim800a.println("AT");
  while (!sim800a.find("OK\r\n"))
  {
    // Serial.println(F("get_mac_address SIM800A Power OFF !!!"));
    sim800a.println("AT");
  }

  sim800a.println("AT+GSN");
  while (!sim800a.find("OK\r\n"))
  {
    // Serial.println(F("get_mac_address AT+GSN !!!"));
    sim800a.println("AT+GSN");
  }

  sim800a.println("AT+GSN");
  String read_data = sim800a.readStringUntil("OK");

  if (read_data.length() == 32)
  {
    String IMEI = read_data.substring(12, 24);
    String _mac;
    for (int i = 0; i < IMEI.length(); i++)
    {
      _mac = _mac + IMEI[i];
      if (((i + 1) % 2) == 0)
      {
        _mac = _mac + ':';
      }
    }
    _mac.remove(_mac.length() - 1);
    MAC = _mac;
    if (MAC.length() != 17)
    {
      reset_arduino();
    }
    return;
  }
  reset_arduino();
}

void running_task(unsigned long timeout)
{
  long _timeout_get_value = timeout - DELAY_RUNNING;

  if (_timeout_get_value <= 0)
  {
    _timeout_get_value = timeout;
  }

  if (count_timeout >= (_timeout_get_value * 2) || f_getinfo_timeout == 1)
  {
    arduino_timeout = 0;

    DELAY_RUNNING = millis();

    sim800a.listen();
    sim800a.setTimeout(3000);
    sim800a.println("AT");
    while (!sim800a.find("OK\r\n"))
    {
      // Serial.println(F("running_task SIM800A Power OFF !!!"));
      sim800a.println("AT");
    }

    sim800a.println("AT+CIPSHUT");
    while (!sim800a.find("SHUT OK\r\n"))
    {
      // Serial.println(F("SIM800A Don't AT+CIPSHUT!!!"));
      sim800a.println("AT+CIPSHUT");
    }

    pinMode(ledPin, OUTPUT);
    for (byte i = 0; i < 10; i++)
    {
      digitalWrite (ledPin, HIGH);
      delay (50);
      digitalWrite (ledPin, LOW);
      delay (50);
    }
    digitalWrite (ledPin, HIGH);

    if (count_timeout >= (_timeout_get_value * 2))
    {
      user_task_send_data();
      get_info_controll();
    }
    else
    {
      getinfo_timeout = 0;
      get_info_controll();
      f_getinfo_timeout = 0;
    }

    count_timeout = 0;
    arduino_timeout = 0;

    DELAY_RUNNING = (millis() - DELAY_RUNNING) / 1000;
  }
  count_timeout++;
}


void setup() {
  // Turnon sim800a
  // pinMode(pin_reset_arduino, INPUT_PULLUP);
  pinMode(pin_reset_arduino, OUTPUT);
  digitalWrite(pin_reset_arduino, HIGH);
  // RESET SIM800A
  pinMode(power_sim800a, OUTPUT);
  digitalWrite(power_sim800a, HIGH);
  delay(2000);
  digitalWrite(power_sim800a, LOW);

  pinMode(ledPin, OUTPUT);
  digitalWrite (ledPin, HIGH);
  delay(5000);

  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }
  sim800a.begin(9600);
  get_mac_address();
  if (! am2315.begin()) {
    // Serial.println(F("Sensor am2315 not found, check wiring & pullups!"));
    while (1);
  }

  rs485_ttl.begin(9600);

  // LightSensor
  LightSensor.begin();
  LightSensor.SetAddress(Device_Address_L);//Address 0x23
  LightSensor.SetMode(Continuous_H_resolution_Mode);

  // CPU Sleep Modes
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)

  cbi( SMCR, SE );     // sleep enable, power down mode
  cbi( SMCR, SM0 );    // power down mode
  sbi( SMCR, SM1 );    // power down mode
  cbi( SMCR, SM2 );    // power down mode

  setup_watchdog(5);
  arduino_timeout = 0;
  getinfo_timeout = 0;
}


//****************************************************************
//****************************************************************
//****************************************************************
void loop() {
  if (f_wdt == 1) { // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    f_wdt = 0;     // reset flag

    running_task(INTERVAL);
  }
}

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

void get_data_ep520_02(float &moisture, float &temperature, float &ec)
{
  byte array_rs485[12] = {0x00};

  rs485_ttl.listen();
  // Clear data
  delay(10);
  while (rs485_ttl.available())
  {
    rs485_ttl.read();
  }

  rs485_ttl.write(code_ep520_02, 8);
  delay(500);
  int index_array_rs485 = 0;
  while (rs485_ttl.available())
  {
    byte read_rs485 = rs485_ttl.read();
    array_rs485[index_array_rs485] = read_rs485;
    index_array_rs485++;
  }

  // Serial.print("HEX array_rs485:");
  // for (int i = 0; i < index_array_rs485; i++)
  // {
  //   if (array_rs485[i] < 0x10) {
  //     Serial.print("0");
  //   }
  //   Serial.print(array_rs485[i], HEX); Serial.print(" ");
  // }
  // Serial.println("");

  String str_hex_mois1 = String(array_rs485[3], HEX);
  String str_hex_mois2 = String(array_rs485[4], HEX);
  String str_hex_mois = str_hex_mois1 + str_hex_mois2;
  moisture = hexToDec(str_hex_mois) / 100.00;

  String str_hex_temp1 = String(array_rs485[5], HEX);
  String str_hex_temp2 = String(array_rs485[6], HEX);
  String str_hex_temp = str_hex_temp1 + str_hex_temp2;
  if (hexToDec(str_hex_temp) >= 32768)
  {
    temperature = (hexToDec(str_hex_temp) - 32768 - 1) / 100.00;
  }
  else
  {
    temperature = hexToDec(str_hex_temp) / 100.00;
  }

  String str_hex_ec1 = String(array_rs485[7], HEX);
  String str_hex_ec2 = String(array_rs485[8], HEX);
  String str_hex_ec = str_hex_ec1 + str_hex_ec2;
  ec = hexToDec(str_hex_ec) / 1000.00;

}


void tinh_sai_so(float &last_data, float input_data, float saiso)
{
  if (( input_data  < (last_data - saiso)) || (input_data > (last_data + saiso)))
  {
    last_data = input_data;
  }
}


void user_task_send_data()
{
  arduino_timeout = 0;
  float temp_c_air;
  float humidity_air;


  float float_avgValue;  //Store the average value of the sensor feedback
  float buf_sensor_data1[10], buf_sensor_data2[10], buf_sensor_data3[10], float_tmp_swap;
  // for (int i = 0; i < 10; i++)
  // {
  //   buf_sensor_data1[i] = 0.00;
  //   buf_sensor_data2[i] = 0.00;
  //   buf_sensor_data3[i] = 0.00;
  //   float_avgValue = 0.00;
  //   float_tmp_swap = 0.00;
  // }
  // for (int i = 0; i < 4; i++) //Get 10 sample value from the sensor for smooth the value
  // {
  //   while (1)
  //   {
  //     if (am2315.readTemperatureAndHumidity(&temp_c_air, &humidity_air))
  //     {
  //       buf_sensor_data1[i] = temp_c_air;
  //       buf_sensor_data2[i] = humidity_air;
  //       break;
  //     }
  //     delay(10);
  //   }
  //   // buf[i] = analogRead(AnalogPin);
  //   delay(10);
  // }
  // for (int i = 0; i < 3; i++) //sort the analog from small to large
  // {
  //   for (int j = i + 1; j < 4; j++)
  //   {
  //     if (buf_sensor_data1[i] > buf_sensor_data1[j])
  //     {
  //       float_tmp_swap = buf_sensor_data1[i];
  //       buf_sensor_data1[i] = buf_sensor_data1[j];
  //       buf_sensor_data1[j] = float_tmp_swap;
  //     }
  //     if (buf_sensor_data2[i] > buf_sensor_data2[j])
  //     {
  //       float_tmp_swap = buf_sensor_data2[i];
  //       buf_sensor_data2[i] = buf_sensor_data2[j];
  //       buf_sensor_data2[j] = float_tmp_swap;
  //     }
  //   }
  // }
  // float_avgValue = 0;
  // for (int i = 1; i < 3; i++)
  // {
  //   float_avgValue += buf_sensor_data1[i];
  // }
  // temp_c_air = float_avgValue / 2;
  // tinh_sai_so(last_temp_c_air, temp_c_air, 0.1);

  // float_avgValue = 0;
  // for (int i = 1; i < 3; i++)
  // {
  //   float_avgValue += buf_sensor_data2[i];
  // }
  // humidity_air = float_avgValue / 2;
  // tinh_sai_so(last_humidity_air, humidity_air, 2);

  while (1)
  {
    if (am2315.readTemperatureAndHumidity(&temp_c_air, &humidity_air))
    {
      break;
    }
    delay(100);
  }
  tinh_sai_so(last_temp_c_air, temp_c_air, 0.1);
  tinh_sai_so(last_humidity_air, humidity_air, 2);

  float moisture, temp_c_soil, ec;
  // float sum_moisture, sum_temp_c_soil, sum_ec;
  // active sensor ep520_02
  get_data_ep520_02(moisture, temp_c_soil, ec);
  while (temp_c_soil < 2.0)
  {
    delay(10);
    get_data_ep520_02(moisture, temp_c_soil, ec);
  }
  tinh_sai_so(last_moisture, moisture, 1);
  tinh_sai_so(last_temp_c_soil, temp_c_soil, 0.5);
  tinh_sai_so(last_ec, ec, 0);

  // Read values from the sensor
  delay(10);

  uint16_t lux = LightSensor.GetLightIntensity();
  tinh_sai_so(last_light, lux, 5);

  for (int i = 0; i < 10; i++)
  {
    buf_sensor_data1[i] = 0.00;
    buf_sensor_data2[i] = 0.00;
    buf_sensor_data3[i] = 0.00;
    float_avgValue = 0.00;
    float_tmp_swap = 0.00;
  }
  for (int i = 0; i < 10; i++) //Get 10 sample value from the sensor for smooth the value
  {
    delay(10);
    buf_sensor_data1[i] = analogRead(AnalogPin);
  }
  for (int i = 0; i < 9; i++) //sort the analog from small to large
  {
    for (int j = i + 1; j < 10; j++)
    {
      if (buf_sensor_data1[i] > buf_sensor_data1[j])
      {
        float_tmp_swap = buf_sensor_data1[i];
        buf_sensor_data1[i] = buf_sensor_data1[j];
        buf_sensor_data1[j] = float_tmp_swap;
      }
    }
  }
  float_avgValue = 0;
  for (int i = 2; i < 8; i++)               //take the average value of 6 center sample
  {
    float_avgValue += buf_sensor_data1[i];
  }
  float phValue = (float)float_avgValue * 5.0 / 1024 / 6; //convert the analog into millivolt
  phValue = 3.5 * phValue;                  //convert the millivolt into pH value
  if (phValue > 14)
  {
    phValue = 14.00;
  }
  tinh_sai_so(last_phvalue, phValue, 0.1);


  String s_data = "\"humi1\":\"" + (String)last_humidity_air + "\",\"temp1\":\"" + (String)last_temp_c_air + "\",\"mois1\":\"" + (String)last_moisture + "\",\"stemp1\":\"" + (String)last_temp_c_soil + "\",\"ec1\":\"" + (String)last_ec + "\",\"ph1\":\"" + (String)last_phvalue + "\",\"light1\":\"" + (String)last_light + "\"";
  // Serial.println(s_data);
  send_data_to_cloud(s_data);
}

//    "humi1":"54.49","temp1":"55.55","mois1":"345"
void send_data_to_cloud(String sensor_data)
{
  arduino_timeout = 0;
  sim800a.listen();
  sim800a.println(cmd_connect);

  while (!sim800a.find("CONNECT OK\r\n"))
  {
    // Serial.println(F("send_data_to_cloud SIM800A Don't Connect !!!"));
    sim800a.println(cmd_connect);
  }

  String s_data = MAC + "[]Insertsensor-{\"sensor\":{" + sensor_data + "}}";
  Serial.println(s_data);
  int length_s_data = s_data.length();
  String cmd_CIPSEND = "AT+CIPSEND=" + String(length_s_data);
  sim800a.println(cmd_CIPSEND);
  sim800a.find("> ");
  // Serial.println(F("Ready Send data"));

  //  Serial.println(s_data);
  sim800a.println(s_data);
  sim800a.find("SEND OK\r\n");
  // Serial.println(F("s_data SEND OK"));

  sim800a.println("AT+CIPSHUT");
  sim800a.find("SHUT OK\r\n");
  // Serial.println(F("SHUT OK"));
}


void apply_controll(String data)
{
  arduino_timeout = 0;
  StaticJsonDocument<200> jsonBuffer;
  DeserializationError error_data_json = deserializeJson(jsonBuffer, data);

  if (error_data_json) {
    //    Serial.println(F("parseObject() failed"));
    INTERVAL = 5;
    return;
  }

  unsigned long timeout = jsonBuffer["interval"];
  if (timeout >= 5)
  {
    INTERVAL = timeout;
  }
  //  Serial.print(F("INTERVAL:"));
  //  Serial.println(INTERVAL);
}

void get_info_controll()
{
  arduino_timeout = 0;
  sim800a.listen();
  sim800a.println(cmd_connect);

  while (!sim800a.find("CONNECT OK\r\n"))
  {
    // Serial.println(F("get_info_controll SIM800A Don't Connect !!!"));
    sim800a.println(cmd_connect);
  }

  String s_data = MAC + "[]CtrlGPIO";
  int length_s_data = s_data.length();
  String cmd_CIPSEND = "AT+CIPSEND=" + String(length_s_data);
  sim800a.println(cmd_CIPSEND);
  sim800a.find("> ");
  // Serial.println(F("Ready Send data"));

  // Serial.println(s_data);
  sim800a.println(s_data);
  String read_data = sim800a.readStringUntil('}');
  String split_read_data = read_data.substring(read_data.indexOf('{'), read_data.lastIndexOf(']') + 1);
  split_read_data += '}';

  sim800a.println("AT+CIPSHUT");
  sim800a.find("SHUT OK\r\n");
  // Serial.println(F("SHUT OK"));
  apply_controll(split_read_data);
}


//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  ww = bb;


  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}

//****************************************************************
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) {
  f_wdt = 1; // set global flag

  arduino_timeout++;
  if (arduino_timeout >= ((INTERVAL + 120) * 2))
  {
    reset_arduino();
  }

  if (f_getinfo_timeout == 0 && ((INTERVAL + 120) > 14400))
  {
    getinfo_timeout++;
  }
  //  Serial.print(F("getinfo_timeout:"));
  //  Serial.println(getinfo_timeout);
  if (getinfo_timeout >= 28800)
  {
    getinfo_timeout = 0;
    f_getinfo_timeout = 1;
  }
}
