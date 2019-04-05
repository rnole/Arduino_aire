#include <Wire.h>
#include <SHT1x.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_CCS811.h"
#include "Adafruit_SI1145.h"
#include "Definitions.h"

Adafruit_CCS811 Ccs811;
SHT1x sht1x(DATAPIN, CLOCKPIN);
SoftwareSerial CO2_serial(CO2_RX, CO2_TX);
SoftwareSerial O3_serial(O3_RX, O3_TX);
SoftwareSerial NO2_serial(NO2_RX, NO2_TX);
SoftwareSerial SO2_serial(SO2_RX, SO2_TX);
SoftwareSerial PM_serial(PM_RX, PM_TX);
PMS7003_struct PM_answer;

uint8_t  _12_secs_counter   = 0;
uint8_t spec_array_index    = 0;

uint32_t CCS_value      = 0;
uint32_t NO_value       = 0;
uint32_t O3_values[6]   = {0};
uint32_t NO2_values[6]  = {0};
uint32_t SO2_values[6]  = {0};
uint32_t O3_avg_value   = 0;
uint32_t NO2_avg_value  = 0;
uint32_t SO2_avg_value  = 0;
uint32_t CO2_value      = 0;
uint16_t pm1_value      = 0;
uint16_t pm25_value     = 0;
uint16_t pm10_value     = 0;


float uv_value = 0;
float Temperature_value = 0;
float Humidity_value = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  Ccs811_init();
  O3_serial.begin(9600);
  O3_serial.print('r');
  delay(8000);
  O3_serial.end();
  SO2_serial.begin(9600);
  SO2_serial.print('r');
  delay(8000);
  SO2_serial.end();
}

void loop() {

  Serial.println("Leyendo O3...");
  O3_Test(spec_array_index);
  Serial.println("Leyendo NO2...");
  NO2_Test(spec_array_index);
  Serial.println("Leyendo SO2...");
  SO2_Test(spec_array_index);
  Serial.println("Leyendo PM...");
  PM_Test();
  Serial.println("Leyendo CO2...");
  CO2_Test();
  Serial.println("Leyendo Temp...");
  Tem_hum_Test();
  Serial.println("Leyendo CCS...");
  CCS_Test();
  Serial.println("Leyendo NO...");
  NO_Test();
  Serial.println("Todos los sensores leidos.Entro a delay...");

  if(_12_secs_counter < 2){
      _12_secs_counter++;
      spec_array_index++;
      delay(WAIT_TWELVE_SECONDS);
      
  }   
  else{
      Spec_sensors_get_average_values();
      Send_to_server();
      Clean_global_variables();
      delay(WAIT_TWELVE_SECONDS);
      
  }    
}

void Ccs811_init(void){

  if(!Ccs811.begin()){
    //Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }

  while(!Ccs811.available());
  float temp = Ccs811.calculateTemperature();
  Ccs811.setTempOffset(temp - 25.0);
  
}

bool Z_was_received(void){

  bool response = false;

  if(Serial.available() > 0){
      char rpi_command = Serial.read();
      if(rpi_command == 'Z')
          response = true;
  }

  return response;
}

void Recalibrate_spec_sensors(void){
  delay(10000);
  
  O3_serial.begin(9600);
  O3_serial.print('Z');
  O3_serial.end();

  NO2_serial.begin(9600);
  NO2_serial.print('Z');
  NO2_serial.end();

  SO2_serial.begin(9600);
  SO2_serial.print('Z');
  SO2_serial.end();
  
  delay(10000);
}

void NO_Test(void){
  
  NO_value = analogRead(NO_ANALOG);

  
}

void CCS_Test(void){

  if(Ccs811.available()){
    if(!Ccs811.readData()){
      CCS_value = Ccs811.getTVOC();
    }
    else{
      while(1);
    }
  }
  
}


void CO2_Test(void){

  byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25}; 
  byte response[] = {0,0,0,0,0,0,0};

  CO2_send_request(readCO2, response);
  CO2_value = CO2_get_value(response);
  
}

void CO2_send_request(byte packet_cmd[], byte response[]){

  uint32_t start_time = millis();
  CO2_serial.begin(9600);
  
  while(!CO2_serial.available() ){

    if( (millis() - start_time) > TIMEOUT_VALUE){
        CO2_serial.end();
        return;
    }
    
    CO2_serial.write(packet_cmd,7);
    delay(50);
  }
  
  int timeout=0;
  while(CO2_serial.available() < 7){
    timeout++;  
    if(timeout > 10){
        while(CO2_serial.available()){
            CO2_serial.read();
        }
        break;
    }
    delay(50);
  }
  
  for (int i=0; i < 7; i++)
    response[i] = CO2_serial.read();

  CO2_serial.end();
  
}

uint32_t CO2_get_value(byte packet[]){

  int high = packet[3];
  int low = packet[4];
  uint32_t val = high*256 + low;                
  return val*1;
  
}


void O3_Test(uint8_t array_idx){
  int serial_rx_flag = 0;
  int buf_idx = 0;
  char O3_response[80] = {0};
  uint32_t start_time = millis();
  
  //ln("Starting O3...");
  O3_serial.begin(9600);
  O3_serial.print('c');
  while((serial_rx_flag == 0) && ((millis() - start_time) < TIMEOUT_VALUE)){
      
       while (O3_serial.available() > 0){
         O3_response[buf_idx++]= O3_serial.read();
         serial_rx_flag = 1;
         delay(10);
       }
  }

  O3_response[buf_idx] = '\0';
  //ln(O3_response);
  O3_serial.print('r');
  O3_serial.end();
  O3_save_value(O3_response, array_idx);
}

void O3_save_value(char *buf, uint8_t array_idx){

  char *p_cmd;

  p_cmd = strstr(buf, ", ") + 2;

  p_cmd = strstr(p_cmd, ", ") + 2;
  p_cmd = strstr(p_cmd, ", ") + 2;
  p_cmd = strstr(p_cmd, ", ") + 2;
  
  O3_values[array_idx] = atoi(p_cmd);
}

void NO2_Test(uint8_t array_idx){
  int serial_rx_flag = 0; 
  int buf_idx = 0;
  char NO2_response[80] = {0};
  uint32_t start_time = millis();
  
  NO2_serial.begin(9600);
  NO2_serial.print('c');

  while((serial_rx_flag == 0) && ((millis() - start_time) < TIMEOUT_VALUE)){
      
       while(NO2_serial.available() > 0){
         NO2_response[buf_idx++]= NO2_serial.read();
         serial_rx_flag = 1;
         delay(10);
       }
  }
  NO2_response[buf_idx] = '\0';
  NO2_serial.print('r');
  NO2_serial.end();
  NO2_save_value(NO2_response,array_idx);
}

void NO2_save_value(char *buf, uint8_t array_idx){

  char *p_cmd;

  p_cmd = strstr(buf, ", ") + 2;

  p_cmd = strstr(p_cmd, ", ") + 2;
  p_cmd = strstr(p_cmd, ", ") + 2;
  p_cmd = strstr(p_cmd, ", ") + 2;
  
  NO2_values[array_idx] = atoi(p_cmd);
}

void SO2_Test(uint8_t array_idx){
  
  int serial_rx_flag = 0; 
  int buf_idx = 0;
  char SO2_response[80] = {0};
  uint32_t start_time = millis();
  
  SO2_serial.begin(9600);
  SO2_serial.print('c');

  while((serial_rx_flag == 0) && ((millis() - start_time) < TIMEOUT_VALUE)){
      
       while (SO2_serial.available() > 0){
         SO2_response[buf_idx++]= SO2_serial.read();
         serial_rx_flag = 1;
         delay(10);
       }
  }
  
  SO2_response[buf_idx] = '\0';
  SO2_serial.print('r');
  SO2_serial.end();
  SO2_save_value(SO2_response, array_idx);
}

void SO2_save_value(char *buf, uint8_t array_idx){

  char *p_cmd;

  p_cmd = strstr(buf, ", ") + 2;

  p_cmd = strstr(p_cmd, ", ") + 2;
  p_cmd = strstr(p_cmd, ", ") + 2;
  p_cmd = strstr(p_cmd, ", ") + 2;
  
  SO2_values[array_idx] = atol(p_cmd);
}



void PM_Test(void){
  
 int serial_rx_flag = 0;
 int buf_idx = 0;
 uint8_t PM_response[32] = {0};
 uint32_t start_time = millis();
 
 uint8_t  set_passive_mode_cmd[] = {
   0x42,    // Start Byte 1
   0x4D,    // Start Byte 2
   0xE1,    // Command
   0x00,    // Passive mode
   0x01,    // Verify Byte 1
   0x70     // Verify Byte 2
  };

  uint8_t read_data_cmd[] = {
   0x42,    // Start Byte 1
   0x4D,    // Start Byte 2
   0xE2,    // Command          
   0x01,    // Verify Byte 1 
   0x71     // Verify Byte 2
  };

  PM_serial.begin(9600);
  
  for(int i = 0; i < 6; i++)
      PM_serial.print(set_passive_mode_cmd[i]);

  PM_serial.flush(); 
  for(int i = 0; i < 5; i++)
      PM_serial.print(read_data_cmd[i]);

  while((serial_rx_flag == 0) && ((millis() - start_time) < TIMEOUT_VALUE)){     
       while (PM_serial.available() > 0){
         PM_response[buf_idx++]= PM_serial.read();
         serial_rx_flag = 1;
         delay(10);
       }
  }

  if((PM_response[0] == 0x42) && (PM_response[1] == 0x4D)){
      PM_answer.frame_length[0] = PM_response[2];
      PM_answer.frame_length[1] = PM_response[3];
      pm1_value  = PM_response[4]*256 + PM_response[5];
      pm25_value = PM_response[6]*256 + PM_response[7];
      pm10_value = PM_response[8]*256 + PM_response[9];
  }
  
  PM_serial.end();
}



void Tem_hum_Test(void){
  Temperature_value = sht1x.readTemperatureC();
  Humidity_value = sht1x.readHumidity();
}


void Spec_sensors_get_average_values(void){
  
  O3_avg_value = O3_values[0];
  NO2_avg_value= NO2_values[0];
  SO2_avg_value= SO2_values[0];
  
  //Check_if_Z_is_neccesary(O3_avg_value,  O3_serial);
  //Check_if_Z_is_neccesary(NO2_avg_value, NO2_serial);
  //Check_if_Z_is_neccesary(SO2_avg_value, SO2_serial);

  
  if(O3_avg_value > 999999)
      O3_avg_value = 0;
  if(NO2_avg_value > 999999)
      NO2_avg_value = 0;
  if(SO2_avg_value > 999999)
      SO2_avg_value = 0;
  
}

uint32_t Spec_sensor_get_average_from_array(uint32_t spec_sensor_values[]){

  uint8_t valid_values_counter = 0;
  uint32_t average_value = 0;

  for(int idx = 0; idx < 6; idx++){

    if((spec_sensor_values[idx] < 999999) && (spec_sensor_values[idx]!= 0)){
          average_value += spec_sensor_values[idx];
          valid_values_counter++;
    }
  }

  if(valid_values_counter > 0)
      average_value = average_value / valid_values_counter;
    
  return average_value;
}

void Check_if_Z_is_neccesary(uint32_t avg_value, SoftwareSerial &Spec_sensor_serial){

  if(avg_value > 999999){
    Spec_sensor_serial.begin(9600);
    Spec_sensor_serial.print('Z');
    Spec_sensor_serial.end();
  }
      
}

void Clean_global_variables(void){

  _12_secs_counter = 0;
  spec_array_index = 0;
  memset(O3_values, 0, sizeof(O3_values));
  memset(NO2_values, 0, sizeof(NO2_values));
  memset(SO2_values, 0, sizeof(SO2_values));
}

void Send_to_server(void){

  Serial.print(NO_value);          Serial.print(',');
  Serial.print(CCS_value);         Serial.print(',');
  Serial.print(O3_avg_value);      Serial.print(',');
  Serial.print(NO2_avg_value);     Serial.print(',');
  Serial.print(SO2_avg_value);     Serial.print(',');
  Serial.print(NO_value);          Serial.print(',');
  Serial.print(CO2_value);         Serial.print(',');
  Serial.print(pm1_value);         Serial.print(',');
  Serial.print(pm25_value);        Serial.print(',');
  Serial.print(pm10_value);        Serial.print(',');  
  Serial.print(Temperature_value); Serial.print(',');
  Serial.print(Humidity_value);
  Serial.print('\n');
  
}

