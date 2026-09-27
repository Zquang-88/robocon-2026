#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "RobotConfig.h"
using namespace RobotConfig;

// READ-ONLY diagnostic for both 3-eye side arrays and H1/H2.
// This firmware never configures or drives any motor output.
namespace {
constexpr uint32_t SERIAL_BAUD=115200;
uint16_t rawValue[8]={},normalizedValue[8]={};
float filteredValue[8]={};
uint32_t printPeriodMs=100,lastPrintMs=0;
bool streamEnabled=true;
// Print logical sensor storage in physical MCP3008 CH0..CH7 order.
constexpr uint8_t PHYSICAL_CHANNEL_ORDER[8]={3,4,5,0,1,2,6,7};

uint16_t readMcp(uint8_t channel){
  digitalWrite(MCP_STOP_CS,LOW);
  SPI.transfer(0x01);
  const uint8_t high=SPI.transfer((0x08|(channel&0x07))<<4);
  const uint8_t low=SPI.transfer(0);
  digitalWrite(MCP_STOP_CS,HIGH);
  return static_cast<uint16_t>(((high&0x03)<<8)|low);
}

uint16_t normalizeValue(uint16_t raw,uint16_t minimum,uint16_t maximum){
  if(maximum<=minimum+5)return 0;
  long value=constrain(raw,minimum,maximum);
  value=(value-minimum)*1000L/(maximum-minimum);
  if(!LINE_IS_HIGHER_THAN_FLOOR)value=1000L-value;
  return static_cast<uint16_t>(constrain(value,0L,1000L));
}

void readSensors(){
  SPI.beginTransaction(SPISettings(MCP_SPI_HZ,MSBFIRST,SPI_MODE0));
  for(uint8_t i=0;i<3;i++){
    rawValue[i]=readMcp(STOP_LEFT_CHANNELS[i]);
    rawValue[3+i]=readMcp(STOP_RIGHT_CHANNELS[i]);
  }
  rawValue[6]=readMcp(LATERAL_HOLD_TAIL_CHANNEL);
  rawValue[7]=readMcp(LATERAL_HOLD_FRONT_CHANNEL);
  SPI.endTransaction();

  for(uint8_t i=0;i<8;i++){
    filteredValue[i]+=LINE_FILTER_ALPHA*(rawValue[i]-filteredValue[i]);
    const uint16_t minimum=i<6?STOP_SENSOR_MIN[i]:LATERAL_HOLD_SENSOR_MIN[i-6];
    const uint16_t maximum=i<6?STOP_SENSOR_MAX[i]:LATERAL_HOLD_SENSOR_MAX[i-6];
    normalizedValue[i]=normalizeValue(
        static_cast<uint16_t>(lroundf(filteredValue[i])),minimum,maximum);
  }
}

bool onBlack(uint8_t index){
  const uint16_t threshold=index<6?LINE_ACTIVE_NORMALIZED:
                                     LATERAL_HOLD_LINE_THRESHOLD;
  return normalizedValue[index]>=threshold;
}

void printHeader(){
  Serial.println("SIDE_HOLD_LINE_DEBUG,READ_ONLY,MOTORS_DISABLED");
  Serial.println("LABEL,MS,CH0,CH1,CH2,CH3,CH4,CH5,CH6,CH7");
  Serial.println("MAP,-,R_OUT,R_MID,R_IN,L_OUT,L_MID,L_IN,H1_TAIL,H2_FRONT");
  Serial.print("THRESHOLD,SIDE,");Serial.print(LINE_ACTIVE_NORMALIZED);
  Serial.print(",H1_H2,");Serial.println(LATERAL_HOLD_LINE_THRESHOLD);
  Serial.println("COMMANDS,SNAP,STREAM ON,STREAM OFF,RATE 50..2000,HELP");

}

void printRow(const char* label,const uint16_t* values){
  Serial.print(label);Serial.print(',');Serial.print(millis());
  for(uint8_t channel=0;channel<8;channel++){
    Serial.print(',');
    Serial.print(values[PHYSICAL_CHANNEL_ORDER[channel]]);
  }
  Serial.println();
}

void printSnapshot(){
  uint16_t black[8]={};
  for(uint8_t i=0;i<8;i++)black[i]=onBlack(i)?1:0;
  printRow("RAW",rawValue);
  printRow("NORM",normalizedValue);
  printRow("BLACK",black);
  Serial.print("STATUS,LEFT_OUTER_MIDDLE=");
  Serial.print(onBlack(0)&&onBlack(1)?"ON":"OFF");
  Serial.print(",RIGHT_OUTER_MIDDLE=");
  Serial.print(onBlack(3)&&onBlack(4)?"ON":"OFF");
  Serial.print(",H1_H2=");
  Serial.println(onBlack(6)&&onBlack(7)?"ON":"OFF");
  Serial.println();
}

void processCommand(char* line){
  while(*line==' '||*line=='\t')line++;
  if(!strcmp(line,"SNAP")){readSensors();printSnapshot();}
  else if(!strcmp(line,"STREAM ON")){streamEnabled=true;Serial.println("ACK,STREAM,ON");}
  else if(!strcmp(line,"STREAM OFF")){streamEnabled=false;Serial.println("ACK,STREAM,OFF");}
  else if(!strncmp(line,"RATE ",5)){
    const long rate=strtol(line+5,nullptr,10);
    if(rate<50||rate>2000)Serial.println("ERR,RATE_RANGE,50..2000_MS");
    else{printPeriodMs=static_cast<uint32_t>(rate);Serial.print("ACK,RATE,");Serial.println(rate);}
  }else if(!strcmp(line,"HELP")||!strcmp(line,"?"))printHeader();
  else if(*line)Serial.println("ERR,UNKNOWN_COMMAND");
}

void pollSerial(){
  static char buffer[48]={};static uint8_t length=0;
  while(Serial.available()){
    const char c=static_cast<char>(Serial.read());
    if(c=='\r'||c=='\n'){
      if(length){buffer[length]='\0';processCommand(buffer);length=0;}
    }else if(length<sizeof(buffer)-1)buffer[length++]=c;
    else length=0;
  }
}
}

void setup(){
  Serial.begin(SERIAL_BAUD);
  pinMode(MCP_STOP_CS,OUTPUT);digitalWrite(MCP_STOP_CS,HIGH);SPI.begin();
  delay(250);
  for(uint8_t i=0;i<12;i++){readSensors();delay(5);}
  printHeader();
}

void loop(){
  pollSerial();readSensors();
  if(streamEnabled&&millis()-lastPrintMs>=printPeriodMs){
    lastPrintMs=millis();printSnapshot();
  }
}

//CH0: H đuôi
//CH1: H mũi
//CH2: l trong
//CH3: l giữa
//CH4: l ngoài
//CH5: r ngoài
//CH6: r giữa
//CH7: r trong