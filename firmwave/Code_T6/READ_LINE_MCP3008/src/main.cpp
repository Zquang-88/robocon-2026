#include <SPI.h>

#define CS1 10

SPISettings MCP3008_SPI(1000000, MSBFIRST, SPI_MODE0);

uint16_t readMCP3008(uint8_t channel)
{
  if (channel > 7) return 0;

  SPI.beginTransaction(MCP3008_SPI);

  digitalWrite(CS1, LOW);

  byte command1 = 0x01;
  byte command2 = (0x08 | channel) << 4;

  byte highByte = SPI.transfer(command1);
  byte lowByte1 = SPI.transfer(command2);
  byte lowByte2 = SPI.transfer(0x00);

  digitalWrite(CS1, HIGH);

  SPI.endTransaction();

  uint16_t value = ((lowByte1 & 0x03) << 8) | lowByte2;

  return value;
}

void setup()
{
  Serial.begin(115200);

  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, HIGH);

  SPI.begin();
}

void loop()
{
  for (int i = 0; i < 8; i++)
  {
    Serial.print(readMCP3008(i));
    Serial.print("\t");
  }

  Serial.println();

  delay(10);
}