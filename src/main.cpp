
//*******************Example 1 ********************
#include <Arduino.h>
#include <Wire.h>

#define TMP112Addr 0x48

/* Address Pin Options:
 *  GND 1001000 0x48
 *  V+  1001001 0x49
 *  SDA 1001010 0x4A
 *  SCL 1001011 0x4B
 */
void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void loop()
{
  Wire.beginTransmission(TMP112Addr);
  Wire.write(0x0); /* Write Pointer=0 to select Temperature Register */
  Wire.endTransmission();
  Wire.requestFrom(TMP112Addr, 2); /* Request 2 bytes */
  int16_t ret = (Wire.read() << 8) | Wire.read();
  float tempC = (ret >> 4) * 0.0625;
  delay(2000);
    Serial.println(tempC);

 if (tempC <=27)
 {
   printf("Temprature is normal level\n");

 }
  else  if (tempC >=28)
 {
   printf("Temprature is above normal\n");

 }
}

//*********************Example 2************************

/*#include <Arduino.h>
#include <Wire.h>

#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22
#define I2C_SPEED 100000

#define TMP112Addr 0x48
#define TMP112_ADDRESS 0x48

void I2C_write()
{
  uint8_t buf[2] = {0};
  Wire.beginTransmission(TMP112Addr);
  Wire.write(0x00);
  Wire.readBytes(buf, 2);//2 bytes
  Wire.endTransmission();
  float temp = (buf[0] << 4 | buf[1] >> 4) * 0.0625;
  printf("Temp : %f\n", temp);
}

float readTemperature()
{
  Wire.beginTransmission(TMP112_ADDRESS); // Start communication with TMP112
  Wire.write(0x00);                       // Select temperature register
  Wire.endTransmission();

  // Request 2 bytes from the TMP112
  Wire.requestFrom(TMP112_ADDRESS, 2);
  if (Wire.available() == 2)
  {
    // Read the temperature data
    byte msb = Wire.read();
    byte lsb = Wire.read();

    // Combine the bytes to get the temperature value
    int tempData = (msb << 4) | (lsb >> 4);

    // Convert the temperature data to Celsius
    float temperature = tempData * 0.0625;
    return temperature;
  }
  else
  {
    Serial.println("Error: Unable to read temperature");
    return 0.0;
  }
}
float temperatur;
void setup()
{
  Wire.begin(I2C_SDA, I2C_SCL, I2C_SPEED);
  Serial.begin(115200);
}

void loop()
{
  while (1)
  {
    temperatur = readTemperature();
    printf("Temp : %f\n", temperatur);
    delay(1000);
  }
}*/
 

//************** I2C Scanner ******************
/*#include <Arduino.h>
#include <Wire.h>

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\nI2C Scanner");
}

void loop()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
         Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
     }
     else if (error==4)
     {
      Serial.print("Unknown error at address 0x");
      if (address<16)
         Serial.print("0");
      Serial.println(address,HEX);
     }
    }
    if (nDevices == 0)
       Serial.println("No I2C devices found\n");
    else
       Serial.println("done\n");
    delay(5000);           // wait 5 seconds for next scan
}*/
