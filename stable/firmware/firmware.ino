#include <Wire.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define    DISPLAY_IMU               true
#define    DISPLAY_PINS              true

// Read Nbytes bytes from I2C device at Address and write bytes starting at Register to the Data array
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

uint32_t analogReadFast(uint32_t pin)
{
  uint32_t valueRead = 0;

  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber;

  // Enable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->CTRLA.bit.ENABLE = 0x01;             
  
  // Start conversion
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again since the first conversion after the reference is changed must not be used
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->SWTRIG.bit.START = 1;

  // Store the value
  while (ADC->INTFLAG.bit.RESRDY == 0);     // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;

  while (ADC->STATUS.bit.SYNCBUSY == 1);
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1);

  return valueRead;
}

// Variables to store values coming from sensors
int sensorValue0 = 0;
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int sensorValue4 = 0;

char aPrintBuffer[255];


// Write a Data to the device at Address at Register
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  
  Wire.write(Data);
  Wire.endTransmission();
}

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);

  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);


  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);

  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);

  SerialUSB.println("12345");

  analogReadResolution(10);
  analogReference(AR_INTERNAL1V65);

  pinMode(13, OUTPUT);
}

// Main loop, read and display data
void loop()
{  
  sensorValue0 = analogReadFast(A0);
  sensorValue1 = analogReadFast(A1);
  sensorValue2 = analogReadFast(A2);
  sensorValue3 = analogReadFast(A3);
  sensorValue4 = analogReadFast(A4);
  
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

  // Create 16 bits values from 8 bits data

  // Accelerometer
  int16_t ax = -(Buf[0] << 8 | Buf[1]);
  int16_t ay = -(Buf[2] << 8 | Buf[3]);
  int16_t az = Buf[4] << 8 | Buf[5];

  // Gyroscope
  int16_t gx = -(Buf[8] << 8 | Buf[9]);
  int16_t gy = -(Buf[10] << 8 | Buf[11]);
  int16_t gz = Buf[12] << 8 | Buf[13];

  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
  }
  while (!(ST1 & 0x01));

  // Read magnetometer data
  uint8_t Mag[7];
  I2Cread(MAG_ADDRESS, 0x03, 7, Mag);


  // Create 16 bits values from 8 bits data

  // Magnetometer
  int16_t mx = -(Mag[3] << 8 | Mag[2]);
  int16_t my = -(Mag[1] << 8 | Mag[0]);
  int16_t mz = -(Mag[5] << 8 | Mag[4]);

  if (DISPLAY_IMU) {
    SerialUSB.print("IMU\t");
    SerialUSB.print(String(ax) + "\t" + String(ay) + "\t" + String(az) + "\t");
    SerialUSB.print(String(gx) + "\t" + String(gy) + "\t" + String(gz) + "\t");
    SerialUSB.print(String(mx + 200) + "\t" + String(my - 70) + "\t" + String(mz - 700));
    SerialUSB.println("");
  }

  if(DISPLAY_PINS) {
    sprintf(aPrintBuffer, "Fingers\t%d\t%d\t%d\t%d\t%d", sensorValue0, sensorValue1, sensorValue2, sensorValue3, sensorValue4);
    SerialUSB.print(aPrintBuffer);
    SerialUSB.println("");
  }
 
  delay(30);
}
