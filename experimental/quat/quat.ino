#include <SparkFunMPU9250-DMP.h>

#define    MPU9250_ADDRESS            0x68

#define    DISPLAY_PINS              true
#define    DISPLAY_CUAT              true

// Read Nbytes bytes from I2C device at Address and write bytes starting at Register to the Data array
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

MPU9250_DMP imu;

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);

  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);

  if (imu.begin() != INV_SUCCESS) {
    while(true) {
      SerialUSB.println("Unable to communicate with MPU-9250");
      delay(1000);
    }
  }

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               10);

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

  // Display values

  if (DISPLAY_CUAT) {
    if ( imu.fifoAvailable() ) {
      if ( imu.dmpUpdateFifo() == INV_SUCCESS)
      {
        imu.computeEulerAngles();

        float q0 = imu.calcQuat(imu.qw);
        float q1 = imu.calcQuat(imu.qx);
        float q2 = imu.calcQuat(imu.qy);
        float q3 = imu.calcQuat(imu.qz);
    
        SerialUSB.print("CUAT\t");
        SerialUSB.print(String(q0) + "\t" + String(q1) + "\t" + String(q2) + "\t" + String(q3) + "\t");
        SerialUSB.println("");
      }
    }
  }

  if(DISPLAY_PINS) {
    sprintf(aPrintBuffer, "Fingers\t%d\t%d\t%d\t%d\t%d", sensorValue0, sensorValue1, sensorValue2, sensorValue3, sensorValue4);
    SerialUSB.print(aPrintBuffer);
    SerialUSB.println("");
  }
 
  delay(30);
}
