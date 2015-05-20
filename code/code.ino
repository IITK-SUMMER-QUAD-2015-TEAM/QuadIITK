/*Gyro output rate: 8kHz (if we do'nt use DLPF and 1kHz if do use DLPF)
**Accelo output rate: 1kHz*/

#define BAUD_RATE 115200 //general baud rates: 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, and 115200

/*All of the below regiater addresses are for the most significant bits, i.e. for bits[15:8]*/

#define GYRO_XOUT_REGISTER  0x43
#define GYRO_YOUT_REGISTER  0x45
#define GYRO_ZOUT_REGISTER  0x47

#define ACCELO_XOUT_REGISTER 0x3B
#define ACCELO_YOUT_REGISTER 0x3D
#define ACCELO_ZOUT_REGISTER 0x3F

#define TEMP_OUT_REGISTER 0x41

/*The below register addresses are for the configuration bits of the gyrometer and accelometer ranges*/
#define GYRO_CONFIG_REGISTER 0x1B
#define FS_SEL_BIT 3//0-> +-250 deg/s 1-> +-500 deg/s 0-> +-1000 deg/s 0-> +-2000 deg/s

#define ACCEL0_CONFIG_REGISTER 0x1C
#define AFS_SEL_BIT 3//0-> +-2g 1-> +-4g 0-> +-8g 0-> +-16g

/*One of the power registers.  Contains the Sleep mode bit. The sleep mode bit should be set to 0 for sleep mode to be deactivated.*/
#define PWR_MGMT_1_REGISTER 0x6B

#define MPU_ADDRESS 0x68 // Can be made 0x69 as well using the AD0 pin of the IMU.

#define CONFIG_REGISTER 0x1A //The register containing the DLPF bits.

#define CONFIG_VALUE  1  //0->260Hz  1->184Hz  2->94Hz  3->44Hz  4->21Hz  5->260Hz  6->10Hz  7->5Hz
//It is should be noted that decreasing the bandwidth increases the delay.
/*The dividing factors. They change with different modes of operation of the gyro and accelerometer*/
#define ACCELO_DIVIDING_FACTOR 16384//2g->16384  4g->8192  8g->4096  16g->2048
#define GYRO_DIVIDING_FACTOR 131  //250dps->131  500dps->65.5  1000dps->32.8  2000dps->16.4 

/*For temperature sensor*/
#define TEMP_DIVIDING_FACTOR 340
#define TEMP_OFFSET  36.53

#include <Wire.h>

void initI2CMPU(void);
void getMPUValues(void);

void printMPUValues(void);
int16_t Ax, Ay, Az, Gx, Gy, Gz, temperature;

extern void initReceiver(void);
extern void printReceiverInput(void);

void setup()
{
  initI2CMPU();
  initReceiver();
  Serial.begin(BAUD_RATE);
}

void loop()
{
  getMPUValues();
  printReceiverInput();
  printMPUValues();
}

void initI2CMPU(void) //Begins I2C communication with  MPU using it's internal 8MHz oscillator for CLKSEL and also wakes up the MPU.
{
  Wire.begin(MPU_ADDRESS);

  Wire.beginTransmission(MPU_ADDRESS);

  Wire.write(PWR_MGMT_1_REGISTER);
  Wire.write(0);

  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDRESS);

  Wire.write(CONFIG_REGISTER);
  Wire.write(CONFIG_VALUE);

  Wire.endTransmission(true);

}

void getMPUValues(void)
{
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(ACCELO_XOUT_REGISTER);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDRESS, 14, true);

  while (!Wire.available());

  Ax = (Wire.read() << 8) | Wire.read();
  Ay = (Wire.read() << 8) | Wire.read();
  Az = (Wire.read() << 8) | Wire.read();

  temperature = (Wire.read() << 8) | Wire.read();

  Gx = (Wire.read() << 8) | Wire.read();
  Gy = (Wire.read() << 8) | Wire.read();
  Gz = (Wire.read() << 8) | Wire.read();
}

void printMPUValues(void)
{
  Serial.print((double)Gx / GYRO_DIVIDING_FACTOR); Serial.print('\t');Serial.print((double)Gy / GYRO_DIVIDING_FACTOR);Serial.print('\t');  Serial.print((double)Gz / GYRO_DIVIDING_FACTOR); Serial.print('\t');Serial.print((double)Ax / ACCELO_DIVIDING_FACTOR);Serial.print('\t');Serial.print((double)Ay / ACCELO_DIVIDING_FACTOR);Serial.print('\t');Serial.print((double)Az / ACCELO_DIVIDING_FACTOR);Serial.println((double)temperature / TEMP_DIVIDING_FACTOR + TEMP_OFFSET);
}
