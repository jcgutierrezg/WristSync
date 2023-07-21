#include <TinyWireM.h>
#include <SendOnlySoftwareSerial.h>

const int Tx = 7; //SoftwareSerial TX

SendOnlySoftwareSerial SWSerial(Tx);

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ; //MSB 8bit RAW values from the MPU
double AcX_Out, AcY_Out, AcZ_Out, GyX_Out, GyY_Out, GyZ_Out, MgX_Out, MgY_Out, MgZ_Out; //Output processed values accounting sensitivity settings

const int AccGyr_CLICK = 0x6A; // I2C address of the MPU-9250
const int Mag_CLICK = 0x1C;

double elapsedTime, currentTime, previousTime;

const double maxRes = 32768;

const int macroSens = 0;
double scale_G = 0;
double LSB_G = 0;
double scale_A = 0;
double LSB_A = 0;
double scale_M = 0;
double LSB_M = 0;

const int buttonRange = 5;

int joyX, joyY, buttons;

int activeBut = 0;

const int IND_1 = 256;
const int IND_2 = 204;
const int IND_3 = 170;

const int MID_1 = 1023;
const int MID_2 = 162;
const int MID_3 = 341;

void setup() {
  SWSerial.begin(38400);

  delay(2000);

  TinyWireM.begin(); // sda, scl, clock speed

  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);

  initiate_IMU_CLICK(); //4%

}

void loop() {

  readSensor_CLICK(); //12%
  readAnalogs(); //1%

  previousTime = currentTime;

  currentTime = millis();

  elapsedTime = currentTime - previousTime;

  //  SWSerial.print(elapsedTime);
  //  SWSerial.print(", ");

  SWSerial.print(AcX_Out);
  SWSerial.print(";");
  SWSerial.print(AcY_Out);
  SWSerial.print(";");
  SWSerial.print(AcZ_Out);
  SWSerial.print(";");
  SWSerial.print(GyX_Out);
  SWSerial.print(";");
  SWSerial.print(GyY_Out);
  SWSerial.print(";");
  SWSerial.print(GyZ_Out);
  SWSerial.print(";");
  SWSerial.print(MgX_Out);
  SWSerial.print(";");
  SWSerial.print(MgY_Out);
  SWSerial.print(";");
  SWSerial.print(MgZ_Out);
  SWSerial.print(";");
  SWSerial.print(joyX);
  SWSerial.print(";");
  SWSerial.print(joyY);
  SWSerial.print(";");
  SWSerial.print(activeBut);
  SWSerial.print(";");
  SWSerial.print(elapsedTime);
  SWSerial.println(";");
  //  SWSerial.print(buttons);
  //  SWSerial.println(";");


}

void initiate_IMU_CLICK()
{

  //SENSITIVITY SET FOR MPU

  TinyWireM.beginTransmission(AccGyr_CLICK);
  TinyWireM.write(0x10);  // GYRO_CONFIG register

#if macroSens == 0
  TinyWireM.write(0b01000000); // Bit 3,4: 0 -> ±245 °/s, 133.75 LSB/°/s
  scale_G = 245;
#elif macroSens == 1
  TinyWireM.write(0b01001000); // Bit 3,4: 1 -> ±500 °/s, 65.5 LSB/°/s
  scale_G = 500;
#elif (macroSens == 2 || macroSens == 3)
  TinyWireM.write(0b01011000); // Bit 3,4: 2/3 -> ±2000 °/s, 16.38 LSB/°/s
  scale_G = 2000;
#else
  TinyWireM.write(0b01000000); // Default: Bit 3,4: 0 -> ±245 °/s, 133.75 LSB/°/s
  scale_G = 245;
#endif

  LSB_G = maxRes / scale_G;
  TinyWireM.endTransmission(true);

  TinyWireM.beginTransmission(AccGyr_CLICK);
  TinyWireM.write(0x20);  // ACCEL_CONFIG register

#if macroSens == 0
  TinyWireM.write(0b00000000); // Bit 3,4: 0 -> ±2g, 16384 LSB/g
  scale_A = 2;
#elif macroSens == 1
  TinyWireM.write(0b00001000); // Bit 3,4: 1 -> ±4g, 8192 LSB/g
  scale_A = 4;
#elif macroSens == 2
  TinyWireM.write(0b00010000); // Bit 3,4: 2 -> ±8g, 4096 LSB/g
  scale_A = 8;
#elif macroSens == 3
  TinyWireM.write(0b00011000); // Bit 3,4: 3 -> ±16g, 2048 LSB/g
  scale_A = 16;
#else
  TinyWireM.write(0b00000000); // Bit 3,4: 0 -> ±2g, 16384 LSB/g
  scale_A = 2;
#endif

  LSB_A = maxRes / scale_A;
  TinyWireM.endTransmission(true);

  TinyWireM.beginTransmission(Mag_CLICK);
  TinyWireM.write(0x21);  // MAGN_CONFIG register

#if macroSens == 0
  TinyWireM.write(0b00000000); // Bit 5,6: 0 -> ±4G, 8192 LSB/G
  scale_M = 4;
#elif macroSens == 1
  TinyWireM.write(0b00100000); // Bit 5,6: 1 -> ±8G, 4096 LSB/G
  scale_M = 8;
#elif macroSens == 2
  TinyWireM.write(0b01000000); // Bit 5,6: 2 -> ±12G, 2730.66 LSB/G
  scale_M = 12;
#elif macroSens == 3
  TinyWireM.write(0b01100000); // Bit 5,6: 3 -> ±16G, 2048 LSB/G
  scale_M = 16;
#else
  TinyWireM.write(0b01000000); // Bit 3,4: 0 -> ±2g, 8192 LSB/G
  scale_M = 4;
#endif

  LSB_M = maxRes / scale_M;
  TinyWireM.endTransmission(true);

  TinyWireM.beginTransmission(Mag_CLICK);
  TinyWireM.write(0x22);
  TinyWireM.write(0b00000000);
  TinyWireM.endTransmission(true);
}

void readSensor_CLICK()
{
  TinyWireM.beginTransmission(AccGyr_CLICK);
  TinyWireM.write(0x28);
  TinyWireM.endTransmission(true);
  TinyWireM.requestFrom(AccGyr_CLICK, 6);

  AcX = TinyWireM.read();
  AcX_Out = (AcX | (TinyWireM.read() << 8)) / LSB_A;
  AcY = TinyWireM.read();
  AcY_Out = (AcY | (TinyWireM.read() << 8)) / LSB_A;
  AcZ = TinyWireM.read();
  AcZ_Out = (AcZ | (TinyWireM.read() << 8)) / LSB_A;

  TinyWireM.beginTransmission(AccGyr_CLICK);
  TinyWireM.write(0x18);
  TinyWireM.endTransmission(true);
  TinyWireM.requestFrom(AccGyr_CLICK, 6);

  GyX = TinyWireM.read();
  GyX_Out = (GyX | (TinyWireM.read() << 8)) / LSB_G;
  GyY = TinyWireM.read();
  GyY_Out = (GyY | (TinyWireM.read() << 8)) / LSB_G;
  GyZ = TinyWireM.read();
  GyZ_Out = (GyZ | (TinyWireM.read() << 8)) / LSB_G;

  TinyWireM.beginTransmission(Mag_CLICK);
  TinyWireM.write(0x28);
  TinyWireM.endTransmission(true);
  TinyWireM.requestFrom(Mag_CLICK, 6);

  MgX = TinyWireM.read();
  MgX_Out = (MgX | (TinyWireM.read() << 8)) / LSB_M;
  MgY = TinyWireM.read();
  MgY_Out = (MgY | (TinyWireM.read() << 8)) / LSB_M;
  MgZ = TinyWireM.read();
  MgZ_Out = (MgZ | (TinyWireM.read() << 8)) / LSB_M;
}

void readAnalogs()
{
  joyX = analogRead(2);
  joyY = analogRead(1);

  buttons = analogRead(3);

  if (buttons <= IND_1 + buttonRange and buttons >= IND_1 - buttonRange)
  {
    activeBut = 1;
  }
  else if (buttons <= IND_2 + buttonRange and buttons >= IND_2 - buttonRange)
  {
    activeBut = 2;
  }
  else if (buttons <= IND_3 + buttonRange and buttons >= IND_3 - buttonRange)
  {
    activeBut = 3;
  }
  else if (buttons <=  MID_1 + buttonRange and buttons >= MID_1 - buttonRange)
  {
    activeBut = 4;
  }
  else if (buttons <= MID_2 + buttonRange and buttons >= MID_2 - buttonRange)
  {
    activeBut = 5;
  }
  else if (buttons <= MID_3 + buttonRange and buttons >= MID_3 - buttonRange)
  {
    activeBut = 6;
  }
  else
  {
    activeBut = 0;
  }

}
