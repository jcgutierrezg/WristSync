#include "config.h"
#include "BluetoothSerial.h"
#include "WiFi.h"
#include "ESPAsyncWebServer.h"

const char* ssid = "A32";
const char* password = "DkPepper";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

TTGOClass *watch;
TFT_eSPI *tft;

AXP20X_Class *power;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

double AcX_Out, AcY_Out, AcZ_Out, Tmp_Out, GyX_Out[3], GyY_Out[3], GyZ_Out[3], MgX_Out, MgY_Out, MgZ_Out;  //Output processed values accounting sensor range settings

double roll_A[3], pitch_A[3], yaw_A[3];  //
double roll_AC[3], pitch_AC[3], yaw_AC[3];
double roll_filter[3], pitch_filter[3], yaw_filter[3];
double roll_OFFSET, pitch_OFFSET, yaw_OFFSET;
double roll_ABS, pitch_ABS, yaw_ABS;

const double alpha = 0.7;
const double K_p = 25;
const double K_i = 100;
const double T = 0.030;

double Cz2 = 0;
double Cz1 = 0;
double Cz0 = 0;
double Cg2 = 0;
double Cg1 = 0;
double Ca2 = 0;
double Ca1 = 0;
double Ca0 = 0;

unsigned long elapsedTime, currentTime, previousTime, lastTime, lastRead, Tmoving;

int quadrantR = 1;
int quadrantP = 1;
int quadrantY = 1;


const int jumpThresholdR = 60;
const int jumpThresholdP = 60;
const int jumpThresholdY = 60;

int joyX, joyY;

int activeBut = 0;

int DKpair = 1;
int IKaxis = 0;

int currentStatus = 0;
int currentSubstatus = 0;
int commandOut = 0;

String inputMsg = "";

String subMsg[12];
int StringCount = 0;

unsigned long refresh = 20.0;

int state = 0;
int buttonFlag = 0;

int serialFlag = 0;

int startFlag = 0;

int dataInFlag = 0;
int filterCount = 0;

static uint8_t conv2d(const char *p)
{
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

char buf[128];

String BTname = "WristSync Glove";
char *pin = "2111";

bool isconnected;

const int numReadings = 10;     // Number of readings to average
int readings[numReadings];     // Array to store readings
int indexPoint = 0;                 // Index for storing readings
double total = 0;                 // Running total of readings
double average = 0;               // Moving average

void delayMillis(unsigned long tempo)
{
  previousTime = millis();
  currentTime = previousTime;

  while (currentTime - previousTime < tempo)
  {
    if (startFlag == 1)
    {
      //Serial.println(millis() - lastRead);
      processSerialBT();
    }


    if (state == 3)
    {
      if (dataInFlag == 1)
      {
        if (filterCount > 100)
        {
          recalculateParams(average / 1000);
        }
        calculateAccelRPY();
        quadrantize_v2();
        quadrantFlip();
        applyCCF();
        quad2deg();

        Tmoving = millis() - lastTime;
        lastTime = millis();

        // Subtract the oldest reading from the total
        total -= readings[indexPoint];

        // Store the new reading in the array
        readings[indexPoint] = Tmoving;

        // Add the new reading to the total
        total += readings[indexPoint];

        // Move to the next position in the array
        indexPoint++;

        // If the end of the array is reached, wrap around to the beginning
        if (indexPoint >= numReadings) {
          indexPoint = 0;
        }

        // Calculate the moving average
        average = total / numReadings;

        filterCount++;

        dataInFlag = 0;
      }

    }
    currentTime = millis();
  }
}

void setup() {

  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }

  Cz2 = -alpha * T * K_p + alpha * K_i * T * T + 1;
  Cz1 = alpha * T * K_p - 2;
  Cz0 = 1;
  Cg2 = -alpha * T;
  Cg1 = alpha * T;
  Ca2 = -alpha - alpha * T * K_p + alpha * K_i * T * T + 1;
  Ca1 = alpha * T * K_p + 2 * alpha - 2;
  Ca0 = -alpha + 1;

  //Serial.begin(115200);
  SerialBT.begin("WristSync Watch");  //Bluetooth device name
  SerialBT.setPin(pin);
  isconnected = SerialBT.connect(BTname);
  //Serial.println("The device started, now you can pair it with bluetooth!");

  // Get watchClass instance
  watch = TTGOClass::getWatch();

  // Initialize the hardware, the BMA423 sensor has been initialized internally
  watch->begin();

  // Turn on the backlight
  watch->openBL();

  //Receive objects for easy writing
  tft = watch->tft;

  power = watch->power;

  power->adc1Enable(
    AXP202_VBUS_VOL_ADC1 |
    AXP202_VBUS_CUR_ADC1 |
    AXP202_BATT_CUR_ADC1 |
    AXP202_BATT_VOL_ADC1,
    true);

  watch->motor_begin();
  pinMode(TP_INT, INPUT);

  watch->rtc->disableAlarm();

  watch->rtc->setDateTime(2023, 6, 19, conv2d(__TIME__), conv2d(__TIME__ + 3), conv2d(__TIME__ + 6) + 28);

  tft->fillRect(0, 0, 240, 240, TFT_BLACK);
  tft->setCursor(30, 10);
  tft->setTextFont(4);
  tft->setTextSize(3);
  tft->setTextColor(TFT_WHITE);
  tft->println("Wrist");
  tft->setCursor(30, 70);
  tft->println("Sync");

  tft->setTextFont(1);
  tft->setTextSize(3);
  tft->setTextColor(TFT_CYAN);
  tft->setCursor(20, 150);
  tft->println("Juan Camilo");
  tft->setCursor(20, 180);
  tft->println("Gutierrez G.");

  tft->setTextSize(2);
  tft->setTextColor(TFT_YELLOW);
  tft->setCursor(30, 210);
  tft->println("For Robocol Arm");

  delayMillis(4000.0);

  tft->fillRect(0, 0, 240, 240, TFT_BLACK);
  tft->drawLine(10, 20, 230, 20, TFT_WHITE);
  tft->drawLine(10, 220, 230, 220, TFT_WHITE);

  tft->setCursor(10, 40);
  tft->setTextFont(1);
  tft->setTextSize(3);
  tft->setTextColor(TFT_WHITE);
  tft->println("Please touch");
  tft->setCursor(10, 70);
  tft->println("the screen");
  tft->setCursor(10, 100);
  tft->setTextColor(TFT_YELLOW);
  tft->println("when WiFi");
  tft->setCursor(10, 130);
  tft->println("network is");
  tft->setCursor(10, 160);
  tft->println("activated");

  while (state == 0)
  {
    if (digitalRead(TP_INT) == LOW) {
      watch->motor->onec();
      delayMillis(500.0);
      watch->motor->onec();
      delayMillis(500.0);
      watch->motor->onec();
      state = 1;
    }
  }
  tft->fillRect(0, 0, 240, 240, TFT_BLACK);
  tft->drawLine(10, 20, 230, 20, TFT_WHITE);
  tft->drawLine(10, 220, 230, 220, TFT_WHITE);

  tft->setCursor(10, 80);
  tft->setTextFont(1);
  tft->setTextSize(3);
  tft->setTextColor(TFT_WHITE);
  tft->println("Connecting");
  tft->setCursor(10, 120);
  tft->println("to WiFi...");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {

  }

  server.on("/pose", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", sendPose().c_str());
  });
  // Start server
  server.begin();


  tft->fillRect(0, 0, 240, 240, TFT_BLACK);
  tft->drawLine(10, 20, 230, 20, TFT_WHITE);
  tft->drawLine(10, 220, 230, 220, TFT_WHITE);

  tft->setCursor(10, 40);
  tft->setTextFont(1);
  tft->setTextSize(3);
  tft->setTextColor(TFT_YELLOW);
  tft->println("Connected:");
  tft->setTextSize(2);
  tft->setCursor(10, 80);
  tft->println(WiFi.localIP());
  tft->setCursor(10, 130);
  tft->setTextSize(3);
  tft->setTextColor(TFT_WHITE);
  tft->println("Please touch");
  tft->setCursor(10, 160);
  tft->println("the screen");
  tft->setCursor(10, 190);
  tft->println("to start");

  state = 0;

  while (state == 0)
  {
    if (digitalRead(TP_INT) == LOW) {
      watch->motor->onec();
      delayMillis(500.0);
      watch->motor->onec();
      delayMillis(500.0);
      watch->motor->onec();
      state = 1;
    }
  }

  startFlag = 1;
  currentStatus = 1;
  lastTime = millis();

}

void displayPower()
{
  //tft->fillRect(0, 0, 240, 240, TFT_BLACK);
  tft->setCursor(0, 0);
  tft->setTextFont(1);
  tft->setTextSize(2);
  tft->setTextColor(TFT_GREEN);
  tft->print("BATTERY: ");
  tft->print(power->getBattPercentage());
  tft->println("%");

  if (power->isChargeing())
  {
    tft->print("Charge:");
    tft->print(power->getBattChargeCurrent());
    tft->println(" mA");
  }
  else
  {
    // Show current consumption
    tft->print("Discharge:");
    tft->print(power->getBattDischargeCurrent());
    tft->println(" mA");

  }
}


void displayTime()
{

  tft->setTextColor(TFT_WHITE, TFT_BLACK);
  tft->setCursor (20, 50);
  tft->setTextSize(3);
  tft->print(__DATE__);

  tft->setTextColor(TFT_YELLOW, TFT_BLACK);
  tft->setTextSize(1);
  snprintf(buf, sizeof(buf), "%s", watch->rtc->formatDateTime());
  tft->drawString(buf, 5, 80, 7);

}

void loop() {


  tft->fillRect(0, 0, 240, 240, TFT_BLACK);

  determineState();

  delayMillis(500.0);

}

void processSerialBT() {

  if (SerialBT.available()) //and millis() - lastRead > 35.0)
  {
    inputMsg = SerialBT.readStringUntil('\n');
    //Serial.println(inputMsg);

    GyX_Out[2] = GyX_Out[1];
    GyY_Out[2] = GyY_Out[1];
    GyZ_Out[2] = GyZ_Out[1];

    GyX_Out[1] = GyX_Out[0];
    GyY_Out[1] = GyY_Out[0];
    GyZ_Out[1] = GyZ_Out[0];

    // Variables for storing the current value
    double currentValue = 0.0;
    double parsedValue = 0.0;
    bool isDecimal = false;
    double decimalPlace = 1;
    //int decimalCount = 0;
    bool isNegative = false;
    int semicolons = 0;

    // Process each character in the input string
    for (size_t i = 0; i < inputMsg.length(); i++) {
      char c = inputMsg.charAt(i);

      // Check for a negative sign
      if (c == '-') {
        isNegative = true;
        continue;
      }

      // Check for a decimal point
      if (c == '.') {
        isDecimal = true;
        continue;
      }

      // Check for the delimiter character ';'
      if (c == ';') {
        // Multiply the parsed value by -1 if negative
        if (isNegative) {
          parsedValue *= -1;
        }
        if (isDecimal) {
          parsedValue = parsedValue * decimalPlace;
        }

        switch (semicolons) {
          case 0:
            {
              AcX_Out = parsedValue;
              break;
            }
          case 1:
            {
              AcY_Out = parsedValue;
              break;
            }
          case 2:
            {
              AcZ_Out = parsedValue;
              break;
            }
          case 3:
            {
              GyX_Out[0] = parsedValue;
              break;
            }
          case 4:
            {
              GyY_Out[0] = parsedValue;
              break;
            }
          case 5:
            {
              GyZ_Out[0] = parsedValue;
              break;
            }
          case 6:
            {
              MgX_Out = parsedValue;
              break;
            }
          case 7:
            {
              MgY_Out = parsedValue;
              break;
            }
          case 8:
            {
              MgZ_Out = parsedValue;
              break;
            }
          case 9:
            {
              joyX = parsedValue;
              break;
            }
          case 10:
            {
              joyY = parsedValue;
              break;
            }
          case 11:
            {
              activeBut = parsedValue;
              //Serial.println("read");
              lastRead = millis();
              dataInFlag = 1;
              break;
            }
          default:
            break;
        }

        // Reset variables for the next value
        currentValue = 0.0;
        parsedValue = 0.0;
        isDecimal = false;
        decimalPlace = 1;
        isNegative = false;
        semicolons++;

        continue;
      }
      if (c != '-' && c != '.' && c != ';') {
        // Convert character to digit
        int digit = c - '0';

        // Update the current value
        currentValue = currentValue * 10.0 + digit;
        //Serial.println(currentValue);
        parsedValue = currentValue;

        // Update the parsed value
        if (isDecimal) {
          parsedValue = currentValue;
          decimalPlace /= 10;
        }
      }
    }
    serialFlag = 0;
  }
  else if (!SerialBT.available())
  {
    serialFlag++;
  }
}

void calculateAccelRPY() {
  roll_A[0] = atan(AcY_Out / AcZ_Out);
  pitch_A[0] = atan(-AcX_Out / (AcY_Out * sin(roll_A[0]) + AcZ_Out * cos(roll_A[0])));
  yaw_A[0] = atan((MgZ_Out * sin(roll_A[0]) - MgY_Out * cos(roll_A[0])) / (MgX_Out * cos(pitch_A[0]) + MgY_Out * sin(pitch_A[0]) * sin(roll_A[0]) + MgZ_Out * sin(pitch_A[0]) * cos(roll_A[0])));

  roll_A[0] = roll_A[0] * 180 / PI;
  pitch_A[0] = pitch_A[0] * 180 / PI;
  yaw_A[0] = yaw_A[0] * 180 / PI;
}

void applyCCF() {
  roll_AC[2] = roll_AC[1];
  pitch_AC[2] = pitch_AC[1];
  yaw_AC[2] = yaw_AC[1];

  roll_AC[1] = roll_AC[0];
  pitch_AC[1] = pitch_AC[0];
  yaw_AC[1] = yaw_AC[0];

  roll_A[2] = roll_A[1];
  pitch_A[2] = pitch_A[1];
  yaw_A[2] = yaw_A[1];

  roll_A[1] = roll_A[0];
  pitch_A[1] = pitch_A[0];
  yaw_A[1] = yaw_A[0];


  roll_filter[2] = roll_filter[1];
  pitch_filter[2] = pitch_filter[1];
  yaw_filter[2] = yaw_filter[1];

  roll_filter[1] = roll_filter[0];
  pitch_filter[1] = pitch_filter[0];
  yaw_filter[1] = yaw_filter[0];

  roll_filter[0] = -Cz2 * roll_filter[2] - Cz1 * roll_filter[1] + Cg2 * GyX_Out[2] + Cg1 * GyX_Out[1] + Ca2 * roll_AC[2] + Ca1 * roll_AC[1] + Ca0 * roll_AC[0];
  pitch_filter[0] = -Cz2 * pitch_filter[2] - Cz1 * pitch_filter[1] + Cg2 * GyY_Out[2] + Cg1 * GyY_Out[1] + Ca2 * pitch_AC[2] + Ca1 * pitch_AC[1] + Ca0 * pitch_AC[0];
  yaw_filter[0] = -Cz2 * yaw_filter[2] - Cz1 * yaw_filter[1] + Cg2 * GyZ_Out[2] + Cg1 * GyZ_Out[1] + Ca2 * yaw_AC[2] + Ca1 * yaw_AC[1] + Ca0 * yaw_AC[0];
}

void quadrantize_v2() {
  switch (quadrantR) {
    case 1:
      if (abs(roll_A[0]) > jumpThresholdR and abs(roll_A[1]) > jumpThresholdR) {
        if (roll_A[0] >= 0 and roll_A[1] < 0) {
          quadrantR = 2;
        }
      } else if (abs(roll_A[0]) < jumpThresholdR and abs(roll_A[1]) < jumpThresholdR) {
        if (roll_A[0] >= 0 and roll_A[1] < 0) {
          quadrantR = 4;
        }
      }
      break;

    case 2:
      if (abs(roll_A[0]) > jumpThresholdR and abs(roll_A[1]) > jumpThresholdR) {
        if (roll_A[0] < 0 and roll_A[1] >= 0) {
          quadrantR = 1;
        }
      } else if (abs(roll_A[0]) < jumpThresholdR and abs(roll_A[1]) < jumpThresholdR) {
        if (roll_A[0] < 0 and roll_A[1] >= 0) {
          quadrantR = 3;
        }
      }
      break;

    case 3:
      if (abs(roll_A[0]) > jumpThresholdR and abs(roll_A[1]) > jumpThresholdR) {
        if (roll_A[0] >= 0 and roll_A[1] < 0) {
          quadrantR = 4;
        }
      } else if (abs(roll_A[0]) < jumpThresholdR and abs(roll_A[1]) < jumpThresholdR) {
        if (roll_A[0] >= 0 and roll_A[1] < 0) {
          quadrantR = 2;
        }
      }
      break;

    case 4:
      if (abs(roll_A[0]) > jumpThresholdR and abs(roll_A[1]) > jumpThresholdR) {
        if (roll_A[0] < 0 and roll_A[1] >= 0) {
          quadrantR = 3;
        }
      } else if (abs(roll_A[0]) < jumpThresholdR and abs(roll_A[1]) < jumpThresholdR) {
        if (roll_A[0] < 0 and roll_A[1] >= 0) {
          quadrantR = 1;
        }
      }
      break;

    default:
      break;
  }


  switch (quadrantP) {
    case 1:
      if (abs(pitch_A[0]) > jumpThresholdP and abs(pitch_A[1]) > jumpThresholdP) {
        if (pitch_A[0] >= 0 and pitch_A[1] < 0) {
          quadrantP = 2;
        }
      } else if (abs(pitch_A[0]) < jumpThresholdP and abs(pitch_A[1]) < jumpThresholdP) {
        if (pitch_A[0] >= 0 and pitch_A[1] < 0) {
          quadrantP = 4;
        }
      }
      break;

    case 2:
      if (abs(pitch_A[0]) > jumpThresholdP and abs(pitch_A[1]) > jumpThresholdP) {
        if (pitch_A[0] < 0 and pitch_A[1] >= 0) {
          quadrantP = 1;
        }
      } else if (abs(pitch_A[0]) < jumpThresholdP and abs(pitch_A[1]) < jumpThresholdP) {
        if (pitch_A[0] < 0 and pitch_A[1] >= 0) {
          quadrantP = 3;
        }
      }
      break;

    case 3:
      if (abs(pitch_A[0]) > jumpThresholdP and abs(pitch_A[1]) > jumpThresholdP) {
        if (pitch_A[0] >= 0 and pitch_A[1] < 0) {
          quadrantP = 4;
        }
      } else if (abs(pitch_A[0]) < jumpThresholdP and abs(pitch_A[1]) < jumpThresholdP) {
        if (pitch_A[0] >= 0 and pitch_A[1] < 0) {
          quadrantP = 2;
        }
      }
      break;

    case 4:
      if (abs(pitch_A[0]) > jumpThresholdP and abs(pitch_A[1]) > jumpThresholdP) {
        if (pitch_A[0] < 0 and pitch_A[1] >= 0) {
          quadrantP = 3;
        }
      } else if (abs(pitch_A[0]) < jumpThresholdP and abs(pitch_A[1]) < jumpThresholdP) {
        if (pitch_A[0] < 0 and pitch_A[1] >= 0) {
          quadrantP = 1;
        }
      }
      break;

    default:
      break;
  }


  switch (quadrantY) {
    case 1:
      if (abs(yaw_A[0]) > jumpThresholdY and abs(yaw_A[1]) > jumpThresholdY) {
        if (yaw_A[0] < 0 and yaw_A[1] >= 0) {
          quadrantY = 2;
        }
      } else if (abs(yaw_A[0]) < jumpThresholdY and abs(yaw_A[1]) < jumpThresholdY) {
        if (yaw_A[0] < 0 and yaw_A[1] >= 0) {
          quadrantY = 4;
        }
      }
      break;

    case 2:
      if (abs(yaw_A[0]) > jumpThresholdY and abs(yaw_A[1]) > jumpThresholdY) {
        if (yaw_A[0] >= 0 and yaw_A[1] < 0) {
          quadrantY = 1;
        }
      } else if (abs(yaw_A[0]) < jumpThresholdY and abs(yaw_A[1]) < jumpThresholdY) {
        if (yaw_A[0] >= 0 and yaw_A[1] < 0) {
          quadrantY = 3;
        }
      }
      break;

    case 3:
      if (abs(yaw_A[0]) > jumpThresholdY and abs(yaw_A[1]) > jumpThresholdY) {
        if (yaw_A[0] < 0 and yaw_A[1] >= 0) {
          quadrantY = 4;
        }
      } else if (abs(yaw_A[0]) < jumpThresholdY and abs(yaw_A[1]) < jumpThresholdY) {
        if (yaw_A[0] < 0 and yaw_A[1] >= 0) {
          quadrantY = 2;
        }
      }
      break;

    case 4:
      if (abs(yaw_A[0]) > jumpThresholdY and abs(yaw_A[1]) > jumpThresholdY) {
        if (yaw_A[0] >= 0 and yaw_A[1] < 0) {
          quadrantY = 3;
        }
      } else if (abs(yaw_A[0]) < jumpThresholdY and abs(yaw_A[1]) < jumpThresholdY) {
        if (yaw_A[0] >= 0 and yaw_A[1] < 0) {
          quadrantY = 1;
        }
      }
      break;

    default:
      break;
  }
}

void quadrantFlip() {

  //ROLL:

  if (quadrantR == 1 or quadrantR == 4) {
    roll_AC[0] = -roll_A[0];
  } else if (quadrantR == 2 or quadrantR == 3) {
    roll_AC[0] = roll_A[0];
  }

  //PITCH:

  if (quadrantP == 1 or quadrantP == 4) {
    pitch_AC[0] = -pitch_A[0];
  } else if (quadrantP == 2 or quadrantP == 3) {
    pitch_AC[0] = pitch_A[0];
  }

  //YAW:

  if (quadrantY == 2 or quadrantY == 3) {
    yaw_AC[0] = -yaw_A[0];
  } else if (quadrantY == 1 or quadrantY == 4) {
    yaw_AC[0] = yaw_A[0];
  }
}

void quad2deg() {
  //ROLL:

  if (quadrantR == 1) {
    roll_ABS = roll_filter[0];
  } else if (quadrantR == 2) {
    roll_ABS = 180 - roll_filter[0];
  } else if (quadrantR == 3) {
    roll_ABS = -180 - roll_filter[0];
  } else if (quadrantR == 4) {
    roll_ABS = roll_filter[0];
  }

  //PITCH:

  if (quadrantP == 1) {
    pitch_ABS = pitch_filter[0];
  } else if (quadrantP == 2) {
    pitch_ABS = 180 - pitch_filter[0];
  } else if (quadrantP == 3) {
    pitch_ABS = -180 - pitch_filter[0];
  } else if (quadrantP == 4) {
    pitch_ABS = pitch_filter[0];
  }

  //YAW:

  if (quadrantY == 1) {
    yaw_ABS = yaw_filter[0];
  } else if (quadrantY == 2) {
    yaw_ABS = 180 - yaw_filter[0];
  } else if (quadrantY == 3) {
    yaw_ABS = -180 - yaw_filter[0];
  } else if (quadrantY == 4) {
    yaw_ABS = yaw_filter[0];
  }
}

void determineState()
{
  if (serialFlag > 50000)
  {
    displayTime();
    displayPower();

    tft->drawLine(10, 220, 230, 220, TFT_WHITE);
    tft->setCursor(10, 130);
    tft->setTextFont(1);
    tft->setTextSize(3);
    tft->setTextColor(TFT_RED);
    tft->println("GLOVE ERROR");
    tft->setCursor(10, 160);
    tft->println("DISCONNECTED");
    tft->setCursor(20, 190);
    tft->setTextColor(TFT_WHITE);
    tft->println("-WristSync-");
    isconnected = SerialBT.connect(BTname);
  }
  else
  {
    switch (activeBut)
    {
      case 0:
        switch (state)
        {
          case 1: //Null

            displayTime();
            displayPower();

            tft->drawLine(10, 220, 230, 220, TFT_WHITE);
            tft->setCursor(40, 150);
            tft->setTextFont(1);
            tft->setTextSize(3);
            tft->setTextColor(TFT_WHITE);
            tft->println("Mode: NULL");
            tft->setCursor(20, 190);
            tft->println("-WristSync-");

            currentStatus = 1;


            break;
          case 2: //IK

            displayPower();

            tft->drawLine(10, 220, 230, 220, TFT_WHITE);
            tft->setCursor(50, 60);
            tft->setTextFont(1);
            tft->setTextSize(3);
            tft->setTextColor(TFT_WHITE);
            tft->println("Mode: IK");

            if (IKaxis == 0)
            {
              tft->setCursor(20, 100);
              tft->setTextColor(TFT_GREEN);
              tft->println("Forward/Back");
              currentSubstatus = 1;
            }
            else
            {
              tft->setCursor(40, 100);
              tft->setTextColor(TFT_CYAN);
              tft->println("Up/Down");
              currentSubstatus = 2;
            }

            tft->setCursor(20, 190);
            tft->setTextColor(TFT_WHITE);
            tft->println("-WristSync-");

            currentStatus = 2;

            break;
          case 3: //RPY

            displayPower();

            tft->drawLine(10, 220, 230, 220, TFT_WHITE);

            // Show the data

            tft->setCursor(40, 45);
            tft->setTextColor(TFT_WHITE);
            tft->setTextSize(3);
            tft->println("Mode: RPY");
            tft->setCursor(20, 90);
            tft->setTextColor(TFT_RED);
            tft->setTextFont(1);
            tft->setTextSize(3);
            tft->print("ROLL: ");
            tft->println(int(roll_ABS));
            tft->setCursor(20, 120);
            tft->setTextColor(TFT_GREEN);
            tft->print("PITCH: ");
            tft->println(int(pitch_ABS));
            tft->setCursor(20, 150);
            tft->setTextColor(TFT_CYAN);
            tft->print("YAW: ");
            tft->println(int(yaw_ABS));
            tft->setCursor(20, 190);
            tft->setTextColor(TFT_WHITE);
            tft->println("-WristSync-");

            currentStatus = 3;

            break;
          case 4: //Home

            displayPower();

            tft->drawLine(10, 220, 230, 220, TFT_WHITE);
            tft->setCursor(30, 150);
            tft->setTextFont(1);
            tft->setTextSize(3);
            tft->setTextColor(TFT_WHITE);
            tft->println("Mode: Home");
            tft->setCursor(20, 190);
            tft->println("-WristSync-");

            currentStatus = 4;

            break;
          case 5: //DK

            displayPower();

            tft->drawLine(10, 220, 230, 220, TFT_WHITE);
            tft->setCursor(40, 60);
            tft->setTextFont(1);
            tft->setTextSize(3);
            tft->setTextColor(TFT_WHITE);
            tft->println("Mode: DK");
            tft->setCursor(30, 90);
            tft->println("Selected:");
            tft->setTextSize(5);

            if (DKpair == 1)
            {
              tft->setCursor(50, 130);
              tft->setTextColor(TFT_YELLOW);
              tft->println("1, 2");
              currentSubstatus = 1;
            }
            else if (DKpair == 2)
            {
              tft->setCursor(50, 130);
              tft->setTextColor(TFT_GREEN);
              tft->println("3, 4");
              currentSubstatus = 2;
            }
            else
            {
              tft->setCursor(50, 130);
              tft->setTextColor(TFT_CYAN);
              tft->println("5, 6");
              currentSubstatus = 3;
            }

            tft->setCursor(20, 190);
            tft->setTextColor(TFT_WHITE);
            tft->setTextSize(3);
            tft->println("-WristSync-");

            currentStatus = 5;

            break;
          case 6: //Disable


            displayPower();

            tft->drawLine(10, 220, 230, 220, TFT_WHITE);
            tft->setCursor(10, 150);
            tft->setTextFont(1);
            tft->setTextSize(3);
            tft->setTextColor(TFT_WHITE);
            tft->println("Mode: Disable");
            tft->setCursor(20, 190);
            tft->println("-WristSync-");

            currentStatus = 6;

            break;

          default:

            break;

        }
        break;
      case 1:
        watch->motor->onec();
        switch (state)
        {
          case 1: //Null, pressed 1

            tft->fillRect(0, 0, 240, 240, TFT_BLACK);

            tft->drawLine(10, 20, 230, 20, TFT_WHITE);
            tft->drawLine(10, 220, 230, 220, TFT_WHITE);

            tft->setCursor(10, 40);
            tft->setTextFont(1);
            tft->setTextSize(2);
            tft->setTextColor(TFT_WHITE);
            tft->println("Unable to plan");
            tft->setCursor(10, 70);
            tft->println("or execute");
            tft->setCursor(10, 120);
            tft->println("Please select an");
            tft->setCursor(10, 150);
            tft->println("operating mode");
            tft->setCursor(10, 180);
            tft->println("first");
            delayMillis(2000.0);

            break;
          case 2: //IK, pressed 1

            tft->fillRect(0, 0, 240, 240, TFT_BLACK);

            tft->drawLine(10, 20, 230, 20, TFT_WHITE);
            tft->drawLine(10, 220, 230, 220, TFT_WHITE);

            tft->setCursor(10, 40);
            tft->setTextFont(1);
            tft->setTextSize(2);
            tft->setTextColor(TFT_WHITE);
            tft->println("Attempting to");
            tft->setCursor(10, 70);
            tft->println("plan or execute");
            tft->setCursor(10, 120);
            tft->println("Please check");
            tft->setCursor(10, 150);
            tft->println("RVIZ and terminal");
            tft->setCursor(10, 180);
            tft->println("for results");
            delayMillis(2000.0);

            break;
          case 3: //RPY, pressed 1

            tft->fillRect(0, 0, 240, 240, TFT_BLACK);

            tft->drawLine(10, 20, 230, 20, TFT_WHITE);
            tft->drawLine(10, 220, 230, 220, TFT_WHITE);

            tft->setCursor(10, 40);
            tft->setTextFont(1);
            tft->setTextSize(2);
            tft->setTextColor(TFT_WHITE);
            tft->println("Attempting to");
            tft->setCursor(10, 70);
            tft->println("plan or execute");
            tft->setCursor(10, 120);
            tft->println("Please check");
            tft->setCursor(10, 150);
            tft->println("RVIZ and terminal");
            tft->setCursor(10, 180);
            tft->println("for results");
            delayMillis(2000.0);

            break;
          case 4: //Home, pressed 1

            tft->fillRect(0, 0, 240, 240, TFT_BLACK);

            tft->drawLine(10, 20, 230, 20, TFT_WHITE);
            tft->drawLine(10, 220, 230, 220, TFT_WHITE);

            tft->setCursor(10, 40);
            tft->setTextFont(1);
            tft->setTextSize(2);
            tft->setTextColor(TFT_WHITE);
            tft->println("Attempting to");
            tft->setCursor(10, 70);
            tft->println("plan or execute");
            tft->setCursor(10, 120);
            tft->println("Please check");
            tft->setCursor(10, 150);
            tft->println("RVIZ and terminal");
            tft->setCursor(10, 180);
            tft->println("for results");
            delayMillis(2000.0);

            break;
          case 5: //DK, pressed 1

            tft->fillRect(0, 0, 240, 240, TFT_BLACK);

            tft->drawLine(10, 20, 230, 20, TFT_WHITE);
            tft->drawLine(10, 220, 230, 220, TFT_WHITE);

            tft->setCursor(10, 40);
            tft->setTextFont(1);
            tft->setTextSize(2);
            tft->setTextColor(TFT_WHITE);
            tft->println("Executing direct");
            tft->setCursor(10, 70);
            tft->println("kinematic inputs");
            tft->setCursor(10, 120);
            tft->println("Please watch out");
            tft->setCursor(10, 150);
            tft->println("for the Arm");
            tft->setCursor(10, 180);
            tft->println("self-collisions");
            delayMillis(2000.0);

            break;
          case 6: //Disable, pressed 1

            tft->fillRect(0, 0, 240, 240, TFT_BLACK);

            tft->drawLine(10, 20, 230, 20, TFT_WHITE);
            tft->drawLine(10, 220, 230, 220, TFT_WHITE);

            tft->setCursor(10, 40);
            tft->setTextFont(1);
            tft->setTextSize(2);
            tft->setTextColor(TFT_WHITE);
            tft->println("Unable to plan");
            tft->setCursor(10, 70);
            tft->println("or execute");
            tft->setCursor(10, 120);
            tft->println("Please exit");
            tft->setCursor(10, 150);
            tft->println("disabled mode");
            tft->setCursor(10, 180);
            tft->println("to continue");
            delayMillis(2000.0);

            break;

          default:

            break;
        }
        break;

      case 2: //Gripper
        watch->motor->onec();

        tft->fillRect(0, 0, 240, 240, TFT_BLACK);

        tft->drawLine(10, 20, 230, 20, TFT_WHITE);
        tft->drawLine(10, 220, 230, 220, TFT_WHITE);

        tft->setCursor(10, 80);
        tft->setTextFont(1);
        tft->setTextSize(2);
        tft->setTextColor(TFT_WHITE);
        tft->println("Triggering");
        tft->setCursor(10, 110);
        tft->println("end-effector");
        tft->setCursor(10, 140);
        tft->println("Grip or laser");
        delayMillis(2000.0);

        break;
      case 3:
        watch->motor->onec();

        if (state == 1)
        {
          state = 4;
          delayMillis(500.0);
        }
        else
        {
          state = 1;
          delayMillis(500.0);
        }
        break;
      case 4: //
        watch->motor->onec();
        state = 3;
        delayMillis(500.0);
        break;
      case 5:
        watch->motor->onec();

        if (state == 2)
        {
          if (IKaxis == 0)
          {
            IKaxis = 1;
          }
          else
          {
            IKaxis = 0;
          }
        }
        else if (state == 5)
        {
          if (DKpair == 1 or DKpair == 2)
          {
            DKpair++;
          }
          else
          {
            DKpair = 1;
          }
        }

        else
        {
          state = 2;
        }
        delayMillis(500.0);

        break;

      case 6:
        watch->motor->onec();
        state = 5;
        delayMillis(500.0);
        break;

      default:
        break;
    }
  }
}



String sendPose() {
  // ....
  //unsigned long miltime = millis();
  String message = String(roll_ABS, 2) + ";" + String(pitch_ABS, 2) + ";" + String(yaw_ABS, 2) + ";" + String(joyX) + ";" + String(joyY) + ";" + String(activeBut) + ";" + String(currentStatus) + ";" + String(currentSubstatus);
  //String message = String(miltime)+","+String(roll_filter[0],2)+","+String(pitch_filter[0],2)+","+String(yaw_filter[0],2)+","+String(roll_AC[0],2)+","+String(pitch_AC[0],2)+","+String(yaw_AC[0],2)+","+String(roll_ABS,2)+","+String(pitch_ABS,2)+","+String(yaw_ABS,2);

  return message;
}

void recalculateParams(double newT)
{
  Cz2 = -alpha * newT * K_p + alpha * K_i * newT * newT + 1;
  Cz1 = alpha * newT * K_p - 2;
  Cz0 = 1;
  Cg2 = -alpha * newT;
  Cg1 = alpha * newT;
  Ca2 = -alpha - alpha * newT * K_p + alpha * K_i * newT * newT + 1;
  Ca1 = alpha * newT * K_p + 2 * alpha - 2;
  Ca0 = -alpha + 1;
}
