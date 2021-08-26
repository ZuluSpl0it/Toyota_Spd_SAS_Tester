#include <Arduino.h>

#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <U8g2lib.h>

// Define Pins
#define vssPulse PA1

#define CAN_CTRL1_INT PA3
#define CAN_CTRL1_CS PA4

#define spi_Clock PA5
#define spi_MISO PA6
#define spi_MOSI PA7

#define usart_TX PA9
#define usart_RX PA10

#define precision_SW PB3

#define modeSelect_SW PB4

#define onboard_LED PC13

//Define OLED display dimensions
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//check_can
long check_can = 0;

// CAN RX Variables
unsigned long rxId;
unsigned char len;
unsigned char rxBuf[8];

// Serial Output String Buffer
char msgString[128];

//Steering Angle value
short steerAngle = 0;
signed char steerFraction = 0;
unsigned char tempSteerFraction = 0;
short steerRate = 0;
short totalSteerAngle = 0;
unsigned long rxLast = 0;
bool rxValid = false;
bool metric = false;

//VSS signal
const byte interruptPin = vssPulse;
volatile int rpm_Pulse = 0; //volatile bcuz modified in handler
int carSpeed = 0;
int lastCarSpeed = 0;
unsigned long lastPulse_micro = 0;
unsigned long duration = 0;
unsigned char encoder = 0;

// Declare CAN class and set CS pins
MCP_CAN CAN_CTRL1(CAN_CTRL1_CS);

//Initialize display object, the (-1) parameter means
//display doesn’t have a RESET pin
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, PB6, PB7, -1);

/*********************************************************************************************************
** Function name:           get_MsgCAN1
** Descriptions:            Retrieves message from CAN controller
*********************************************************************************************************/
void get_MsgCAN1()
{

  //for debug
  if (digitalRead(CAN_CTRL1_INT) == 0)
  {                                           // If pin is low, read receive buffer
    CAN_CTRL1.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)

    if ((rxId & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

    Serial.print(msgString);

    for (byte i = 0; i < len; i++)
    {
      sprintf(msgString, " 0x%.2X", rxBuf[i]);
      Serial.print(msgString);
    }

    Serial.println("");

    //Get Steering angle from message 0x25
    if (rxId == 0x25)
    {
      rxValid = true;
      rxLast = millis();

      /*
    Name:       STEER_ANGLE_SENSOR
    Id:         HEX = 0x25        Decimal = 37
    Length:     8 bytes
    Cycle time: - ms
    Senders:    XXX
    Layout:
                          Bit

             7   6   5   4   3   2   1   0
           +---+---+---+---+---+---+---+---+
         0 |   |   |   |   |<--------------|
           +---+---+---+---+---+---+---+---+
         1 |------------------------------x|
           +---+---+---+---+---+---+---+---+
                                         └-- STEER_ANGLE   [Min = -500   Max = 500]   (Units = deg)
           +---+---+---+---+---+---+---+---+
         2 |   |   |   |   |   |   |   |   |
           +---+---+---+---+---+---+---+---+
     B   3 |   |   |   |   |   |   |   |   |
     y     +---+---+---+---+---+---+---+---+
     t   4 |<-------------x|<--------------|
     e     +---+---+---+---+---+---+---+---+
                         └-- STEER_FRACTION   [Min = -0.7   Max = 0.7]   (Units = deg)
           +---+---+---+---+---+---+---+---+
         5 |------------------------------x|
           +---+---+---+---+---+---+---+---+
                                         └-- STEER_RATE   [Min = -2000   Max = 2000]   (Units = deg/s)
           +---+---+---+---+---+---+---+---+
         6 |   |   |   |   |   |   |   |   |
           +---+---+---+---+---+---+---+---+
         7 |   |   |   |   |   |   |   |   |
           +---+---+---+---+---+---+---+---+

    Signal tree:

    -- {root}
       └-- STEER_ANGLE
       └-- STEER_FRACTION              Note: 1/15th of the signal STEER_ANGLE, which
                                       is 1.5 deg; note that 0x8 is never set
       └-- STEER_RATE                  Note: factor is tbd
*/

      //Load the two bytes into a short
      steerAngle = int(rxBuf[0] & 0x0F);
      steerAngle = (steerAngle << 8) | rxBuf[1];
      Serial.print("Raw Angle: ");
      Serial.print(steerAngle);
      //Make negative if bit 11 ia set
      if (bitRead(steerAngle, 11) == 1)
      {
        bitClear(steerAngle, 11);
        steerAngle = steerAngle - 2048;
      }
      //Scale value per DBC (Mult by 10 to avoid using float)
      steerAngle = steerAngle * 15; //1.5 * 10
      Serial.print("; Scaled Angle: ");
      Serial.println(steerAngle);

      //Load the byte into a char
      tempSteerFraction = rxBuf[4] & 0xF0;
      tempSteerFraction = tempSteerFraction >> 4;

      Serial.print("Raw Fraction: ");
      Serial.print(tempSteerFraction);
      if (tempSteerFraction > 8)
      {
        steerFraction = tempSteerFraction - 16;
      }
      else
      {
        steerFraction = tempSteerFraction;
      }
      //Scale value per DBC and multiply by 10 to avoid using float)
      steerFraction = steerFraction * 1; //0.1 * 10
      Serial.print("; Scaled Fraction: ");
      Serial.println(steerFraction);

      //Load the two bytes into a short
      steerRate = rxBuf[4] & 0x0F;
      steerRate = (steerRate << 8) | rxBuf[5];
      Serial.print("Raw Rate: ");
      Serial.print(steerRate);
      //Make negative if bit 11 ia set
      if (bitRead(steerRate, 11) == 1)
      {
        bitClear(steerRate, 11);
        steerRate = steerRate - 2048;
      }
      //Scale value per DBC (already integer in message)
      steerRate = steerRate * 1;
      Serial.print("; Scaled Rate: ");
      Serial.println(steerRate);

      if (digitalRead(precision_SW) == 0)
      {
        //Add both data fields to get total degrees
        totalSteerAngle = steerAngle + steerFraction;
      }
      else
      {
        totalSteerAngle = steerAngle;
      }

      Serial.print("Total Angle: ");
      Serial.println(totalSteerAngle);

      Serial.println("");
    }
  }
  else if (digitalRead(CAN_CTRL1_INT == 1))
  {
    if (CAN_CTRL1.checkError() != 0)
    {
      Serial.printf("EFLG (ERROR FLAG REGISTER) = 0x%X\n", CAN_CTRL1.getError());
      Serial.println("");
    }
  }
}

/*********************************************************************************************************
** Function name:           outputShaft_rpm
** Descriptions:            Counts speed signal pulses
*********************************************************************************************************/
void outputShaft_rpm()
{
  rpm_Pulse++;
}

/*********************************************************************************************************
** Function name:           calc_Speed
** Descriptions:            Calculates the average speed over the last 160 readings 
*********************************************************************************************************/
void calc_Speed()
{

  // speed = (1 / your pulses per distance) / (pulse duration in sec)
  // IE if your sensor is 4000 pulses/km then use 0.00025 (1/4000)
  // For Hyundai Santa Fe, 1026 RPM * 4 pulses = 4104 Pulses/Minute @ 60 MPH

  if (rpm_Pulse >= 10)
  {
    //Stop interrupts while calculating speed
    detachInterrupt(interruptPin);

    //Averge length of time between pulses
    duration = (micros() - lastPulse_micro) / rpm_Pulse;

    //Calculate the speed (MPH) if within range (max = 877800 for 0 MPH)
    if (duration < 877200)
    {
      carSpeed = 877193 / duration;

      if (metric)
      {
        carSpeed = carSpeed * 1.60934;
      }

      //Average last two readings
      carSpeed = (carSpeed + lastCarSpeed) / 2;
      if (carSpeed > 999)
      {
        carSpeed = 999;
      }
    }
    else
    {
      carSpeed = 0;
    }

    //Reset the interupt routine
    rpm_Pulse = 0;
    lastPulse_micro = micros();
    lastCarSpeed = carSpeed;
    attachInterrupt(digitalPinToInterrupt(interruptPin), outputShaft_rpm, FALLING);
  }
  else if ((micros() - lastPulse_micro) > 1000000)
  {

    carSpeed = 0;
  }
}

/*********************************************************************************************************
** Function name:           update_display
** Descriptions:            Send date to display
*********************************************************************************************************/

void update_display()
{

  u8g2.clearBuffer();

  if (digitalRead(modeSelect_SW) == 1)
  {
    //Display Spd and Engaged status
    //Display Header
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.setCursor(0, 16);
    u8g2.print("        SPEED:");

    //SPEED stored as Integer, so format and display
    u8g2.setFont(u8g2_font_ncenB18_tr);
    u8g2.setCursor(0, 35);
    u8g2.print("    ");
    if (carSpeed < 10)
    {
      u8g2.print("00");
    }
    else if (carSpeed < 100)
    {
      u8g2.print("0");
    }
    else
    {
      u8g2.print("");
    }
    u8g2.print(carSpeed);
    u8g2.setFont(u8g2_font_ncenB12_tr);
    if (metric)
    {
      u8g2.print(" kmh");
    }
    else
    {
      u8g2.print(" mph");
    }
  }
  else
  {

    //Display Steer Angle Value
    // Display Header
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.setCursor(0, 16);
    u8g2.print("        ANGLE:");

    //Steer Angle stored as Integer, so format output
    u8g2.setFont(u8g2_font_ncenB18_tr);
    u8g2.setCursor(0, 35);
    u8g2.print(" ");

    if (rxValid != true)
    {
      //Display "---.-" if CAN message not received
      u8g2.setFont(u8g2_font_ncenB18_tr);
      u8g2.setCursor(0, 35);
      u8g2.print(" ");
      u8g2.print("   ---.-");
      u8g2.setFont(u8g2_font_ncenB12_tr);
      u8g2.print(" deg");
    }
    else
    {

      if (totalSteerAngle < -1000)
      {
        u8g2.print("-");
      }
      else if (totalSteerAngle < -100)
      {
        u8g2.print("-0");
      }
      else if (totalSteerAngle < 0)
      {
        u8g2.print("-00");
      }
      else if (totalSteerAngle < 100)
      {
        u8g2.print(" 00");
      }
      else if (totalSteerAngle < 1000)
      {
        u8g2.print(" 0");
      }

      u8g2.print(abs(totalSteerAngle / 10));
      u8g2.print(".");
      u8g2.print(abs(totalSteerAngle % 10));
      u8g2.setFont(u8g2_font_ncenB12_tr);
      u8g2.print(" deg");

      u8g2.setFont(u8g2_font_ncenB10_tr);
      u8g2.setCursor(0, 58);

      if (digitalRead(precision_SW) == 0)
      {
        if (steerRate > 1500 || steerRate < -1500)
        {

          u8g2.print("         FAIL !!!");
        }
        else
        {
          //Display Rate Value
          u8g2.print(" Rate: ");
          u8g2.print("  ");
          u8g2.print(steerRate);
          u8g2.print(" deg/s");
        }
      }
    }
  }
  //Refresh display
  u8g2.sendBuffer();
}

/*********************************************************************************************************
** Function name:           setup
** Descriptions:            Run once prior to running loop()
*********************************************************************************************************/
void setup()
{

  //CAN is running at 500,000BPS; 115,200BPS is SLOW
  Serial.begin(115200);

  while (!Serial)
    ;

  Serial.println("SAS ECU Tester");

  //Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN_CTRL1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
  {
    Serial.println("Error Initializing MCP2515!");
  }

  //Set CAN Controllers to Normal mode
  CAN_CTRL1.setMode(MCP_NORMAL);

  //Initialize display (address 0x3C for 128x64)
  u8g2.begin();
  delay(2000); //Delay for display startup
  u8g2.clearBuffer();
  u8g2.sendBuffer();

  //Setup Pins
  pinMode(onboard_LED, OUTPUT);

  //For Speed calculation (via external Interrupt)
  pinMode(vssPulse, INPUT);

  //SPI control and monitor lines
  pinMode(CAN_CTRL1_CS, OUTPUT);
  pinMode(CAN_CTRL1_INT, INPUT);

  //Switches
  pinMode(modeSelect_SW, INPUT_PULLUP);
  pinMode(precision_SW, INPUT_PULLUP);

  //interrupt for VSS calculation
  lastPulse_micro = micros();
  attachInterrupt(digitalPinToInterrupt(interruptPin), outputShaft_rpm, FALLING);
}

/*********************************************************************************************************
** Function name:           loop
** Descriptions:            The main program that will loop forever
*********************************************************************************************************/

void loop()
{

  calc_Speed();

  get_MsgCAN1();

  //Consider steering angle invalid after 250ms
  if (millis() - rxLast > 250)
  {
    rxValid = false;
  }

  update_display();
}
