#include "SPI.h"

/////////////////
// Arduino Micro
const unsigned int SLAVE_SELECT_PIN = SS; // Slave Select or Chip Select
const unsigned int BOARD_INT0 = 0x00;     // Arduino Micro Interrupt 0
const unsigned int BOARD_INT1 = 0x01;     // Arduino Micro Interrupt 1

int LED_PIN = 13;    // Pin 13 has an LED connected
volatile bool ledState = false; // LED state false = off, true = on

/////////////////
// MMA7455
const unsigned char SPI_READ = 0x00; // Bit 7 = 0 for reading
const unsigned char SPI_WRITE = 0x80; // Bit 7 = 1 for writing

const unsigned char REG_XOUTL = 0x00; // 10 bits output value X LSB
const unsigned char REG_XOUTH = 0x01; // 10 bits output value X MSB
const unsigned char REG_YOUTL = 0x02; // 10 bits output value Y LSB
const unsigned char REG_YOUTH = 0x03; // 10 bits output value Y MSB
const unsigned char REG_ZOUTL = 0x04; // 10 bits output value Z LSB
const unsigned char REG_ZOUTH = 0x05; // 10 bits output value Z MSB

const unsigned char REG_WHOAMI = 0x0F;

const unsigned char REG_XOUT8 = 0x06; // X acceleration data (8-Bit)
const unsigned char REG_YOUT8 = 0x07; // Y acceleration data (8-Bit)
const unsigned char REG_ZOUT8 = 0x08; // Z acceleration data (8-Bit)

const unsigned char REG_STATUS = 0x09; // Status
const unsigned char REG_DETSRC = 0x0A; // Detection Source Register

const unsigned char REG_XOFFL = 0x10; // X LOW offset setting register
const unsigned char REG_XOFFH = 0x11; // X HIGH offset setting register
const unsigned char REG_YOFFL = 0x12; // Y LOW offset setting register
const unsigned char REG_YOFFH = 0x13; // Y HIGH offset setting register
const unsigned char REG_ZOFFL = 0x14; // Z LOW offset setting register
const unsigned char REG_ZOFFH = 0x15; // Z HIGH offset setting register

const unsigned char REG_MCTL = 0x16; // Mode Control
const unsigned char REG_INTRST = 0x17; // Interrupt Reset

const unsigned char REG_CTL1 = 0x18; // Control 1
const unsigned char REG_CTL2 = 0x19; // Control 2
const unsigned char REG_LDTH = 0x1A; // Level detection threshold limit value
const unsigned char REG_PDTH = 0x1B; // Pulse detection threshold limit value
const unsigned char REG_PW = 0x1C; // Pulse Duration Value
const unsigned char REG_LT = 0x1D; // Latency Duration Value
const unsigned char REG_TW = 0x1E; // Time window for 2nd pulse value

//////////////
// Features
// G-Select
const unsigned char FEATURE_G_SELECT_2G = 0x04; // 2G
const unsigned char FEATURE_G_SELECT_4G = 0x08; // 4G
const unsigned char FEATURE_G_SELECT_8G = 0x00; // 8G
// MODE
const unsigned char FEATURE_MODE_STANDBY = 0x00; // Standby Mode
const unsigned char FEATURE_MODE_MEASUREMENT = 0x01; // Measurement Mode
const unsigned char FEATURE_MODE_LEVEL_DETECTION = 0x02; // Level Detection Mode
const unsigned char FEATURE_MODE_PULSE_DETECTION = 0x03; // Pulse Detection Mode
// STON
const unsigned char FEATURE_STON_ENABLED = 0x10; // Self Test Enabled
const unsigned char FEATURE_STON_DISABLED = 0x00; // Self Test Disabled
// SPI3W
const unsigned char FEATURE_SPI3W = 0x20; // SPI 3 Wire
const unsigned char FEATURE_SPI4W = 0x00; // SPI 4 Wire
// DRPD
const unsigned char FEATURE_DRPD_INT1_ENABLED = 0x00; // Data ready status is output to INT1/DRDY PIN
const unsigned char FEATURE_DRPD_INT1_DISABLED = 0x40; // Data ready status is not output to INT1/DRDY PIN

//////////////
// STATUS
const unsigned char STATUS_DRDY = 0x01; // Data Ready
const unsigned char STATUS_DOVR = 0x02; // Data Over Written
const unsigned char STATUS_PERR = 0x04; // Parity Error

//////////////
// STATUS
const unsigned char CLR_INT1 = 0x01; // Clear Interrupt 1
const unsigned char CLR_INT2 = 0x02; // Clear Interrupt 2

//////////////
// CONTROL 1
const unsigned char CTRL1_INTPIN_INT1 = 0x00; // INT1 pin is routed to INT1 bit in Detection Source Register ($0A) and INT2 pin is routed to INT2 bit in Detection Source Register ($0A).
const unsigned char CTRL1_INTPIN_INT2 = 0x01; // INT2 pin is routed to INT1 bit in Detection Source Register ($0A) and INT1 pin is routed to INT2 bit in Detection Source Register ($0A).
const unsigned char CTRL1_INTREG1 = 0x00; // 00: INT1 Register is detecting Level while INT2 is detecting Pulse.
const unsigned char CTRL1_INTREG2 = 0x02; // 01: INT1 Register is detecting Pulse while INT2 is detecting Level.
const unsigned char CTRL1_INTREG3 = 0x04; // 10: INT1 Register is detecting a Single Pulse and INT2 is detecting Single Pulse (if 2nd Time Window = 0) or if there is a latency
// time window and second time window > 0 then INT2 will detect the double pulse only.
const unsigned char CTRL1_XDA_ENABLED = 0x00; // X-axis is enabled for detection.
const unsigned char CTRL1_XDA_DISABLE = 0x08; // X-axis is disabled for detection.
const unsigned char CTRL1_YDA_ENABLED = 0x00; // Y-axis is enabled for detection.
const unsigned char CTRL1_YDA_DISABLE = 0x10; // Y-axis is disabled for detection.
const unsigned char CTRL1_ZDA_ENABLED = 0x00; // Z-axis is enabled for detection.
const unsigned char CTRL1_ZDA_DISABLE = 0x20; // Z-axis is disabled for detection.
const unsigned char CTRL1_THOPT_ABSOLUTE = 0x00; // Threshold value is absolute only (This bit is valid for level detection only, not valid for pulse detection)
const unsigned char CTRL1_THOPT_INTEGER = 0x40; // Integer value is available. (This bit is valid for level detection only, not valid for pulse detection)
const unsigned char CTRL1_TFBW_LOW = 0x00; // Digital filter band width is 62.5 Hz
const unsigned char CTRL1_TFBW_HIGH = 0x80; // Digital filter band width is 125 Hz

//////////////
// CONTROL 2
const unsigned char CTRL2_LDPL_OR = 0x00; // Level detection polarity is positive and detecting condition is OR 3 axes.
const unsigned char CTRL2_LDPL_AND = 0x01; // Level detection polarity is negative detecting condition is AND 3 axes.
const unsigned char CTRL2_PDPL_OR = 0x00; // Pulse detection polarity is positive and detecting condition is OR 3 axes.
const unsigned char CTRL2_PDPL_AND = 0x04; // Pulse detection polarity is negative and detecting condition is AND 3 axes.
const unsigned char CTRL2_DRVO_STANDARD = 0x00; // Standard drive strength on SDA/SDO pin
const unsigned char CTRL2_DRVO_STRONG = 0x04; // Strong drive strength on SDA/SDO pin


// Used to read 10 Bit Values
union UAxis
{
  // register values
  struct
  {
    byte xLSB; // x-Axis lower 8 Bit
    byte xMSB; // x-Axis higher 2 Bit + sign
    byte yLSB; // y-Axis lower 8 Bit
    byte yMSB; // y-Axis higher 2 Bit + sign
    byte zLSB; // z-Axis lower 8 Bit
    byte zMSB; // z-Axis higher 2 Bit + sign
  } reg;
  // 10 Bits Axis Values
  struct
  {
    int x; // x-Axis 10 Bit
    int y; // y-Axis 10 Bit
    int z; // z-Axis 10 Bit
  } values;
};

/* receive values from MMA7455 */
byte readSensor( byte regAddress )
{
  byte retValue = 0x00;
  digitalWrite( SLAVE_SELECT_PIN, LOW );
  byte command = SPI_READ | ( regAddress << 1 );
  SPI.transfer( command );
  retValue = SPI.transfer( 0x00 );
  digitalWrite( SLAVE_SELECT_PIN, HIGH );
  return retValue;
}

/* send commands to the MMA7455 */
void writeSensor( byte regAddress, byte value )
{
  digitalWrite( SLAVE_SELECT_PIN, LOW );
  byte command = SPI_WRITE | ( regAddress << 1 );
  SPI.transfer( command );
  SPI.transfer( value );
  digitalWrite( SLAVE_SELECT_PIN, HIGH );
  delay( 100 );
}

void clearInterrupts()
{
  writeSensor( REG_INTRST, CLR_INT1 | CLR_INT2 );
  writeSensor( REG_INTRST, 0x00 );
}

/* read 10 bit acceleration values */
void readAcceleration( int & x, int & y, int & z )
{
  UAxis axis;

  axis.reg.xLSB = readSensor( REG_XOUTL );
  axis.reg.xMSB = readSensor( REG_XOUTH );
  axis.reg.yLSB = readSensor( REG_YOUTL );
  axis.reg.yMSB = readSensor( REG_YOUTH );
  axis.reg.zLSB = readSensor( REG_ZOUTL );
  axis.reg.zMSB = readSensor( REG_ZOUTH );

  // bit 9 is sign

  if ( axis.reg.xMSB & 0x02 )
    axis.reg.xMSB |= 0xFC;

  if ( axis.reg.yMSB & 0x02 )
    axis.reg.yMSB |= 0xFC;

  if ( axis.reg.zMSB & 0x02 )
    axis.reg.zMSB |= 0xFC;

  x = axis.values.x;
  y = axis.values.y;
  z = axis.values.z;

  ledState = true;

}

void readAcceleration()
{
  int x;
  int y;
  int z;
  readAcceleration( x, y, z );

  String string;
  string.concat( x );
  string.concat( ", " );
  string.concat( y );
  string.concat( ", " );
  string.concat( z );
  string.concat( "\n" );
  Serial.print( string );
  
  int len = string.length();
  char data[30];
  string.toCharArray(data, 30);
  Serial.write(data);
}

void calibrateFlat()
{
  byte status;

  for ( int i = 0; i < 3; i++ )
  {
    // wait till data is ready
    do
    {
      status = readSensor( REG_STATUS );
    } while ( !( status & STATUS_DRDY ) && !( status & STATUS_PERR ) );

    UAxis calibration;
    UAxis currentCalibration;

    // read stored calibration values
    currentCalibration.reg.xLSB = readSensor( REG_XOFFL );
    currentCalibration.reg.xMSB = readSensor( REG_XOFFH );
    currentCalibration.reg.yLSB = readSensor( REG_YOFFL );
    currentCalibration.reg.yMSB = readSensor( REG_YOFFH );
    currentCalibration.reg.zLSB = readSensor( REG_ZOFFL );
    currentCalibration.reg.zMSB = readSensor( REG_ZOFFH );

    // read values for calibration
    readAcceleration( calibration.values.x, calibration.values.y,
                      calibration.values.z );

    calibration.values.x = currentCalibration.values.x
                           + 2 * ( -calibration.values.x );
    calibration.values.y = currentCalibration.values.y
                           + 2 * ( -calibration.values.y );

    // Calibration for z = 1G
     calibration.values.z = currentCalibration.values.z + 2 * (-(calibration.values.z + (calibration.values.z >= 0 ? -64 : 64)));

    // Calibration for z = 0G
    //calibration.values.z = currentCalibration.values.z
    //                       + 2 * -( calibration.values.z );

    // Set new calibration values
    writeSensor( REG_XOFFL, calibration.reg.xLSB );
    writeSensor( REG_XOFFH, calibration.reg.xMSB );
    writeSensor( REG_YOFFL, calibration.reg.yLSB );
    writeSensor( REG_YOFFH, calibration.reg.yMSB );
    writeSensor( REG_ZOFFL, calibration.reg.zLSB );
    writeSensor( REG_ZOFFH, calibration.reg.zMSB );

    delay( 100 );
  }
}

void interruptHandler()
{
  ledState = true;
  clearInterrupts();
}

void blink()
{
  digitalWrite( LED_PIN, HIGH ); // turn the LED on (HIGH is the voltage level)
  delay( 50 );                 // wait some time
  digitalWrite( LED_PIN, LOW );   // turn the LED off by making the voltage LOW
}

//The setup function is called once at startup of the sketch
void setup()
{
  // set up SPI
  pinMode( SLAVE_SELECT_PIN, OUTPUT );
  SPI.begin();
  SPI.setBitOrder( MSBFIRST );
  SPI.setDataMode( SPI_MODE0 );
  SPI.setClockDivider( SPI_CLOCK_DIV128 );

  // initialize serial communication at 9600 bits per second:
  Serial.begin( 9600 );

  // set mode pulse detection
  //writeSensor( REG_MCTL,
  //             FEATURE_G_SELECT_8G
  //             | FEATURE_MODE_PULSE_DETECTION
  //             | FEATURE_DRPD_INT1_DISABLED );

  // set mode level detection
   writeSensor( REG_MCTL,
        FEATURE_G_SELECT_8G
        | FEATURE_MODE_LEVEL_DETECTION
        | FEATURE_DRPD_INT1_DISABLED );

  // calibrate sensor
  calibrateFlat();
  clearInterrupts();

  // Set Level threshold
  // Set Threshold to +/-0.5g, which is 7 counts (16 counts/g).
  writeSensor( REG_LDTH, 0x16 );

  // Set Pulse threshold
  // Set Threshold to +/-0.5g, which is 7 counts (16 counts/g).
  writeSensor( REG_PDTH, 0x16 );

  // set Pulse duration
  // Min: PD[7:0] = 4’h01 = 0.5 ms
  // Max: PD[7:0] = 4’hFF = 127 ms
  // 1 LSB = 0.5 ms
  writeSensor( REG_PW, 0xFF );

  writeSensor( REG_CTL1, CTRL1_INTREG1 );

  // assign interrupt
  // Interrupt Level Detection
  // attachInterrupt(BOARD_INT0, interruptHandler, RISING);

  // Interrupt Pulse Detection
  attachInterrupt( BOARD_INT1, interruptHandler, RISING );
}

// The loop function is called in an endless loop
void loop()
{

  // check first status register if data is available
  byte status = readSensor(REG_STATUS);

  if ( status & STATUS_DRDY ) {
    readAcceleration();
    //delay(10);
  } else {
    delay(10);
  }

  //digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(50);               // wait for a second
  //digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW

  //delay(100);

  if ( ledState )
  {
    blink();
    ledState = false;
  }
}

