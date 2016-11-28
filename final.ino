// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>
#include <Servo.h> 
#include <Wire.h>
#include "mpu6050.h"

// define the pins used
//#define CLK 13       // SPI Clock, shared with SD card
//#define MISO 12      // Input data, from VS1053/SD card
//#define MOSI 11      // Output data, to VS1053/SD card
#define BUTTON_PIN 1

// These are the pins used for the music maker shield
#define SHIELD_RESET  -1      // VS1053 reset pin (unused!)
#define SHIELD_CS     7      // VS1053 chip select pin (output)
#define SHIELD_DCS    6      // VS1053 Data/command select pin (output)
#define CARDCS 4     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

Servo myservo1;  
Servo myservo2;  
int questions[12] = {0,1,2,3,4,5,6,7,8,9,10,11};
const int yesResponses[12][4] = {{1,1,0,0},{1,0,0,1},{1,0,0,1},{1,1,0,0},{1,0,0,1},{1,0,1,0},{0,1,0,1},{1,0,1,0},{0,1,0,1},{0,1,0,1},{0,1,1,0},{1,0,0,1}};
const int noResponses[12][4] = {{0,0,1,1},{0,1,1,0},{0,1,1,0},{0,0,1,1},{0,1,1,0},{0,1,0,1},{1,0,1,0},{0,1,0,1},{1,0,1,0},{1,0,1,0},{1,0,0,1},{0,1,1,0}};
Adafruit_VS1053_FilePlayer musicPlayer = 
  Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);
int houses[4] = {0,0,0,0};
accel_t_gyro_union accel_t_gyro;

void setup() {
  int error;
  uint8_t c;

  Serial.begin(9600);
  Serial.println(F("sorting hat"));

  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();


  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //

  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  Serial.print(F("WHO_AM_I : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);

  // According to the datasheet, the 'sleep' bit
  // should read a '1'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. 
  error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
  Serial.print(F("PWR_MGMT_1 : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);


  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
  
  myservo1.attach(3);  
  myservo2.attach(5); 

  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while (1);
  }
  Serial.println(F("VS1053 found"));
  
  SD.begin(CARDCS);    // initialise the SD card
  
  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(20,20);
  
  // If DREQ is on an interrupt pin (on uno, #2 or #3) we can do background
  // audio playing
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);
}

void loop() {
  if (digitalRead(BUTTON_PIN) == HIGH) {
    playTest();
  }  
  
  int error;
  double dT;


  Serial.println(F(""));
  Serial.println(F("MPU-6050"));

  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  Serial.print(F("Read accel, temp and gyro, error = "));
  Serial.println(error,DEC);


  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);


  // Print the raw acceleration values

  Serial.print(F("accel x,y,z: "));
  Serial.print(accel_t_gyro.value.x_accel, DEC);
  Serial.print(F(", "));
  Serial.print(accel_t_gyro.value.y_accel, DEC);
  Serial.print(F(", "));
  Serial.print(accel_t_gyro.value.z_accel, DEC);
  Serial.println(F(""));

  // Print the raw gyro values.

  Serial.print(F("gyro x,y,z : "));
  Serial.print(accel_t_gyro.value.x_gyro, DEC);
  Serial.print(F(", "));
  Serial.print(accel_t_gyro.value.y_gyro, DEC);
  Serial.print(F(", "));
  Serial.print(accel_t_gyro.value.z_gyro, DEC);
  Serial.print(F(", "));
  Serial.println(F(""));

  delay(1000);
}

void playTest() {
  musicPlayer.playFullFile("preamble.mp3");
  shuffle(questions,12);
  intializeHouseValues();
  
  int leader;
  
  for (int i = 0; i < 12; i++) {
    playQuestion(i);
    leader = findHouseLeader(false);
    if (i == 11 || leader != 0)
      break;
    else
      playRandomNoise();
  }
  if (leader!= 5)
    playHouseAnnouncement(leader);
  else {
    playHouseAnnouncement(findHouseLeader(true));
  }
}

int intializeHouseValues() {
  for (int i = 0; i < 4; i++)
    houses[i] = 0;
}
  
int findHouseLeader(bool goodEnough) {
  int index = 0;
  int highestValue = houses[index];
  int secondHighestIndex = 5;
  int secondHighestValue = 0;
  for (int i = i; i < 4; i++) {
    if (houses[index] <= houses[i]) {
      secondHighestIndex = index;
      secondHighestValue = houses[index];
      index = i;
      highestValue = houses[i];
    }
  }
  if (secondHighestValue + 3 <= highestValue)
    return index;
  else if (goodEnough) {
    if (random(0,1) == 0)
      return index;
    else
      return secondHighestIndex;
  } else
    return 5;
}
void playHouseAnnouncement(int leader) {
  char fileName[13];
  sprintf_P(fileName, PSTR("house%02d.mp3"), leader);
  musicPlayer.startPlayingFile(fileName);
  controlMouth(90,500);
  controlMouth(0,100);
  controlMouth(90,500);
  controlMouth(0,100);
  controlMouth(90,500);
  controlMouth(0,0);
}

void controlMouth(int num, int delayTime) {
  myservo1.write(num);
  myservo2.write(num);
  delay(delayTime);
}

void playRandomNoise() {
  char fileName[13];
  sprintf_P(fileName, PSTR("random%01d.mp3"), random(0,8));
  musicPlayer.playFullFile(fileName);
}

void playQuestion(int questionNumber) {
  char fileName[13];
  sprintf_P(fileName, PSTR("ques%03d.mp3"), questionNumber);
  musicPlayer.playFullFile(fileName);
  
  int response = getResponse();
  
   if (response == 1) {
     for (int i = 0; i < 4; i++) {
       houses[i] = houses[i] + yesResponses[questionNumber][i];
     }
   } else if (response == 0) {
     for (int i = 0; i < 4; i++) {
       houses[i] = houses[i] + yesResponses[questionNumber][i];
     }
   }
}

int getResponse() {
  long startTime = millis();
  bool nod = (accel_t_gyro.value.x_accel == 0) &&
             (accel_t_gyro.value.y_accel == 0) &&
             (accel_t_gyro.value.z_accel == 0) &&
             (accel_t_gyro.value.x_gyro == 0) &&
             (accel_t_gyro.value.y_gyro == 0) &&
             (accel_t_gyro.value.z_gyro == 0);
  bool shake = (accel_t_gyro.value.x_accel == 0) &&
             (accel_t_gyro.value.y_accel == 0) &&
             (accel_t_gyro.value.z_accel == 0) &&
             (accel_t_gyro.value.x_gyro == 0) &&
             (accel_t_gyro.value.y_gyro == 0) &&
             (accel_t_gyro.value.z_gyro == 0);
  while (!nod && !shake) {
    if ((millis() - startTime) == (10 * 1000)) { // if it's been ten seconds
      musicPlayer.startPlayingFile("track002.mp3"); // tell them to nod harder
    }
  }
  musicPlayer.stopPlaying();
  if (nod) // if nod
    return 1;
  else if (shake) // if head shake
    return 0;
  else // try again?? shouldn't happen
    return getResponse();
}
void shuffle(int *array, size_t n) {
    if (n > 1) {
        size_t i;
        for (i = 0; i < n - 1; i++) {
          size_t j = i + rand() / (RAND_MAX / (n - i) + 1);
          int t = array[j];
          array[j] = array[i];
          array[i] = t;
        }
    }
}



// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}
