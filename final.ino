// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>

#include <Servo.h>
#include <Wire.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// define the pins used
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define BUTTON_PIN 5
#define SERVO1_PIN 9
#define SERVO2_PIN 11

// These are the pins used for the music maker shield
#define SHIELD_CS     7      // VS1053 chip select pin (output)
#define SHIELD_DCS    6      // VS1053 Data/command select pin (output)
#define CARDCS 4     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053_FilePlayer musicPlayer =
  Adafruit_VS1053_FilePlayer(-1, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);

MPU6050 mpu;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

// orientation/motion vars
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

Servo myservo1;
Servo myservo2;

byte houses[4] = {0, 0, 0, 0};

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void setup() {
  Wire.begin();
  TWBR = 12;
  Serial.begin(115200);
  randomSeed(analogRead(5)); // randomize so random works properly

  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();
  
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed code = "));
    Serial.print(devStatus);
  }

  pinMode(BUTTON_PIN, INPUT);
  myservo1.attach(SERVO1_PIN);
  myservo2.attach(SERVO2_PIN);
  

  if (!musicPlayer.begin()) { // initialise the music player
    Serial.println(F("Couldn't find VS1053"));
    while (1);
  }
  
  SD.begin(CARDCS);    // initialise the SD card
  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(5,5);
}

void loop() {
  if (digitalRead(BUTTON_PIN) == HIGH) {
    Serial.println(F("start the test"));
    playTest();
  }
  delay(1000);
}

void playTest() {
  const byte n = 12;
  byte questions[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

  controlMouth(0,0,0);

  musicPlayer.playFullFile("intro003.mp3");
  delay(1000); // give 'em time to process the instructions
  
  // shuffle the questions
  for (size_t i = 0; i < n - 1; i++) {
    size_t j = random(0, n-i);
    byte t = questions[j];
    questions[j] = questions[i];
    questions[i] = t;
  }

  // reset values
  for (size_t i = 0; i < 4; i++)
    houses[i] = 0;

  byte leader = 5;

  for (size_t i = 0; i < n; i++) {
    playQuestion(questions[i]);
  
    leader = findHouseLeader(false);
    if (leader != 5)
      break;
    else
      playMP3("sound", random(0, 5), true);
  }
    
  playMP3("sound", random(7, 8), true);
  
  
  if (leader != 5)
    playMP3("house", leader, false);
  else
    playMP3("house", findHouseLeader(true), false);
  controlMouth(80,40,700);
  controlMouth(80,40,600);
  controlMouth(80,0,700);
}

byte findHouseLeader(bool goodEnough) {
  byte index = 0;
  byte secondHighestIndex = 0;
  byte highestValue = houses[index];
  byte secondHighestValue = houses[index];
  
  for (byte i = 1; i < 4; i++) {
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
    if (random(0, 2) == 0)
      return index;
    else
      return secondHighestIndex;
  } else
    return 5;
}

void controlMouth(int num1, int num2, int waitTime) {
  myservo1.write(num1);
  myservo2.write(num1);
  delay(waitTime);
  myservo1.write(num2);
  myservo2.write(num2);
}

void playMP3(char *fileType, byte number, bool full) {
  char fileName[13];
  sprintf_P(fileName, PSTR("%s%03d.mp3"), fileType, number+1);
  if (full = true)
    musicPlayer.playFullFile(fileName);
  else
    musicPlayer.startPlayingFile(fileName);
}

void playQuestion(byte questionNumber) {
  char fileName[13];
  bool yesResponses[12][4] = {{1, 1, 0, 0}, {1, 0, 0, 1}, {1, 0, 0, 1}, {1, 1, 0, 0}, {1, 0, 0, 1}, {1, 0, 1, 0}, {0, 1, 0, 1}, {1, 0, 1, 0}, {0, 1, 0, 1}, {0, 1, 0, 1}, {0, 1, 1, 0}, {1, 0, 0, 1}};
  bool noResponses[12][4] = {{0, 0, 1, 1}, {0, 1, 1, 0}, {0, 1, 1, 0}, {0, 0, 1, 1}, {0, 1, 1, 0}, {0, 1, 0, 1}, {1, 0, 1, 0}, {0, 1, 0, 1}, {1, 0, 1, 0}, {1, 0, 1, 0}, {1, 0, 0, 1}, {0, 1, 1, 0}};

  playMP3("quest", questionNumber, true);

  bool response = getResponse();

  for (byte i = 0; i < 4; i++) {
    if (response == 1)
      houses[i] = houses[i] + yesResponses[questionNumber][i];
    else
      houses[i] = houses[i] + noResponses[questionNumber][i];
  }
}

bool getResponse() {
  long startTime;
  
  mpu.resetFIFO();
  
  while (readAG()==false);
  
  for (int i = 0; i < 2; i++) {
    if (i > 0)
      musicPlayer.startPlayingFile("intro005.mp3"); // tell them to nod harder

    float topPitch = ypr[1];
    float bottomPitch = ypr[1];
    float topYaw = ypr[0];
    float bottomYaw = ypr[0];
    
    startTime = millis();
    while (millis()-startTime < 15000) { // wait 15s before asking
      while (readAG()==false);
          
      if (ypr[1] > topPitch)
        topPitch = ypr[1];
      else if (ypr[1] < bottomPitch)
        bottomPitch = ypr[1];
        
      if (ypr[0] > topYaw)
        topYaw = ypr[0];
      else if (ypr[0] < bottomYaw)
        bottomYaw = ypr[0];
    
      if (abs(topYaw - bottomYaw) > 0.4) { // if shake
        Serial.println("no");
        return 0;
      } else if (abs(topPitch - bottomPitch) > 0.25) { // if nod
        Serial.println("yes");
        return 1;
      } 
    }
  }
  return random(0,2); // just quit and randomize
}

bool readAG() {
  long startTime = millis();

  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (millis()-startTime>3000)
      return false;
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount > packetSize) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return false;
  } else if (mpuIntStatus & 0x02) {
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    return true;
  }
}
