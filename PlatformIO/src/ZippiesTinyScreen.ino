
#include <Wire.h>
#include "commands/Executor.h"
#include "commands/LinearMove.h"
#include "commands/LinearTurn.h"
#include "commands/PauseMove.h"
#include "commands/CubicBezierMove.h"
#include "lighthouse/KPosition.h"
#include "lighthouse/KVector2.h"
#include "Zippy.h"
#include "Bluetooth.h"
#include "ZippyConfig.h"

//the number of milliseconds for each "beat" of the song we are building our movement routine against
#define TEMPO_MS_PER_BEAT                       600.0d
#define ZIPPY_OFFSET                            100.0d

#define BLE_RECEIVE_MOTORS_ALL_STOP  0x00
#define BLE_RECEIVE_MOTORS_SET       0x15
#define BLE_RECEIVE_FORWARD_STRAIGHT 0x16
#define BLE_SEND_DEBUG_INFO          0x00
#define BLE_AUTODRIVE_MODE           0x20
#define BLE_MANUAL_MODE              0x21
#define BLE_SEND_INTERVAL_MS          300

#define LOOP_INDICATOR_INTERVAL 5000

#define M_PI_34 2.356194490192345d

ZippyMove** moves = NULL;
Executor* executor = NULL;

#ifdef ENABLE_BLUETOOTH
Bluetooth bluetooth;
unsigned long bluetoothSendDebugInfoTmeStamp = 0;
#endif

void createExecutor();

void setup()
{
  Wire.begin();
  SerialUSB.begin(115200);
  // while (!SerialUSB);
  // SerialUSB.println("Started serial port.");

#ifdef ENABLE_BLUETOOTH
  bluetooth.start();
#endif

  createExecutor();
  executor->start(millis());
}

void loop()
{
  // SerialUSB.println("Looping");
  // uint64_t currentTime = millis();
  unsigned long currentTime = micros() / 1000;

#ifdef ENABLE_BLUETOOTH
  // processBluetoothInput();
#endif

  executor->loop(currentTime);

#ifdef ENABLE_BLUETOOTH
  processBluetoothOutput(currentTime);
#endif
}

void createExecutor()
{
  double beatHalf = 0.5d * TEMPO_MS_PER_BEAT;
  double beats1 = TEMPO_MS_PER_BEAT;
  double beats2 = 2.0d * TEMPO_MS_PER_BEAT;
  double beats3 = 3.0d * TEMPO_MS_PER_BEAT;
  double beats4 = 4.0d * TEMPO_MS_PER_BEAT;
  double beats7 = 7.0d * TEMPO_MS_PER_BEAT;
  double beats8 = 8.0d * TEMPO_MS_PER_BEAT;

  int nextMove = 0;

  // /*
  //test the ability for the Zippy to move through a variety of circlular paths defined by
  //bezier curves at various target speeds
  int moveCount = 27;
  // int moveCount = 8;
  moves = new ZippyMove*[moveCount];
  double offset = ZIPPY_ID == 0 ? ZIPPY_OFFSET :
      (ZIPPY_ID == 1 ? 0.0d : -ZIPPY_OFFSET);
  bool reversed = false;
  //large figure 8, fastest speed
  moves[nextMove++] = new CubicBezierMove( 460.0d+offset,  460.0d,  M_PI_2, reversed, beats2);
  moves[nextMove++] = new CubicBezierMove( 920.0d+offset,    0.0d,    M_PI, reversed, beats2);
  moves[nextMove++] = new CubicBezierMove( 460.0d+offset, -460.0d, -M_PI_2, reversed, beats2);
  moves[nextMove++] = new CubicBezierMove(   0.0d+offset,    0.0d,    0.0d, reversed, beats2);
  moves[nextMove++] = new CubicBezierMove(-460.0d+offset,  460.0d, -M_PI_2, reversed, beats2);
  moves[nextMove++] = new CubicBezierMove(-920.0d+offset,    0.0d, M_PI, reversed, beats2);
  moves[nextMove++] = new CubicBezierMove(-460.0d+offset, -460.0d, M_PI_2, reversed, beats2);
  moves[nextMove++] = new CubicBezierMove(   0.0d+offset,    0.0d, 0.0d, reversed, beats2);
  moves[nextMove++] = new PauseMove(beats4);
  //medium figure 8, fastest speed
  // moves[nextMove++] = new CubicBezierMove( 345.0d+offset,  345.0d, M_PI_2, reversed, beats2);
  // moves[nextMove++] = new CubicBezierMove( 690.0d+offset,    0.0d, M_PI, reversed, beats2);
  // moves[nextMove++] = new CubicBezierMove( 345.0d+offset, -345.0d, -M_PI_2, reversed, beats2);
  // moves[nextMove++] = new CubicBezierMove(   0.0d+offset,    0.0d, 0.0d, reversed, beats2);
  // moves[nextMove++] = new CubicBezierMove(-345.0d+offset,  345.0d, -M_PI_2, reversed, beats2);
  // moves[nextMove++] = new CubicBezierMove(-690.0d+offset,    0.0d, M_PI, reversed, beats2);
  // moves[nextMove++] = new CubicBezierMove(-345.0d+offset, -345.0d, M_PI_2, reversed, beats2);
  // moves[nextMove++] = new CubicBezierMove(   0.0d+offset,    0.0d, 0.0d, reversed, beats2);
  // moves[nextMove++] = new PauseMove(beats4);
  //smallest figure 8, fastest speed
  double beats = 400.0d;
  moves[nextMove++] = new CubicBezierMove( 100.0d+offset,  100.0d, M_PI_2, reversed, beats);
  moves[nextMove++] = new CubicBezierMove( 200.0d+offset,    0.0d, M_PI, reversed, beats);
  moves[nextMove++] = new CubicBezierMove( 100.0d+offset, -100.0d, -M_PI_2, reversed, beats);
  moves[nextMove++] = new CubicBezierMove(   0.0d+offset,    0.0d, 0.0d, reversed, beats);
  moves[nextMove++] = new CubicBezierMove(-100.0d+offset,  100.0d, -M_PI_2, reversed, beats);
  moves[nextMove++] = new CubicBezierMove(-200.0d+offset,    0.0d, M_PI, reversed, beats);
  moves[nextMove++] = new CubicBezierMove(-100.0d+offset, -100.0d, M_PI_2, reversed, beats);
  moves[nextMove++] = new CubicBezierMove(   0.0d+offset,    0.0d, 0.0d, reversed, beats);
  moves[nextMove++] = new PauseMove(beats4);
  //medium figure 8, slowest speed
  // moves[nextMove++] = new CubicBezierMove( 345.0d+offset,  345.0d, M_PI_2, reversed, beats4);
  // moves[nextMove++] = new CubicBezierMove( 690.0d+offset,    0.0d, M_PI, reversed, beats4);
  // moves[nextMove++] = new CubicBezierMove( 345.0d+offset, -345.0d, -M_PI_2, reversed, beats4);
  // moves[nextMove++] = new CubicBezierMove(   0.0d+offset,    0.0d, 0.0d, reversed, beats4);
  // moves[nextMove++] = new CubicBezierMove(-345.0d+offset,  345.0d, -M_PI_2, reversed, beats4);
  // moves[nextMove++] = new CubicBezierMove(-690.0d+offset,    0.0d, M_PI, reversed, beats4);
  // moves[nextMove++] = new CubicBezierMove(-345.0d+offset, -345.0d, M_PI_2, reversed, beats4);
  // moves[nextMove++] = new CubicBezierMove(   0.0d+offset,    0.0d, 0.0d, reversed, beats4);
  // moves[nextMove++] = new PauseMove(beats4);
  //small figure 8, slowest speed
  moves[nextMove++] = new CubicBezierMove( 100.0d+offset,  100.0d, M_PI_2, reversed, beats3);
  moves[nextMove++] = new CubicBezierMove( 200.0d+offset,    0.0d, M_PI, reversed, beats3);
  moves[nextMove++] = new CubicBezierMove( 100.0d+offset, -100.0d, -M_PI_2, reversed, beats3);
  moves[nextMove++] = new CubicBezierMove(   0.0d+offset,    0.0d, 0.0d, reversed, beats3);
  moves[nextMove++] = new CubicBezierMove(-100.0d+offset,  100.0d, -M_PI_2, reversed, beats3);
  moves[nextMove++] = new CubicBezierMove(-200.0d+offset,    0.0d, M_PI, reversed, beats3);
  moves[nextMove++] = new CubicBezierMove(-100.0d+offset, -100.0d, M_PI_2, reversed, beats3);
  moves[nextMove++] = new CubicBezierMove(   0.0d+offset,    0.0d, 0.0d, reversed, beats3);
  moves[nextMove++] = new PauseMove(beats4);
  executor = new Executor(0.0d+offset, 0.0d, reversed ? M_PI : 0.0d, moves, moveCount);
  // */

  /*
#if ZIPPY_ID == 0
  double P0 =  beats2;
  double X0 =  -50.0d;
  double X1 =   50.0d;
  double R1 = -M_PI_2;
  double P1 =  beats7;
  int moveCount = 29;
#else
  double P0 =  beats8;
  double X0 =   50.0d;
  double X1 =  -50.0d;
  double R1 =  M_PI_2;
  double P1 =  beats1;
  int moveCount = 43;
#endif

  //start of dance routine, to be completed when we have movement perfected
  moves = new ZippyMove*[moveCount];
  //nothing for first two beats of the song
  moves[nextMove++] = new PauseMove(P0);
  //slowly enter the sceen
  moves[nextMove++] = new LinearMove(X1, 0.0d, beats4);
  //slowly turn to face right
  moves[nextMove++] = new LinearTurn(R1, beats3);
  //pause for 6 beats
  moves[nextMove++] = new PauseMove(P1);

  //now the moves begin, startting with a slow shimmy
  //first measure; shimmy forward and right

  //first measure
  //snap to the front
  moves[nextMove++] = new LinearTurn(0.0d, beats1, false);
  moves[nextMove++] = new CubicBezierMove(X1+100.0d,  75.0d, 0.0d, beats1);
  moves[nextMove++] = new CubicBezierMove(X1       , 150.0d, 0.0d, beats1);
  moves[nextMove++] = new CubicBezierMove(X1+100.0d, 225.0d, 0.0d, beats1);
  // moves[nextMove++] = new CubicBezierMove(300.0d, 300.0d, 0.0d, beats1);
  // moves[nextMove++] = new CubicBezierMove(350.0d, 225.0d, M_PI, true, beats1);
  //second measure
  moves[nextMove++] = new PauseMove(beats1);
  moves[nextMove++] = new CubicBezierMove(X1       , 150.0d, M_PI, true, beats1);
  moves[nextMove++] = new CubicBezierMove(X1+100.0d,  75.0d, M_PI, true, beats1);
  moves[nextMove++] = new CubicBezierMove(X1       ,   0.0d, M_PI, true, beats1);
  //third measure
  moves[nextMove++] = new PauseMove(beats1);
  moves[nextMove++] = new CubicBezierMove(X1-100.0d,  75.0d, 0.0d, beats1);
  moves[nextMove++] = new CubicBezierMove(X1       , 150.0d, 0.0d, beats1);
  moves[nextMove++] = new CubicBezierMove(X1-100.0d, 225.0d, 0.0d, beats1);
  // moves[nextMove++] = new CubicBezierMove(300.0d, 300.0d, 0.0d, beats1);
  // moves[nextMove++] = new CubicBezierMove(250.0d, 225.0d, M_PI, true, beats1);
  //fourth measure
  moves[nextMove++] = new PauseMove(beats1);
  moves[nextMove++] = new CubicBezierMove(X1       , 150.0d, M_PI, true, beats1);
  moves[nextMove++] = new CubicBezierMove(X1-100.0d,  75.0d, M_PI, true, beats1);
  moves[nextMove++] = new CubicBezierMove(X1       ,   0.0d, M_PI, true, beats1);
  //20 moves to this point

  //main dance routine
#if ZIPPY_ID == 0
  moves[nextMove++] = new CubicBezierMove(X1- 75.0d, -75.0d, -M_PI_2, true, beatHalf);
  moves[nextMove++] = new CubicBezierMove(X1-150.0d,   0.0d,    0.0d, true, beatHalf);
  moves[nextMove++] = new CubicBezierMove(X1+ 75.0d,  75.0d,  M_PI_2, true, beatHalf);
  moves[nextMove++] = new CubicBezierMove(X1       ,   0.0d,    M_PI, true, beatHalf);
  moves[nextMove++] = new LinearMove(1000.0d, 0.0d, beats1);
  moves[nextMove++] = new PauseMove(beats3+beats4);
#else
  moves[nextMove++] = new CubicBezierMove(X1+ 75.0d,  75.0d,  M_PI_2, beatHalf);
  moves[nextMove++] = new CubicBezierMove(X1+150.0d,   0.0d,    M_PI, beatHalf);
  moves[nextMove++] = new CubicBezierMove(X1+ 75.0d, -75.0d, -M_PI_2, beatHalf);
  moves[nextMove++] = new CubicBezierMove(X1       ,   0.0d,    0.0d, beatHalf);

  double beatCount = beats4;
  moves[nextMove++] = new CubicBezierMove(X1+ 20.0d,  20.0d,  M_PI_4, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1       ,  40.0d, -M_PI_4, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1+ 20.0d,  60.0d,  M_PI_4, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1       ,  80.0d, -M_PI_4, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1+ 20.0d, 100.0d,  M_PI_4, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1       , 120.0d, -M_PI_4, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1+ 20.0d, 140.0d,  M_PI_4, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1       , 160.0d, -M_PI_4, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1+ 20.0d, 140.0d, -M_PI_34, true, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1       , 120.0d,  M_PI_34, true, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1+ 20.0d, 100.0d, -M_PI_34, true, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1       ,  80.0d,  M_PI_34, true, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1+ 20.0d,  60.0d, -M_PI_34, true, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1       ,  40.0d,  M_PI_34, true, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1+ 20.0d,  20.0d, -M_PI_34, true, beatCount);
  moves[nextMove++] = new CubicBezierMove(X1       ,   0.0d,  M_PI_34, true, beatCount);
#endif

  //reset
  moves[nextMove++] = new PauseMove(beats4);
  moves[nextMove++] = new LinearMove(X0, 500.0d, beats4);
  moves[nextMove++] = new LinearTurn(M_PI, beats1);
  executor = new Executor(X0, 500.0d, M_PI, moves, moveCount);
  // executor = new Executor(300.0d, 0.0d, 0.0d, moves, moveCount);
  // */

  /* move calibration
  int moveCount = 15;
  moves = new ZippyMove*[moveCount];
  unsigned long deltaTimeBeatCount =  4.0d * TEMPO_MS_PER_BEAT;
  moves[nextMove++] = new LinearMove(   0.0d,  200.0d, beats4);
  moves[nextMove++] = new LinearMove(   0.0d,    0.0d, beats4);
  moves[nextMove++] = new LinearMove(   0.0d, -200.0d, beats4);
  moves[nextMove++] = new LinearMove(   0.0d,    0.0d, beats4);
  moves[nextMove++] = new PauseMove(4.0d * TEMPO_MS_PER_BEAT);

  deltaTimeBeatCount =  2.0d * TEMPO_MS_PER_BEAT;
  moves[nextMove++] = new LinearMove(   0.0d,  200.0d, beats2);
  moves[nextMove++] = new LinearMove(   0.0d,    0.0d, beats2);
  moves[nextMove++] = new LinearMove(   0.0d, -200.0d, beats2);
  moves[nextMove++] = new LinearMove(   0.0d,    0.0d, beats2);
  moves[nextMove++] = new PauseMove(4.0d * TEMPO_MS_PER_BEAT);

  deltaTimeBeatCount =  TEMPO_MS_PER_BEAT;
  moves[nextMove++] = new LinearMove(   0.0d,  200.0d, TEMPO_MS_PER_BEAT);
  moves[nextMove++] = new LinearMove(   0.0d,    0.0d, TEMPO_MS_PER_BEAT);
  moves[nextMove++] = new LinearMove(   0.0d, -200.0d, TEMPO_MS_PER_BEAT);
  moves[nextMove++] = new LinearMove(   0.0d,    0.0d, TEMPO_MS_PER_BEAT);
  moves[nextMove++] = new PauseMove(4.0d * TEMPO_MS_PER_BEAT);
  executor = new Executor(0.0d, 0.0d, 0.0d, moves, moveCount);
  // */

  /* turn calibration
  int moveCount = 20;
  moves = new ZippyMove*[moveCount];
  moves[nextMove++] = new LinearTurn(-M_PI_2, beats2);
  moves[nextMove++] = new LinearTurn(M_PI, beats2);
  moves[nextMove++] = new LinearTurn(M_PI_2, beats2);
  moves[nextMove++] = new LinearTurn(0.0d, beats2);
  moves[nextMove++] = new PauseMove(beats4);
  moves[nextMove++] = new LinearTurn(-M_PI_2, TEMPO_MS_PER_BEAT);
  moves[nextMove++] = new LinearTurn(M_PI, TEMPO_MS_PER_BEAT);
  moves[nextMove++] = new LinearTurn(M_PI_2, TEMPO_MS_PER_BEAT);
  moves[nextMove++] = new LinearTurn(0.0d, TEMPO_MS_PER_BEAT);
  moves[nextMove++] = new PauseMove(beats4);
  moves[nextMove++] = new LinearTurn(-M_PI_2, beatHalf);
  moves[nextMove++] = new LinearTurn(M_PI, beatHalf);
  moves[nextMove++] = new LinearTurn(M_PI_2, beatHalf);
  moves[nextMove++] = new LinearTurn(0.0d, beatHalf);
  moves[nextMove++] = new PauseMove(beats4);
  moves[nextMove++] = new LinearTurn(-M_PI_2, beatHalf, false);
  moves[nextMove++] = new LinearTurn(M_PI, beatHalf, false);
  moves[nextMove++] = new LinearTurn(M_PI_2, beatHalf, false);
  moves[nextMove++] = new LinearTurn(0.0d, beatHalf, false);
  moves[nextMove++] = new PauseMove(beats4);
  executor = new Executor(0.0d, 0.0d, 0.0d, moves, moveCount);
  // */

  /*
  //test the ability for the Zippy to start a bezier while facing the opposite direction
  //from the first bezier point
  int moveCount = 12;
  moves = new ZippyMove*[moveCount];
  moves[nextMove++] = new LinearTurn(M_PI, beats4, false);
  moves[nextMove++] = new CubicBezierMove( 400.0d,  400.0d, M_PI_2, beats3);
  moves[nextMove++] = new CubicBezierMove( 800.0d,    0.0d, M_PI, beats3);
  moves[nextMove++] = new CubicBezierMove( 400.0d, -400.0d, -M_PI_2, beats3);
  moves[nextMove++] = new CubicBezierMove(   0.0d,    0.0d, 0.0d, beats3);
  moves[nextMove++] = new PauseMove(beats4);
  moves[nextMove++] = new LinearTurn(M_PI, beats4, false);
  moves[nextMove++] = new CubicBezierMove(-400.0d,  400.0d, -M_PI_2, beats3);
  moves[nextMove++] = new CubicBezierMove(-800.0d,    0.0d, M_PI, beats3);
  moves[nextMove++] = new CubicBezierMove(-400.0d, -400.0d, M_PI_2, beats3);
  moves[nextMove++] = new CubicBezierMove(   0.0d,    0.0d, 0.0d, beats3);
  moves[nextMove++] = new PauseMove(beats4);
  executor = new Executor(0.0d, 0.0d, M_PI, moves, moveCount);
  // */

  /*
  int moveCount = 8;
  moves = new ZippyMove*[moveCount];
  moves[nextMove++] = new CubicBezierMove(350.0d,  75.0d, 0.0d, beats2);
  moves[nextMove++] = new CubicBezierMove(300.0d, 150.0d, 0.0d, beats2);
  moves[nextMove++] = new CubicBezierMove(350.0d, 225.0d, 0.0d, beats2);
  moves[nextMove++] = new CubicBezierMove(300.0d, 300.0d, 0.0d, beats2);
  moves[nextMove++] = new CubicBezierMove(250.0d, 225.0d, 0.0d, true, beats2);
  moves[nextMove++] = new CubicBezierMove(300.0d, 150.0d, 0.0d, true, beats2);
  moves[nextMove++] = new CubicBezierMove(250.0d,  75.0d, 0.0d, true, beats2);
  moves[nextMove++] = new CubicBezierMove(300.0d,   0.0d, 0.0d, true, beats2);
  executor = new Executor(300.0d, 0.0d, 0.0d, moves, moveCount);
  // */

  /*
  //test shimmy forward and backward
  int moveCount = 10;
  moves = new ZippyMove*[moveCount];
  moves[nextMove++] = new CubicBezierMove(400.0d,  75.0d, 0.0d, beats1);
  moves[nextMove++] = new CubicBezierMove(300.0d, 150.0d, 0.0d, beats1);
  moves[nextMove++] = new CubicBezierMove(400.0d, 225.0d, 0.0d, beats1);
  moves[nextMove++] = new CubicBezierMove(300.0d, 300.0d, 0.0d, beats1);
  moves[nextMove++] = new CubicBezierMove(200.0d, 225.0d, 0.0d, true, beats1);
  moves[nextMove++] = new CubicBezierMove(300.0d, 150.0d, 0.0d, true, beats1);
  moves[nextMove++] = new CubicBezierMove(200.0d,  75.0d, 0.0d, true, beats1);
  moves[nextMove++] = new CubicBezierMove(300.0d,   0.0d, 0.0d, true, beats1);
  moves[nextMove++] = new LinearMove(300.0d, 0.0d, beats2, true);
  moves[nextMove++] = new LinearTurn(0.0d, beats2);
  executor = new Executor(300.0d, 0.0d, 0.0d, moves, moveCount);
  // */
}

#ifdef ENABLE_BLUETOOTH
/*
void processBluetoothInput()
{
  //now process all the inbound Bluetooth commands
  uint8_t receivedDataLength = bluetooth.loop();
  while (receivedDataLength) {
    uint8_t* receivedData = bluetooth.getReceivedData();
    for (int i = 0; i < receivedDataLength; i++) {
      switch (receivedData[i]) {
        case BLE_AUTODRIVE_MODE:
          if (autoDriver == NULL)
            autoDriver = new AutoDriveMode(&zippy);
          // autoDriver->start()
          zippy.stop();
          break;

        case BLE_MANUAL_MODE:
          if (autoDriver != NULL) {
            delete autoDriver;
            autoDriver = NULL;
          }
          zippy.stop();
          break;

        case BLE_RECEIVE_FORWARD_STRAIGHT:
          break;

        case BLE_RECEIVE_MOTORS_SET:
          //this command has a payload that should be 8 bytes (two signed floats)
          if (receivedDataLength-i >= 8) {
            i++;
            float motorLeft;
            memcpy(&motorLeft, receivedData+i, 4);
            i += 4;
            float motorRight;
            memcpy(&motorRight, receivedData+i, 4);
            i += 3;

            zippy.setMotors(motorLeft, motorRight);
          }
          break;

        case BLE_RECEIVE_MOTORS_ALL_STOP:
          zippy.stop();
          break;
      }
    }

    //until we've emptied out the bluetooth queue
    receivedDataLength = bluetooth.loop();
  }
}
// */

/*
void extractSensorPacket(LighthouseSensor* sensor, uint8_t* debugPacket)
{
  //X sync ticks (2 bytes)
  unsigned short nextShortValue = 0;//sensor->getXSyncTickCount();
  memcpy(debugPacket, &nextShortValue, sizeof(unsigned short));
  int packetPosition = sizeof(unsigned short);

  //X sweep ticks (4 bytes)
  unsigned int nextIntValue = sensor->getXSweepTickCount();
  memcpy(debugPacket+packetPosition, &nextIntValue, sizeof(unsigned int));
  packetPosition += sizeof(unsigned int);

  //calculated X position (4 bytes)
  KVector2* currentSensorPosition = sensor->getPosition();
  float nextFloatValue = currentSensorPosition->getX();
  memcpy(debugPacket+packetPosition, &nextFloatValue, sizeof(float));
  packetPosition += sizeof(float);

  //Y sync ticks (2 bytes)
  nextShortValue = 0;//sensor->getYSyncTickCount();
  memcpy(debugPacket+packetPosition, &nextShortValue, sizeof(unsigned short));
  packetPosition += sizeof(unsigned short);

  //Y sweep ticks (4 bytes)
  nextIntValue = sensor->getYSweepTickCount();
  memcpy(debugPacket+packetPosition, &nextIntValue, sizeof(unsigned int));
  packetPosition += sizeof(unsigned int);

  //calculated Y position (4 bytes)
  nextFloatValue = currentSensorPosition->getY();
  memcpy(debugPacket+packetPosition, &nextFloatValue, sizeof(float));
}
// */

void processBluetoothOutput(unsigned long currentTime)
{
  //check to see if we need to send debug info over Bluetooth
  if (currentTime - bluetoothSendDebugInfoTmeStamp < BLE_SEND_INTERVAL_MS)
    return;

  // SerialUSB.println("Sent updated data.");
  const Lighthouse* lighthouse = executor->getLighthouse();
  const KPosition* position = lighthouse->getPosition();
  const KPosition* positionDelta = lighthouse->getPositionDelta();
  bluetooth.sendBroadcastData(position->vector.getX(), position->vector.getY(),
      position->orientation, positionDelta->vector.getD());
  /*
  float deltaTimeSeconds = ((float)(currentTime - bluetoothSendDebugInfoTmeStamp)) / 1000.0f;

  //send the sync tick count, sweep tick count, X and Y of each diode sensor
  uint8_t debugPacket[SENSOR_DATA_LENGTH];

  //left sensor data
  LighthouseSensor* sensorLeft = lighthouse->getLeftSensor();
  extractSensorPacket(sensorLeft, debugPacket);
  bluetooth.sendSensorLeft(debugPacket);

  //right sensor data
  LighthouseSensor* sensorRight = lighthouse->getRightSensor();
  extractSensorPacket(sensorRight, debugPacket);
  bluetooth.sendSensorRight(debugPacket);

  //computed data
  static float previousOrientation = 0.0f;

  KVector2* currentPosition = lighthouse->getPosition();

  float floatValue = currentPosition->getX();
  memcpy(debugPacket, &floatValue, sizeof(float));
  floatValue = currentPosition->getY();
  memcpy(debugPacket+4, &floatValue, sizeof(float));
  floatValue = (sensorLeft->getVelocity()->getD() + sensorRight->getVelocity()->getD()) / 2.0f;
  memcpy(debugPacket+8, &floatValue, sizeof(float));

  float orientation = lighthouse->getOrientation()->getOrientation();
  memcpy(debugPacket+12, &orientation, sizeof(float));
  float rotationalVelocityRadians = (orientation - previousOrientation) / deltaTimeSeconds;
  memcpy(debugPacket+16, &rotationalVelocityRadians, sizeof(float));
  previousOrientation = orientation;
  bluetooth.sendComputedData(debugPacket);
  */

  bluetoothSendDebugInfoTmeStamp = currentTime;
}

#endif
