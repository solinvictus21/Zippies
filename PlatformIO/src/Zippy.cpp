
#include <SPI.h>
#include "Zippy.h"
#include "ZippyConfig.h"
#include "commands/Commands.h"
#include "PathData.h"

//power to each wheel is tuned as a combination of targets for rotational power and linear power; the linear power target defines the motion forward
//or backward we wish the robot to drive and will be the same for both wheels; the rotational power target defines the offest from linear power that
//we wish to apply to force the robot to turn and will be equal magnitude but opposite direction for each wheel

#define TEMPO_BPM                               100.0d
#define PATH_POINT_SCALE_FACTOR                1000.0d

//TUNING - COMMAND EXECUTION COMPLETION
//the distince within which is to be considered "at the target" for the purpose of terminating the current driving command; we use the square
//of the radius to eliminate the need for an additional square root
//sqrt(1600mm)/(10mm per cm) = 4cm
#define POSITION_EPSILON_2                     1600.0d
//5 degrees
#define ORIENTATION_EPSILON_2                     0.087266462599716d

#define ZIPPY_LOST_SIGNAL_TIMEOUT               200

//TUNING - PATH PLANNING
#define LOOP_INTERVAL_MS                        200

//TUNING - PID CONFIGURATION
#define LINEAR_Kp                                 5.00d
#define LINEAR_Ki                                 0.00d
#define LINEAR_Kd                                 0.50d
#define ROTATIONAL_Kp                             5.20d
#define ROTATIONAL_Ki                             0.00d
#define ROTATIONAL_Kd                             0.60d

#define LINEAR_MIN_VELOCITY                     100.00d
#define LINEAR_MAX_VELOCITY                    1000.00d
// #define LINEAR_AVG_VELOCITY                     600.00d
#define LINEAR_MIN_INPUT_CHANGE                  40.00d
#define LINEAR_MAX_INPUT_CHANGE_FACTOR            0.2d

//TUNING - PID INPUT
//80 degrees
#define LINEAR_RAMPUP_START                       1.396263401595464d
//50
#define LINEAR_RAMPUP_END                         0.872664625997165d
//60
#define ROTATIONAL_RAMPDOWN_START                 1.047197551196598d
#define ROTATIONAL_AVG_VELOCITY                 350.0d

//TUNING - PCM OUTPUT
//the minimum PCM value below which the motors do not turn
// #define MOTOR_MIN_THRESHOLD                    600.00d
#define MOTOR_MIN_POWER                        3400.00d
//the maximum PCM output the PID controllers are limited to produce out of an absolute max of 65535
#define MOTOR_MAX_POWER                       50000.00d

double snap(double motorPower, double zeroThreshold, double minimumMagnitude)
{
  if (abs(motorPower) < zeroThreshold)
    return 0.0d;

  if (motorPower > 0.0d)
    return minimumMagnitude + motorPower;
  else if (motorPower < 0.0d)
    return -minimumMagnitude + motorPower;

  return 0.0d;
}

double pad(double motorPower, double minimumMagnitude)
{
  if (motorPower > 0.0d)
    return minimumMagnitude + motorPower;
  else if (motorPower < 0.0d)
    return -minimumMagnitude + motorPower;

  return 0.0d;
}

Zippy::Zippy()
  : moving(false),
    linearPID(&linearInput, &linearOutput, &linearSetPoint, LINEAR_Kp, LINEAR_Ki, LINEAR_Kd, P_ON_E, DIRECT),
    rotationalPID(&rotationalInput, &rotationalOutput, &rotationalSetPoint, ROTATIONAL_Kp, ROTATIONAL_Ki, ROTATIONAL_Kd, P_ON_E, DIRECT),
    lostPositionTimestamp(0)
{
  linearPID.SetSampleTime(LOOP_INTERVAL_MS);
  linearPID.SetOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);
  rotationalPID.SetSampleTime(LOOP_INTERVAL_MS);
  rotationalPID.SetOutputLimits(-MOTOR_MAX_POWER, MOTOR_MAX_POWER);

  commands[0] = new MoveToPoint(0.0d, CENTER_OFFSET_Y, 5000);
  commands[1] = new TurnInPlace(0.0d, 0.0d, 1000);
  commands[2] = new Pause(3000);
  for (int i = 0; i < PATH_POINT_COUNT; i++) {
    double x = PATH_POINT_SCALE_FACTOR * PATH_POINTS[i].getX();
    double y = PATH_POINT_SCALE_FACTOR * PATH_POINTS[i].getY();
    PATH_POINTS[i].set(x, y + CENTER_OFFSET_Y);
  }
  commands[3] = new FollowPath(PATH_POINTS, PATH_POINT_COUNT, 15000);
  commands[4] = new Pause(3000);
}

//start all of our peripherals
void Zippy::start(unsigned long currentTime)
{
  //start the lighthouse
  lighthouse.start();

  // motors.start();
#ifdef PLATFORM_TINYSCREEN
  face.start();
  // SerialUSB.println("Started face.");

  motors.start();
  // SerialUSB.println("Started motors.");
#endif

  // /*
  linearPID.SetMode(MANUAL);
  rotationalPID.SetMode(MANUAL);

  //we need to reset the outputs before going back to automatic PID mode; see Arduino PID library source code for details
  linearSetPoint = 0.0d;
  linearInput = 0.0d;
  linearOutput = 0.0d;
  rotationalSetPoint = 0.0d;
  rotationalInput = 0.0d;
  rotationalOutput = 0.0d;

  linearPID.SetMode(AUTOMATIC);
  rotationalPID.SetMode(AUTOMATIC);

  lastUpdateTime = currentTime;
  // */
}

void Zippy::loop(unsigned long currentTime)
{
  // SerialUSB.println("Looping Zippy");
  //first process the Lighthouse input
  lighthouse.loop();

  // SerialUSB.println(currentTime);
  if (currentCommand >= ZIPPY_COMMAND_COUNT || currentTime - lastUpdateTime < LOOP_INTERVAL_MS)
    return;
  lastUpdateTime += LOOP_INTERVAL_MS;

  lighthouse.recalculate();
  if (!lighthouse.hasLighthouseSignal()) {
    if (!moving)
      return;

    //the sensors do not have the latest robot position; wait until they do; if we're currently in motion, setup a
    //timeout to stop moving if we go too long without the position information
    if (!lostPositionTimestamp)
      lostPositionTimestamp = currentTime;
    else if (currentTime - lostPositionTimestamp >= ZIPPY_LOST_SIGNAL_TIMEOUT) {
      // SerialUSB.println("stop moving");
      //we timed out waiting for an updated position; stop moving
      lostPositionTimestamp = 0;
      setMotors(0.0d, 0.0d);
      moving = false;
      // SerialUSB.println("Stopped moving.");
      return;
    }
  }
  else
    lostPositionTimestamp = 0;

  KVector2* currentPosition = lighthouse.getPosition();
  double currentOrientation = lighthouse.getOrientation()->getOrientation();
  if (!moving) {
    // SerialUSB.println("start moving");
    moving = true;
    //first setup the commands to move to the starting position
    ((MoveToPoint*)commands[0])->setStartingPosition(currentPosition->getX(), currentPosition->getY());
    ((TurnInPlace*)commands[1])->setStartingOrientation(atan2(-currentPosition->getX(), -currentPosition->getY()));

    // currentCommandStartPosition.set(currentPosition->getX(), (currentPosition->getY()));
    // currentCommandStartOrientation = currentOrientation;
    currentCommand = 0;
    currentCommandStartTime = currentTime;
    // SerialUSB.println("Started moving.");
  }

  unsigned long commandDeltaTime = currentTime - currentCommandStartTime;
  unsigned long nextExecutionTime = commands[currentCommand]->getExecutionTime();
  // SerialUSB.println(commandDeltaTime);
  while (commandDeltaTime > nextExecutionTime) {
    // SerialUSB.println("next command");
    //complete the current command
    commands[currentCommand]->getPosition(nextExecutionTime, &targetPosition);
    commands[currentCommand]->getOrientation(nextExecutionTime, &targetOrientation);
    commandDeltaTime -= nextExecutionTime;

    //start the next command
    currentCommand++;
    if (currentCommand == ZIPPY_COMMAND_COUNT)
      currentCommand = 2;
    currentCommandStartTime += nextExecutionTime;
    nextExecutionTime = commands[currentCommand]->getExecutionTime();
  }

  bool updatedTargetPosition = commands[currentCommand]->getPosition(commandDeltaTime, &targetPosition);
  KVector2 deltaPosition(targetPosition.getX() - currentPosition->getX(), targetPosition.getY() - currentPosition->getY());
  if (updatedTargetPosition)
    targetOrientation = deltaPosition.getOrientation();
  else
    commands[currentCommand]->getOrientation(commandDeltaTime, &targetOrientation);

  deltaPosition.rotate(-currentOrientation);
  calculateVelocityInput(&deltaPosition);

  double deltaOrientation = subtractAngles(targetOrientation, currentOrientation);
  rotationalInput = constrain(-deltaOrientation / ROTATIONAL_RAMPDOWN_START, -1.0d, 1.0d) *
      ROTATIONAL_AVG_VELOCITY;

  linearPID.Compute();
  rotationalPID.Compute();

  // setMotors(snap(-linearOutput-rotationalOutput, MOTOR_MIN_THRESHOLD, MOTOR_MIN_POWER),
      // snap(-linearOutput+rotationalOutput, MOTOR_MIN_THRESHOLD, MOTOR_MIN_POWER));
      // setMotors(pad(-linearOutput-rotationalOutput, MOTOR_MIN_POWER),
          // pad(-linearOutput+rotationalOutput, MOTOR_MIN_POWER));
  setMotors(pad(-rotationalOutput, MOTOR_MIN_POWER),
      pad(rotationalOutput, MOTOR_MIN_POWER));
  // setMotors(snap(-rotationalOutput, MOTOR_MIN_THRESHOLD, MOTOR_MIN_POWER),
      // snap(rotationalOutput, MOTOR_MIN_THRESHOLD, MOTOR_MIN_POWER));
}

void Zippy::calculateVelocityInput(KVector2* deltaPosition)
{
  double deltaOrientation = deltaPosition->getOrientation();
  double idealVelocity = (deltaPosition->getD() * 1000.0d) / ((double)LOOP_INTERVAL_MS);
  if (idealVelocity < LINEAR_MIN_VELOCITY)
    idealVelocity = 0.0d;

  if (deltaPosition->getY() < 0.0d) {
    idealVelocity = -idealVelocity;
    deltaOrientation = addAngles(deltaOrientation, M_PI);
  }
  double targetLinearVelocity = constrain(idealVelocity, -LINEAR_MAX_VELOCITY, LINEAR_MAX_VELOCITY) *
      (1.0d - constrain((abs(deltaOrientation) - LINEAR_RAMPUP_END) / (LINEAR_RAMPUP_START - LINEAR_RAMPUP_END), 0.0d, 1.0d));

  KVector2* currentVelocityVector = lighthouse.getVelocity();
  double currentVelocity = currentVelocityVector->getD();
  double inputDelta = currentVelocity - targetLinearVelocity - linearInput;
  // /*
  //limit the amount of acceleration to clamp down on PID overshoot; deceleration of any amount is fine
  if (targetLinearVelocity * linearInput > 0.0d && abs(targetLinearVelocity) > abs(linearInput)) {
    double maxInputChange = max(abs(linearInput) * LINEAR_MAX_INPUT_CHANGE_FACTOR, LINEAR_MIN_INPUT_CHANGE);
    if (currentVelocityVector->dotVector(lighthouse.getOrientation()) < 0.0d)
      currentVelocity = -currentVelocity;
    inputDelta = constrain(inputDelta, -maxInputChange, maxInputChange);
  }
  // */
  linearInput += inputDelta;
}

void Zippy::setMotors(int32_t motorLeft, int32_t motorRight)
{
  motors.writeCommand(COMMAND_ALL_PWM,
      motorLeft < 0 ? -motorLeft: 0,
      motorLeft > 0 ? motorLeft: 0,
      motorRight < 0 ? -motorRight: 0,
      motorRight > 0 ? motorRight: 0);
}

Zippy::~Zippy()
{
  for (int i = 0; i < ZIPPY_COMMAND_COUNT; i++)
    delete commands[i];
}
