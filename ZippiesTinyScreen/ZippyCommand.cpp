
#include "ZippyCommand.h"
#include "Lighthouse.h"
#include "MotorDriver.h"

//the minimum PCM value below which the motors do not turn
#define MOTOR_MIN_POWER                      4600.00d
#define LINEAR_VELOCITY_POWER                6000.00d
#define LOOK_AHEAD_DISTANCE                   200.00d
#define HALF_SENSOR_SEPARATION 11.0d

//the radius squared (to prevent the need for an additional square root) when we are can consider the robot to be "at the target"
//currently set to 5cm, since sqrt(2500mm)/(10mm per cm) = 5cm
//#define AUTODRIVE_POSITION_EPSILON_2         2500.0d
#define AUTODRIVE_POSITION_EPSILON_2        10000.0d

#define AUTODRIVE_LINEAR_Kp                 200.0d
//#define AUTODRIVE_LINEAR_Ki                100.0d
//#define AUTODRIVE_LINEAR_Kd                200.0d
#define AUTODRIVE_LINEAR_Ki                  5++_>>>>_+>≥><<<asdasdawdwssasddwdw0.0d
#define AUTODRIVE_LINEAR_Kd                  4.0d

extern Lighthouse lighthouse;
extern MotorDriver motors;

Pause::Pause(double seconds)
  : deltaTimeMS(seconds * 1000.0d),
    startTimeMS(0)
{
}

void Pause::start()
{
  motors.setMotors(0, 0);
  startTimeMS = millis();
}

bool Pause::loop()
{
  return (millis() - startTimeMS) >= deltaTimeMS;
}

double padInner(double motorPower, double magnitude)
{
  if (motorPower > 0.0d)
    return magnitude + motorPower;
  else if (motorPower < 0.0d)
    return -magnitude + motorPower;

  return 0.0d;
}

MoveTowardPoint::MoveTowardPoint(double x, double y)
  : targetPosition(x, y),
    leftPID(&leftInput, &leftOutput, &leftSetPoint, AUTODRIVE_LINEAR_Kp, AUTODRIVE_LINEAR_Ki, AUTODRIVE_LINEAR_Kd, DIRECT),
    rightPID(&rightInput, &rightOutput, &rightSetPoint, AUTODRIVE_LINEAR_Kp, AUTODRIVE_LINEAR_Ki, AUTODRIVE_LINEAR_Kd, DIRECT)
{
  leftPID.SetOutputLimits(-LINEAR_VELOCITY_POWER, LINEAR_VELOCITY_POWER);
  rightPID.SetOutputLimits(-LINEAR_VELOCITY_POWER, LINEAR_VELOCITY_POWER);
}

double MoveTowardPoint::calculateInput(LighthouseSensor* sensor, KVector2* centerOfRobotToNextPosition)
{
  //step 1; calculate the vector from center of the robot to the sensor
  KVector2* robotCenterPosition = lighthouse.getPosition();
  KVector2* sensorPosition = sensor->getPosition();
  KVector2 sensorPositionFromCenter(sensorPosition->getX() - robotCenterPosition->getX(),
                                    sensorPosition->getY() - robotCenterPosition->getY());

  //step 2; calculate the vector from current sensor position to desired sensor position  
  //start with the relative sensor position
  KVector2 desiredSensorPosition(&sensorPositionFromCenter);

  //rotate that by the current orientation of the robot
  KVector2* robotOrientationVector = lighthouse.getOrientation();
  desiredSensorPosition.rotate(robotOrientationVector->angleToVector(centerOfRobotToNextPosition));

  //add the relative location of the next position from the center of the robot
  desiredSensorPosition.addVector(centerOfRobotToNextPosition);

  //subtract the relative sensor position from the center of the robot
  desiredSensorPosition.subtractVector(&sensorPositionFromCenter);

  double cos0 = cos(robotOrientationVector->angleToVector(&desiredSensorPosition));
  return -cos0 * desiredSensorPosition.getD();
}

void MoveTowardPoint::updateInputs(KVector2* deltaCenterToTarget)
{
  KVector2* robotCenterPosition = lighthouse.getPosition();

  KVector2 nextPosition(targetPosition.getX() - startingPosition.getX(),
                        targetPosition.getY() - startingPosition.getY());
  double maxD = nextPosition.getD();
  nextPosition.setD(maxD - min(deltaCenterToTarget->getD() - LOOK_AHEAD_DISTANCE, maxD));

  KVector2 robotRelativeToStart(robotCenterPosition->getX() - startingPosition.getX(),
                                robotCenterPosition->getY() - startingPosition.getY());
  nextPosition.subtractVector(&robotRelativeToStart);

  /*
  //calculate the next target point
  //start with the nearest point on the line to our robot; for a simple straight line, this is just the perpendicular
  //intersection point from our robot position to the line
  KVector2 nextPosition(targetPosition.getX() - startingPosition.getX(),
                        targetPosition.getY() - startingPosition.getY());
  double maxD = nextPosition.getD();

  KVector2 robotRelativeToStart(robotCenterPosition->getX() - startingPosition.getX(),
                                robotCenterPosition->getY() - startingPosition.getY());
  nextPosition.setD(1.0d);
  nextPosition.setD(min(robotRelativeToStart.dotVector(&nextPosition) + LOOK_AHEAD_DISTANCE, maxD));
  */

  /*
  //vector from the center of the robot to the target position
  KVector2 nextPosition(targetPosition.getX() - robotCenterPosition->getX(),
                        targetPosition.getY() - robotCenterPosition->getY());
  */
  
  //limit the distance to the max look-ahead distance
  if (nextPosition.getD() > LOOK_AHEAD_DISTANCE)
    nextPosition.setD(LOOK_AHEAD_DISTANCE);

  //calculate the left input
  leftInput = calculateInput(lighthouse.getLeftSensor(), &nextPosition);

  //calculate the right input
  rightInput = calculateInput(lighthouse.getRightSensor(), &nextPosition);

//SerialUSB.print(leftInput, 2);
//SerialUSB.print(" ");
//SerialUSB.println(rightInput, 2);
}

void MoveTowardPoint::start()
{
  leftPID.SetMode(MANUAL);
  rightPID.SetMode(MANUAL);

  //capture our starting position
  startingPosition.set(lighthouse.getPosition());
  distanceDrivenAlongPath = 0.0d;

  //update the inputs
  KVector2* currentPosition = lighthouse.getPosition();
  KVector2 deltaCenterToTarget(targetPosition.getX() - currentPosition->getX(),
                               targetPosition.getY() - currentPosition->getY());
  updateInputs(&deltaCenterToTarget);

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
}

bool MoveTowardPoint::loop()
{
  //the distance is greater than our look-ahead distance, so reduce it to that distance
  KVector2* currentPosition = lighthouse.getPosition();
  KVector2 deltaCenterToTarget(targetPosition.getX() - currentPosition->getX(),
                               targetPosition.getY() - currentPosition->getY());
  if (deltaCenterToTarget.getD2() < AUTODRIVE_POSITION_EPSILON_2) {
    leftPID.SetMode(MANUAL);
    rightPID.SetMode(MANUAL);
    return true;
  }

  updateInputs(&deltaCenterToTarget);

  leftPID.Compute();
  rightPID.Compute();

  //our base velocity is just a proportional represented by cosine of the angle from our current orientation to the target
//  KVector2* currentOrientation = lighthouse.getOrientation();
//  double baseVelocity = LINEAR_VELOCITY_POWER * cos(currentOrientation->angleToVector(&deltaCenterToTarget));
//  motors.setMotors(padInner(baseVelocity + leftOutput, MOTOR_MIN_POWER),
//                   padInner(baseVelocity + rightOutput, MOTOR_MIN_POWER));

  motors.setMotors(padInner(leftOutput, MOTOR_MIN_POWER),
                   padInner(rightOutput, MOTOR_MIN_POWER));

  /*
  SerialUSB.print("Orientation: ");
  SerialUSB.print(lighthouse.getOrientation()->getOrientation(), 3);
  SerialUSB.print(", Direction: ");
  SerialUSB.print(currentOrientation->angleToVector(&deltaToTarget), 3);
  SerialUSB.print(", Left: ");
  SerialUSB.print(leftInput, 3);
  SerialUSB.print(", Right: ");
  SerialUSB.println(rightInput, 3);
  */

  return false;
}

