
#include "ZippyCommand.h"
#include "LighthouseSensor.h"
#include "MotorDriver.h"

//the minimum PCM value below which the motors do not turn
#define MOTOR_MIN_POWER                      4600.00d
#define LINEAR_VELOCITY_POWER                5000.00d
#define LOOK_AHEAD_DISTANCE                   100.00d

//the radius squared (to prevent the need for an additional square root) when we are can consider the robot to be "at the target"
//currently set to 5cm, since sqrt(2500mm)/(10mm per cm) = 5cm
#define AUTODRIVE_POSITION_EPSILON_2         2500.0d

#define AUTODRIVE_LINEAR_Kp               1200.0d
#define AUTODRIVE_LINEAR_Ki                100.0d
#define AUTODRIVE_LINEAR_Kd                200.0d

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
  : currentTargetPosition(x, y),
    leftPID(&leftInput, &leftOutput, &leftSetPoint, AUTODRIVE_LINEAR_Kp, AUTODRIVE_LINEAR_Ki, AUTODRIVE_LINEAR_Kd, DIRECT),
    rightPID(&rightInput, &rightOutput, &rightSetPoint, AUTODRIVE_LINEAR_Kp, AUTODRIVE_LINEAR_Ki, AUTODRIVE_LINEAR_Kd, DIRECT)
{
  leftPID.SetOutputLimits(-LINEAR_VELOCITY_POWER, LINEAR_VELOCITY_POWER);
  rightPID.SetOutputLimits(-LINEAR_VELOCITY_POWER, LINEAR_VELOCITY_POWER);
}

void MoveTowardPoint::updateInputs()
{
  //calculate a delta vector that is a hardwired distance from the center of the robot to the target position
  KVector2* robotCenterPosition = lighthouse.getPosition();
  KVector2 nextPosition(currentTargetPosition.getX() - robotCenterPosition->getX(),
                        currentTargetPosition.getY() - robotCenterPosition->getY(),
                        LOOK_AHEAD_DISTANCE);
  nextPosition.set(robotCenterPosition->getX() + nextPosition.getX(),
                   robotCenterPosition->getY() + nextPosition.getY());

  //calculate the left input
  LighthouseSensor* sensor = lighthouse.getLeftSensor();
  KVector2* sensorPosition = sensor->getPosition();
  
  //vector from center of robot to this sensor
  KVector2 sensorPositionFromCenter(sensorPosition->getX() - robotCenterPosition->getX(),
                                    sensorPosition->getY() - robotCenterPosition->getY(),
                                    1.0d);

  //vector from sensor offset to target position
  KVector2 deltaSensorToTarget(nextPosition.getX() - sensorPosition->getX(),
                               nextPosition.getY() - sensorPosition->getY(),
                               1.0d);

  //input is the angle from our sensor vector to the target position
  leftInput = cos(sensorPositionFromCenter.angleToVector(&deltaSensorToTarget));

  //calculate the right input
  sensor = lighthouse.getRightSensor();
  sensorPosition = sensor->getPosition();

  //vector from center of robot to this sensor
  sensorPositionFromCenter.set(sensorPosition->getX() - robotCenterPosition->getX(),
                               sensorPosition->getY() - robotCenterPosition->getY(),
                               1.0d);

  //vector from sensor to target position
  deltaSensorToTarget.set(nextPosition.getX() - sensorPosition->getX(),
                          nextPosition.getY() - sensorPosition->getY(),
                          1.0d);

  //input is the angle from our sensor vector to the target position
  rightInput = cos(sensorPositionFromCenter.angleToVector(&deltaSensorToTarget));
}

void MoveTowardPoint::start()
{
  leftPID.SetMode(MANUAL);
  rightPID.SetMode(MANUAL);

  updateInputs();

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
}

bool MoveTowardPoint::loop()
{
  KVector2* currentPosition = lighthouse.getPosition();
  KVector2 deltaCenterToTarget(currentTargetPosition.getX() - currentPosition->getX(),
                               currentTargetPosition.getY() - currentPosition->getY());
  if (deltaCenterToTarget.getD2() < AUTODRIVE_POSITION_EPSILON_2) {
    leftPID.SetMode(MANUAL);
    rightPID.SetMode(MANUAL);
    return true;
  }

  updateInputs();

  leftPID.Compute();
  rightPID.Compute();

  //our base velocity is just a proportional represented by cosine of the angle from our current orientation to the target
  KVector2* currentOrientation = lighthouse.getOrientation();
  double baseVelocity = LINEAR_VELOCITY_POWER * cos(currentOrientation->angleToVector(&deltaCenterToTarget));
  motors.setMotors(padInner(baseVelocity + leftOutput, MOTOR_MIN_POWER),
                   padInner(baseVelocity + rightOutput, MOTOR_MIN_POWER));

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

