
#include "ZippyCommand.h"
#include "Lighthouse.h"
#include "MotorDriver.h"

//the minimum PCM value below which the motors do not turn
#define MOTOR_MIN_POWER                      4600.00d
#define LINEAR_VELOCITY_POWER               20000.00d
#define MAX_LINEAR_VELOCITY                   800.00d
#define MAX_ROTATIONAL_VELOCITY               270.00d
#define LINEAR_VELOCITY_POWER               20000.00d
#define LOOK_AHEAD_DISTANCE                   300.00d
#define HALF_TURNING_RADIUS                   M_PI_2
//#define HALF_TURNING_RADIUS                   1.963495408493621d
//#define HALF_TURNING_RADIUS                   1.178097245096172
#define HALF_SENSOR_SEPARATION 11.0d

//the radius squared (to prevent the need for an additional square root) when we are can consider the robot to be "at the target"
//currently set to 5cm, since sqrt(2500mm)/(10mm per cm) = 5cm
#define AUTODRIVE_POSITION_EPSILON_2         2500.0d
//#define AUTODRIVE_POSITION_EPSILON_2        10000.0d

#define AUTODRIVE_LINEAR_Kp                     8.0d
#define AUTODRIVE_LINEAR_Ki                     0.0d
#define AUTODRIVE_LINEAR_Kd                     0.1d

#define AUTODRIVE_ROTATIONAL_Kp                    40.0d
#define AUTODRIVE_ROTATIONAL_Ki                     0.0d
#define AUTODRIVE_ROTATIONAL_Kd                     0.0d

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

FollowPath::FollowPath(KVector2* pathPoints, int pathPointCount)
  : pathPoints(pathPoints),
    pathPointCount(pathPointCount),
    leftPID(&leftInput, &leftOutput, &leftSetPoint, AUTODRIVE_LINEAR_Kp, AUTODRIVE_LINEAR_Ki, AUTODRIVE_LINEAR_Kd, DIRECT),
    rightPID(&rightInput, &rightOutput, &rightSetPoint, AUTODRIVE_LINEAR_Kp, AUTODRIVE_LINEAR_Ki, AUTODRIVE_LINEAR_Kd, DIRECT)
{
  leftPID.SetOutputLimits(-LINEAR_VELOCITY_POWER, LINEAR_VELOCITY_POWER);
  rightPID.SetOutputLimits(-LINEAR_VELOCITY_POWER, LINEAR_VELOCITY_POWER);
}

//deltaToNextPosition is the next desired position, relative to the current center and orientation of the robot
double FollowPath::calculateInput(LighthouseSensor* sensor,
                                       KVector2* deltaToNextPosition)
{
  //step 1; obtain the sensor position in the global coordinate system
  KVector2* sensorPosition = sensor->getPosition();

  //now translate it to the coordinate system of the robot
  KVector2* robotCenterPosition = lighthouse.getPosition();
  KVector2 sensorPositionFromCenter(sensorPosition->getX() - robotCenterPosition->getX(),
                                    sensorPosition->getY() - robotCenterPosition->getY());
  sensorPositionFromCenter.rotate(-lighthouse.getOrientation()->getOrientation());
  
  //step 2; calculate the vector from current sensor position to desired sensor position  
  //start with the relative sensor position
  KVector2 desiredSensorPosition(&sensorPositionFromCenter);
//  desiredSensorPosition.printDebug();

  //rotate it by the amount of rotation from our current orientation to the desired position
  desiredSensorPosition.rotate(deltaToNextPosition->getOrientation());

  //add the it to the next desired position of the center of the robot
  desiredSensorPosition.addVector(deltaToNextPosition);
//  desiredSensorPosition.printDebug();

  //subtract the current relative sensor position from the center of the robot to obtain the vector from the current sensor
  //position to the desired sensor position
  desiredSensorPosition.subtractVector(&sensorPositionFromCenter);
//  desiredSensorPosition.printDebug();
//  SerialUSB.println();

  //now get the angle from the sensor vector to the vector from the sensor to the desired sensor position as a range from
  //+1.0 to -1.0; this gives us the mix of linear velocity and the rotational velocity
  double linearVelocityRatio = desiredSensorPosition.angleToVector(lighthouse.getOrientation()) / M_PI;
  linearVelocityRatio = (linearVelocityRatio < 0.0d ? 1.0d + linearVelocityRatio : -1.0d + linearVelocityRatio);
//  sensorPositionFromCenter.set(-sensorPositionFromCenter.getX(), -sensorPositionFromCenter.getY());
  double rotationalVelocityRatio = desiredSensorPosition.angleToVector(&sensorPositionFromCenter) / M_PI;
  return (MAX_LINEAR_VELOCITY * linearVelocityRatio) + (MAX_ROTATIONAL_VELOCITY * rotationalVelocityRatio);
}

void FollowPath::updateInputs()
{
  //calculate the next desired position along the path
  double deltaAlongPath = distanceDrivenAlongPath / deltaPosition.getD();

  //calculate the distance from the current position of the robot to the current target position
  KVector2 nextPosition(previousPosition->getX() + (deltaPosition.getX() * deltaAlongPath),
      previousPosition->getY() + (deltaPosition.getY() * deltaAlongPath));
  KVector2* robotCenterPosition = lighthouse.getPosition();
  nextPosition.subtractVector(robotCenterPosition);
  double distanceToNextPosition = nextPosition.getD();
  
  if (distanceToNextPosition < LOOK_AHEAD_DISTANCE) {
    //the robot is within range of the next position, so move it forward by the difference
    distanceDrivenAlongPath += (LOOK_AHEAD_DISTANCE - distanceToNextPosition);
    if (distanceDrivenAlongPath >= deltaPosition.getD()) {
      distanceDrivenAlongPath = deltaPosition.getD();
      nextPosition.set(&pathPoints[currentPathPoint]);
      nextPosition.subtractVector(robotCenterPosition);
    }
    else {
      deltaAlongPath = distanceDrivenAlongPath / deltaPosition.getD();
      nextPosition.set(previousPosition->getX() + (deltaPosition.getX() * deltaAlongPath),
          previousPosition->getY() + (deltaPosition.getY() * deltaAlongPath));
      nextPosition.subtractVector(robotCenterPosition);
    }
  }
  else
    nextPosition.setD(LOOK_AHEAD_DISTANCE);

  //now make the reference to the next position relative to the local coordinate system of the robot
  KVector2* lighthouseOrientation = lighthouse.getOrientation();
  nextPosition.rotate(-lighthouseOrientation->getOrientation());
//  nextPosition.printDebug();

  LighthouseSensor* leftSensor = lighthouse.getLeftSensor();
  LighthouseSensor* rightSensor = lighthouse.getRightSensor();
  double angleToPosition = nextPosition.getOrientation();
//  SerialUSB.print("A: ");
//  SerialUSB.println(angleToPosition, 2);
  if (angleToPosition <= -HALF_TURNING_RADIUS) {
    //fully left
    leftSetPoint = -MAX_ROTATIONAL_VELOCITY;
    rightSetPoint = MAX_ROTATIONAL_VELOCITY;
  }
  else if (angleToPosition >= HALF_TURNING_RADIUS) {
    //fully right
    leftSetPoint = MAX_ROTATIONAL_VELOCITY;
    rightSetPoint = -MAX_ROTATIONAL_VELOCITY;
  }
  else {
    angleToPosition /= HALF_TURNING_RADIUS;
    double linearVelocity = MAX_LINEAR_VELOCITY * (1.0d - abs(angleToPosition));
    double rotationalVelocity = MAX_ROTATIONAL_VELOCITY * angleToPosition;
    leftSetPoint = linearVelocity + rotationalVelocity;
    rightSetPoint = linearVelocity - rotationalVelocity;
//    leftSetPoint = linearVelocity;
//    rightSetPoint = linearVelocity;
//    leftSetPoint = rotationalVelocity;
//    rightSetPoint = -rotationalVelocity;

    /*
    SerialUSB.print("LV: ");
    SerialUSB.print(linearVelocity, 2);
    SerialUSB.print("   RV: ");
    SerialUSB.print(rotationalVelocity, 2);
    SerialUSB.print("   LS: ");
    SerialUSB.print(leftSetPoint, 2);
    SerialUSB.print("   RS: ");
    SerialUSB.println(rightSetPoint, 2);
    */
  }

  leftInput = leftSensor->getVelocity();
//  SerialUSB.println(leftInput, 2);
  rightInput = rightSensor->getVelocity();
//  SerialUSB.println(rightInput, 2);
  /*
  //calculate the left set point and input
  LighthouseSensor* nextSensor = lighthouse.getLeftSensor();
  leftSetPoint = calculateInput(nextSensor, &nextPosition);
  leftInput = nextSensor->getVelocity();

  //calculate the right set point and input
  nextSensor = lighthouse.getRightSensor();
  rightSetPoint = calculateInput(nextSensor, &nextPosition);
  leftInput = nextSensor->getVelocity();
  */

//  SerialUSB.print(leftInput, 2);
//  SerialUSB.print(" ");
//  SerialUSB.println(rightInput, 2);
//  SerialUSB.println();
}

void FollowPath::start()
{
  leftPID.SetMode(MANUAL);
  rightPID.SetMode(MANUAL);

  //capture our starting position
  firstPosition.set(lighthouse.getPosition());
  previousPosition = &firstPosition;
  currentPathPoint = 0;
//  startingPosition.printDebug();
//  SerialUSB.println();
//  deltaPosition.printDebug();
//  SerialUSB.println();
  distanceDrivenAlongPath = 0.0d;

  //update the inputs
  updateInputs();

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
}

bool FollowPath::loop()
{
  updateInputs();

  leftPID.Compute();
  rightPID.Compute();

  //the distance is greater than our look-ahead distance, so reduce it to that distance
  KVector2* currentPosition = lighthouse.getPosition();
  KVector2 deltaCenterToTarget(pathPoints[currentPathPoint].getX() - currentPosition->getX(),
      pathPoints[currentPathPoint].getY() - currentPosition->getY());
  if (deltaCenterToTarget.getD2() < AUTODRIVE_POSITION_EPSILON_2) {
    leftPID.SetMode(MANUAL);
    rightPID.SetMode(MANUAL);
    return true;
  }

//  SerialUSB.print("L: ");
//  SerialUSB.print(leftOutput, 2);
//  SerialUSB.print("   R: ");
//  SerialUSB.println(rightOutput, 2);

  motors.setMotors(padInner(-leftOutput, MOTOR_MIN_POWER),
                   padInner(-rightOutput, MOTOR_MIN_POWER));

  return false;
}

