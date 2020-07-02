
#include "zippies/controllers/MotorTuningController.h"
#include "zippies/config/ZippyConfig.h"

#define MEASUREMENT_TIME_DELTA         500
// #define SUCCESS_TIME_DELTA            5000
#define SUCCESS_ITERATIONS                   10
#define EPSILON_FACTOR                   0.2d

#define ANGULAR_EPSILON_STICTION                      0.052359877559830d  //3 degrees
// #define ANGULAR_EPSILON                               0.087266462599716d  //5 degrees
// #define ANGULAR_EPSILON                               0.261799387799149d  //15 degrees
// #define ANGULAR_EPSILON                               0.523598775598299d  //30 degrees
#define ANGULAR_EPSILON                               M_PI_4  //45 degrees
// #define ANGULAR_EPSILON                               M_PI_2  //90 degrees
const double ANGULAR_EPSILON_MIN = ANGULAR_EPSILON * (1.0d - EPSILON_FACTOR);
const double ANGULAR_EPSILON_MAX = ANGULAR_EPSILON * (1.0d + EPSILON_FACTOR);

// #define LINEAR_VELOCITY_THRESHOLD_MIN   20.0d
// #define ANGULAR_VELOCITY_THRESHOLD_MAX   0.017453292519943d  //1 degree
// #define ANGULAR_VELOCITY_THRESHOLD_MAX   0.087266462599716d  //5 degrees
#define TUNING_FACTOR_ADJUSTMENT         0.005d
// #define MOTOR_OUTPUT_MIN              4130.0d  //works well at 30mm/s
// #define MOTOR_OUTPUT_MIN              5500.0d

#define TUNING_KP                       10.0d
#define TUNING_KI                        0.0d
#define TUNING_KD                        0.0d
#define MOTOR_OUTPUT_MAX             10000.0d

MotorTuningController::MotorTuningController(SensorFusor* s)
  : sensors(s),
    leftMotorDeadZone(MOTOR_DEAD_ZONE_LEFT),
    rightMotorDeadZone(MOTOR_DEAD_ZONE_RIGHT),
    currentTuningState(MotorTuningState::LeftWheelForward),
    velocityPID(
        &velocityInput, &velocityOutput, &velocitySetPoint,
        TUNING_KP, TUNING_KI, TUNING_KD, P_ON_E, DIRECT)
{
  velocityPID.SetSampleTime(MEASUREMENT_TIME_DELTA);
  velocityPID.SetOutputLimits(-MOTOR_OUTPUT_MAX, MOTOR_OUTPUT_MAX);
}

void MotorTuningController::start(unsigned long startTime)
{
  switch (currentTuningState) {
    case MotorTuningState::LeftWheelForward:
      motorDeadZone = leftMotorDeadZone;
      motors.setMotorsDirect(motorDeadZone, 0.0d);
      break;
    case MotorTuningState::RightWheelForward:
      motorDeadZone = rightMotorDeadZone;
      motors.setMotorsDirect(0.0d, motorDeadZone);
      break;
  }
  motorStiction = 0.0d;
  successIterationCounter = 0;
  previousPosition.set(sensors->getPosition());
  previousUpdateTimeStamp = startTime;

  /*
  inputAccumulator.reset();
  outputAccumulator.reset();
  previousPosition.set(sensors->getPosition());
  currentVelocity.reset();
  velocityInput = 0.0d;
  velocitySetPoint = LINEAR_VELOCITY_THRESHOLD_MIN;
  velocityOutput = 0.0d;
  velocityPID.SetMode(AUTOMATIC);
  currentTuningState = MotorTuningState::Forward;
  motors.setMotorsDirect(MOTOR_OUTPUT_MIN, MOTOR_OUTPUT_MIN);
  previousUpdateTimeStamp = startTime;
  */
}

void MotorTuningController::loop(unsigned long currentTime)
{
  if (currentTime - previousUpdateTimeStamp < MEASUREMENT_TIME_DELTA)
    return;
  previousUpdateTimeStamp += MEASUREMENT_TIME_DELTA;

  const ZMatrix2* currentPosition = sensors->getPosition();
  double deltaOrientation = subtractAngles(currentPosition->orientation.get(), previousPosition.orientation.get());
  previousPosition.set(currentPosition);

  switch (currentTuningState) {

    case MotorTuningState::RightWheelForward:
      if (testComplete(currentTime, -deltaOrientation, ANGULAR_EPSILON_MIN, ANGULAR_EPSILON_MAX)) {
        //capture test stats
        rightMotorStiction = motorStiction;
        rightMotorDeadZone = motorDeadZone;
        // rightMotorAverage = rightMotorAverage ? (rightMotorAverage + successZoneAverage) / 2.0d : successZoneAverage;
        rightMotorStatistics.accumulate(rightMotorDeadZone);

        displayDeadZoneData();

        //move to next tuning phase
        motorStiction = 0.0d;
        // successZoneTimeStamp = 0;
        successIterationCounter = 0;
        motorDeadZone = leftMotorDeadZone;
        // motorDeadZone = leftMotorStatistics.getAverage();
        // motorDeadZone = leftMotorStatistics.getAverage();
        motorDeadZone = leftMotorStatistics.getSampleCount() ? leftMotorStatistics.getAverage() : leftMotorDeadZone;
        motors.setMotorsDirect(motorDeadZone, 0.0d);
        currentTuningState = MotorTuningState::LeftWheelForward;
      }
      else
        motors.setMotorsDirect(0.0d, motorDeadZone);
      break;

    case MotorTuningState::LeftWheelForward:
      if (testComplete(currentTime, deltaOrientation, ANGULAR_EPSILON_MIN, ANGULAR_EPSILON_MAX)) {
        //capture test stats
        leftMotorStiction = motorStiction;
        leftMotorDeadZone = motorDeadZone;
        // leftMotorAverage = leftMotorAverage ? (leftMotorAverage + successZoneAverage) / 2.0d : successZoneAverage;
        leftMotorStatistics.accumulate(leftMotorDeadZone);

        displayDeadZoneData();

        //move to next tuning phase
        motorStiction = 0.0d;
        // successZoneTimeStamp = 0;
        successIterationCounter = 0;
        // motorDeadZone = rightMotorDeadZone;
        // motorDeadZone = rightMotorStatistics.getAverage();
        motorDeadZone = rightMotorStatistics.getSampleCount() ? rightMotorStatistics.getAverage() : rightMotorDeadZone;
        motors.setMotorsDirect(0.0d, motorDeadZone);
        currentTuningState = MotorTuningState::RightWheelForward;
      }
      else
        motors.setMotorsDirect(motorDeadZone, 0.0d);
      break;

  }

/*
  const ZMatrix2* currentPosition = sensors->getPosition();
  currentVelocity.set(currentPosition);
  currentVelocity.unconcat(&previousPosition);
  previousPosition.set(currentPosition);

  if (currentVelocity.orientation.get() == 0.0d)
    velocityInput = abs(currentVelocity.position.getY());
  else {
    double rG = currentVelocity.position.getD();
    double thetaG = currentVelocity.orientation.get() / 2.0d;
    velocityInput = abs((rG * thetaG) / (2.0d * sin(thetaG)));
  }
  velocityInput *= 1000.0d / ((double)MEASUREMENT_TIME_DELTA);
  inputAccumulator.accumulate(velocityInput);

  displayTestData();
  */
  /*
  velocityPID.Compute();
  outputAccumulator.accumulate(velocityOutput);
  motors.setMotorsDirect(MOTOR_OUTPUT_MIN + velocityOutput, MOTOR_OUTPUT_MIN + velocityOutput);
  */
}

bool MotorTuningController::testComplete(unsigned long currentTime, double testResult, double testMin, double testMax)
{
  if (!motorStiction && testResult >= ANGULAR_EPSILON_STICTION)
    motorStiction = motorDeadZone;

  if (testResult < testMin) {
    // successZoneTimeStamp = 0;
    successIterationCounter = 0;
    motorDeadZone += motorDeadZone * TUNING_FACTOR_ADJUSTMENT;
  }
  else if (testResult >= testMax) {
    // successZoneTimeStamp = 0;
    successIterationCounter = 0;
    motorDeadZone -= motorDeadZone * (TUNING_FACTOR_ADJUSTMENT / 2.0d);
  }
  else if (!successIterationCounter) {
    successIterationCounter = SUCCESS_ITERATIONS;
    successZoneTotal = testResult;
  }
  else {
    successIterationCounter--;
    if (!successIterationCounter) {
      // successZoneAverage = successZoneTotal / (((double)SUCCESS_TIME_DELTA) / ((double)MEASUREMENT_TIME_DELTA));
      successZoneAverage = successZoneTotal / ((double)SUCCESS_ITERATIONS);
      return true;
    }

    successZoneTotal += testResult;
  }

  return false;
}

void MotorTuningController::displayDeadZoneData() {
  face.clearScreen();
  uint8_t y = 0;
  // uint8_t fontHeight = face.getFontHeight();
  // SerialUSB.println("Displaying test data.");
  face.displayLabelAndData(0, y, "S", rightMotorStiction, 1);
  face.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "S", leftMotorStiction, 1);
  y += face.getFontHeight();
  face.displayLabelAndData(0, y, "T", rightMotorDeadZone, 1);
  face.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "T", leftMotorDeadZone, 1);
  // y += face.getFontHeight();
  // face.displayLabelAndData(0, y, "RA", rightMotorAverage * 180.0d / M_PI, 1);
  // face.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "LA", leftMotorAverage * 180.0d / M_PI, 1);
  y += face.getFontHeight();
  face.displayLabelAndData(0, y, "A", rightMotorStatistics.getAverage(), 1);
  face.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "A", leftMotorStatistics.getAverage(), 1);
  y += face.getFontHeight();
  face.displayLabelAndData(0, y, "D", rightMotorStatistics.getStandardDeviation(), 1);
  face.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "D", leftMotorStatistics.getStandardDeviation(), 1);
  y += face.getFontHeight();
  face.displayLabelAndData(0, y, "C", rightMotorStatistics.getSampleCount());
  face.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "C", leftMotorStatistics.getSampleCount());
  y += face.getFontHeight();
  face.displayLabelAndData(SCREEN_WIDTH_PIXELS_2/2, y, "+-",
      abs(rightMotorStatistics.getAverage() - leftMotorStatistics.getAverage()) / 2.0d);
}

void MotorTuningController::displayTestData()
{
  face.clearScreen();
  uint8_t x = 0;
  uint8_t y = 0;
  uint8_t fontHeight = face.getFontHeight();
  double linearVelocity = currentVelocity.position.getD();
  if (currentVelocity.position.getY() < 0.0d)
    linearVelocity = -linearVelocity;
  face.displayLabelAndData(x, y, "L", velocityOutput, 2);
  face.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "R", velocityOutput, 2);
  y += fontHeight;
  face.displayLabelAndData(x, y, "V", velocityInput, 2);
  face.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "O", currentVelocity.orientation.get(), 2);
  y += fontHeight;
  face.displayLabelAndData(x, y, "AI", inputAccumulator.getAverage(), 2);
  face.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "SI", inputAccumulator.getStandardDeviation(), 2);
  y += fontHeight;
  face.displayLabelAndData(x, y, "AO", outputAccumulator.getAverage(), 2);
  face.displayLabelAndData(SCREEN_WIDTH_PIXELS_2, y, "SO", outputAccumulator.getStandardDeviation(), 2);
}

void MotorTuningController::stop()
{
  velocityPID.SetMode(MANUAL);
  motors.stopMotors();
}
