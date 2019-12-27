
#include <PID_v1.h>
#include "zippies/controllers/MotorTuningController.h"
#include "zippies/config/MotorConfig.h"

#define MEASUREMENT_TIME_DELTA         250
#define LINEAR_VELOCITY_THRESHOLD_MIN   20.0d
// #define ANGULAR_VELOCITY_THRESHOLD_MAX   0.017453292519943d  //1 degree
#define ANGULAR_VELOCITY_THRESHOLD_MAX   0.087266462599716d  //5 degrees
#define EPSILON_FACTOR                   0.2d
#define TUNING_FACTOR_ADJUSTMENT         0.1d
// #define MOTOR_OUTPUT_MIN              4130.0d  //works well at 30mm/s
#define MOTOR_OUTPUT_MIN              5000.0d
#define MOTOR_OUTPUT_MAX             10000.0d

#define TUNING_KP                       10.0d
#define TUNING_KI                        0.0d
#define TUNING_KD                        0.0d

MotorTuningController::MotorTuningController(SensorFusor* s)
  : sensors(s),
    motors(MOTOR_DEAD_ZONE_ABS),
    velocityPID(
        &velocityInput, &velocityOutput, &velocitySetPoint,
        TUNING_KP, TUNING_KI, TUNING_KD, P_ON_E, DIRECT)
{
  velocityPID.SetSampleTime(MEASUREMENT_TIME_DELTA);
  velocityPID.SetOutputLimits(-MOTOR_OUTPUT_MAX, MOTOR_OUTPUT_MAX);
}

void MotorTuningController::start(unsigned long startTime)
{
  motors.stopMotors();
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
}

void MotorTuningController::loop(unsigned long currentTime)
{
  if (currentTime - previousUpdateTimeStamp < MEASUREMENT_TIME_DELTA)
    return;

  const KMatrix2* currentPosition = sensors->getPosition();
  currentVelocity.set(currentPosition);
  currentVelocity.unconcat(&previousPosition);
  previousPosition.set(currentPosition);
  previousUpdateTimeStamp = currentTime;

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
  /*
  velocityPID.Compute();
  outputAccumulator.accumulate(velocityOutput);
  motors.setMotorsDirect(MOTOR_OUTPUT_MIN + velocityOutput, MOTOR_OUTPUT_MIN + velocityOutput);
  */
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
