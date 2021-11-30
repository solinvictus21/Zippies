
#include <SPI.h>
#include "zippies/hardware/ZippyLA.h"

#define PID_LINEAR_KP                        180.0
#define PID_LINEAR_KI                          0.0
#define PID_LINEAR_KD                         60.0
#define PID_ANGULAR_KP                      9400.0
#define PID_ANGULAR_KI                         0.0
#define PID_ANGULAR_KD                      1200.0
#define PID_OUTPUT_LIMIT                   60000.0

ZippyLA::ZippyLA()
  : linearPID(60,
          PID_LINEAR_KP, PID_LINEAR_KI, PID_LINEAR_KD,
          -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT,
          false, false),
    angularPID(60,
          PID_ANGULAR_KP, PID_ANGULAR_KI, PID_ANGULAR_KD,
          -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT,
          false, false)
{
}

void ZippyLA::start()
{
    linearPID.start();
    angularPID.start();
}

void ZippyLA::setInput(const ZMatrix2* positionDelta)
{
    if (positionDelta->position.getX() == 0.0) {
        linearInput = positionDelta->position.getY();
        angularInput = 0.0;
        return;
    }

    linearInput =
        // (positionDelta->position.getD() * positionDelta->position.atan()) /
        // sin(positionDelta->position.atan());
        (positionDelta->position.getD() * positionDelta->orientation.get()) /
        positionDelta->orientation.sin();
    angularInput = positionDelta->orientation.get();
}

void ZippyLA::move(double linearVelocity, double angularVelocity)
{
    double linearMotorOutput = linearPID.compute(linearInput, linearVelocity);
    double angularMotorOutput = angularPID.compute(angularInput, angularVelocity);
    motors.setMotors(
        linearMotorOutput + angularMotorOutput,
        linearMotorOutput - angularMotorOutput);
}

void ZippyLA::turn(double yOffset, double angularVelocity)
{
    double linearMotorOutput;
    linearMotorOutput = linearPID.compute(linearInput, yOffset);
    /*
    if (yOffset == 0.0) {
        linearPID.compute(0.0, 0.0);
        linearMotorOutput = 0.0;
    }
    else if (angularVelocity == 0.0) {
        linearMotorOutput = linearPID.compute(linearInput, yOffset);
    }
    else {
        double thetaG = angularVelocity / 2.0;
        // double rG = yOffset / cos(thetaG);
        // double linearVelocity = (rG * thetaG) / sin(thetaG);
        // double linearVelocity = (yOffset / cos(thetaG)) * (thetaG / sin(thetaG));
        // double linearVelocity = (yOffset / cos(thetaG)) * (thetaG / sin(thetaG));
        // double linearVelocity = (yOffset * thetaG) / (cos(thetaG) * sin(thetaG));
        double linearVelocity = (yOffset * thetaG) / tan(thetaG);
        linearMotorOutput = linearPID.compute(linearInput, linearVelocity);
    }
    */

    double angularMotorOutput = angularPID.compute(angularInput, angularVelocity);
    motors.setMotors(
        linearMotorOutput + angularMotorOutput,
        linearMotorOutput - angularMotorOutput);
}

//stopped when the signal from the lighthouse is lost
void ZippyLA::stop()
{
    linearPID.stop();
    angularPID.stop();
    motors.stopMotors();
}
