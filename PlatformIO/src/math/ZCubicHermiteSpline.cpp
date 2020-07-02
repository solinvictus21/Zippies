
#include <Arduino.h>
#include "zippies/math/ZCubicHermiteSpline.h"

ZCubicHermiteSpline::ZCubicHermiteSpline(const ZippyWaypoint* kfs, int kfc)
    : keyframes(kfs),
      keyframeCount(kfc)
{
    positions = new ZVector2*[keyframeCount];
    tangents = new ZVector2*[keyframeCount];
    const ZippyWaypoint* previousPosition = &keyframes[0];
    for (int i = 0; i < keyframeCount; i++)
    {
        const ZippyWaypoint* currentPosition = &keyframes[i];
        positions[i] = new ZVector2(currentPosition->x, currentPosition->y);
        tangents[i] = new ZVector2();
        const ZippyWaypoint* nextPosition;
        if (i < keyframeCount - 1)
            nextPosition = &keyframes[i+1];
        else
            nextPosition = currentPosition;
        calculateTangent(
            previousPosition, currentPosition, nextPosition,
            tangents[i]);
        /*
        SerialUSB.print("positions/tangents ");
        SerialUSB.println(i);
        positions[i]->printDebug();
        tangents[i]->printDebug();
        previousPosition = currentPosition;
        */
    }
}

void ZCubicHermiteSpline::calculateTangent(
    const ZippyWaypoint* position0,
    const ZippyWaypoint* position1,
    const ZippyWaypoint* position2,
    ZVector2* tangent)
{
    double tangentMagnitude = sqrt(sq(position2->x - position0->x) + sq(position2->y - position0->y));// / 2.0;
    double orientationRadians = M_PI * position1->orientation / 180.0;
    tangent->set(
        sin(orientationRadians) * tangentMagnitude,
        cos(orientationRadians) * tangentMagnitude);
}

void ZCubicHermiteSpline::start(unsigned long st)
{
    if (keyframeCount < 2)
        return;

    previousDeltaTime = 0.0;
    currentPoint = 0;
    currentOutboundFactor = 1.0;
    if (currentPoint < keyframeCount - 2)
        nextInboundFactor = (2.0 * keyframes[currentPoint].timing) / (keyframes[currentPoint].timing + keyframes[currentPoint + 1].timing);
    else
        nextInboundFactor = 1.0;
    currentPointStartTime = st;
    this->startTime = st;
}

void ZCubicHermiteSpline::interpolate(unsigned long currentTime)
{
    if (isCompleted())
        return;

    unsigned long deltaTime = currentTime - currentPointStartTime;
    while (deltaTime >= keyframes[currentPoint].timing)
    {
        previousDeltaTime = keyframes[currentPoint].timing;
        deltaTime -= previousDeltaTime;
        currentPointStartTime += previousDeltaTime;
        currentPoint++;
        if (currentPoint == keyframeCount)
        {
            currentTargetPosition.set(positions[keyframeCount - 1]);
            currentTargetVelocity.set(tangents[keyframeCount - 1]);
            return;
        }
        //scale the hermite tangents so that the velocities animate smoothly across the control points
        //information on how to calculate this scaling described here...
        //    https://www.cubic.org/docs/hermite.htm
        //note that this documentation does not describe how the first and last tangents should be scaled, but
        //after some experimentation and a little reasoning, I figured out that the scaling calculations for
        //the internal control point tangents are a simplied version of this formula...
        //    S = T / Ta
        //  where...
        //    S = amount to scale a given tangent
        //    Tn = time to move along current spline segment
        //    Ta = average time between two segments
        //  then, for example, the inbound tangent becomes...
        //    Ta = (Tn + Tn-1) / 2
        //    S = Tn / ((Tn + Tn-1) / 2) = (2 * Tn) / (Tn + Tn-1)
        //it's essentialy the current time weighted over the average between the two times; this lead me to
        //realize and confirm experimentally that the first outbound and last inbound tangents should not be
        //weighted at all, so the value is just 1.0
        currentOutboundFactor = (2.0 * keyframes[currentPoint].timing) / (previousDeltaTime + keyframes[currentPoint].timing);
        if (currentPoint < keyframeCount - 2)
            nextInboundFactor = (2.0 * keyframes[currentPoint].timing) / (keyframes[currentPoint].timing + keyframes[currentPoint + 1].timing);
        else
            nextInboundFactor = 1.0;
    }

    interpolate(
        ((double)deltaTime) / ((double)keyframes[currentPoint].timing),
        positions[currentPoint],
        tangents[currentPoint],
        currentOutboundFactor,
        positions[currentPoint + 1],
        tangents[currentPoint + 1],
        nextInboundFactor);
}

void ZCubicHermiteSpline::interpolate(
  double t,
  const ZVector2* position1,
  const ZVector2* tangent1,
  double tangent1Factor,
  const ZVector2* position2,
  const ZVector2* tangent2,
  double tangent2Factor)
{
  double tt = t * t;

  double uu = sq(1.0 - t);
  double t2 = 2.0 * t;
  double p00 = (1.0 + t2) * uu;
  double p10 = t * uu;
  double p01 = tt * (3.0 - t2);
  double p11 = tt * (t - 1.0);
  currentTargetPosition.set(
      interpolateCubicHermite(p00, p10, p01, p11,
          position1->getX(), tangent1->getX() * tangent1Factor, position2->getX(), tangent2->getX() * tangent2Factor),
      interpolateCubicHermite(p00, p10, p01, p11,
          position1->getY(), tangent1->getY() * tangent1Factor, position2->getY(), tangent2->getY() * tangent2Factor));

  double v00 = 6.0 * tt - 6.0 * t;
  double v10 = 3.0 * tt - 4.0 * t + 1;
  double v01 = -6.0 * tt + 6.0 * t;
  double v11 = 3.0 * tt - 2 * t;
  currentTargetVelocity.set(
      interpolateCubicHermite(v00, v10, v01, v11,
          position1->getX(), tangent1->getX() * tangent1Factor, position2->getX(), tangent2->getX() * tangent2Factor),
      interpolateCubicHermite(v00, v10, v01, v11,
          position1->getY(), tangent1->getY() * tangent1Factor, position2->getY(), tangent2->getY() * tangent2Factor));
}

double ZCubicHermiteSpline::interpolateCubicHermite(
    double c00, double c10, double c01, double c11,
    double p1, double t1, double p2, double t2)
{
    return (c00 * p1) + (c10 * t1) + (c01 * p2) + (c11 * t2);
}
