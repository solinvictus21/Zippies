
#include <Arduino.h>
#include "zippies/math/ZCubicHermiteSpline.h"

// #define DEFAULT_CARDINAL_TIGHTNESS        0.707106781186548

ZCubicHermiteSpline::ZCubicHermiteSpline()
{
}

ZCubicHermiteSpline::ZCubicHermiteSpline(const ZippyWaypoint* kfs, int kfc)
    : keyframes(kfs),
      keyframeCount(kfc)
{
    generateSpline();
}

void ZCubicHermiteSpline::setKeyframes(const ZippyWaypoint* kfs, int kfc)
{
    deleteSpline();

    this->keyframes = kfs;
    this->keyframeCount = kfc;

    generateSpline();
}

void ZCubicHermiteSpline::generateSpline()
{
    if (!keyframeCount)
        return;

    positions = new ZVector2*[keyframeCount];
    tangents = new ZVector2*[keyframeCount];

    //calculate the tangents at each point
    const ZippyWaypoint* previousPosition = &keyframes[0];
    double previousTime = keyframes[0].timing;
    for (int i = 0; i < keyframeCount; i++)
    {
        const ZippyWaypoint* currentPosition = &keyframes[i];
        positions[i] = new ZVector2(currentPosition->x, currentPosition->y);

        const ZippyWaypoint* nextPosition;
        double currentTime;
        if (i < keyframeCount - 1) {
            currentTime = keyframes[i].timing;
            nextPosition = &keyframes[i+1];
        }
        else {
            currentTime = keyframes[keyframeCount-2].timing;
            nextPosition = currentPosition;
        }

        tangents[i] = new ZVector2();
        calculateTangent(
            previousPosition,
            previousTime,
            currentPosition,
            currentTime,
            nextPosition,
            tangents[i]);

        previousPosition = currentPosition;
        previousTime = currentTime;
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
    double time01,
    const ZippyWaypoint* position1,
    double time12,
    const ZippyWaypoint* position2,
    ZVector2* tangent)
{
    // /*
    double orientationRadians = M_PI * position1->orientation / 180.0;
    double orientationSin = sin(orientationRadians);
    double orientationCos = cos(orientationRadians);

    // /*
    double time02 = time01 + time12;
    /*
    double tangentX = M_PI * 1000.0 * (
        ((position1->x - position0->x) / time01) -
        ((position2->x - position0->x) / time02) +
        ((position2->x - position1->x) / time12));
    double tangentY = M_PI * 1000.0 * (
        ((position1->y - position0->y) / time01) -
        ((position2->y - position0->y) / time02) +
        ((position2->y - position1->y) / time12));
    */
    double tangentX = 2.0 * (
        ((position1->x - position0->x) * time01 / time02) -
        ((position2->x - position0->x)) +
        ((position2->x - position1->x) * time12 / time02));
    double tangentY = 2.0 * (
        ((position1->y - position0->y) * time01 / time02) -
        ((position2->y - position0->y)) +
        ((position2->y - position1->y) * time12 / time02));
    // */
    /*
    double tangentX = (2000.0 * (position2->x - position0->x)) / ((double)(time01 + time12));
    double tangentY = (2000.0 * (position2->y - position0->y)) / ((double)(time01 + time12));
    */

    double tangentMagnitude = abs((tangentX * orientationSin) + (tangentY * orientationCos));
    tangent->set(
        orientationSin * tangentMagnitude,
        orientationCos * tangentMagnitude);
}

void ZCubicHermiteSpline::deleteSpline()
{
    for (int i = 0; i < keyframeCount; i++) {
        delete positions[i];
        delete tangents[i];
    }

    if (positions) {
        delete[] positions;
        positions = NULL;
    }

    if (tangents) {
        delete[] tangents;
        tangents = NULL;
    }

    keyframes = NULL;
    keyframeCount = 0;
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
        if (currentPoint >= keyframeCount - 1)
        {
            //end of the spline
            currentTargetPosition.set(positions[keyframeCount - 1]);
            currentTargetVelocity.set(tangents[keyframeCount - 1]);
            return;
        }

        //scale the hermite tangents so that the velocities animate smoothly across the control points
        //information on how to calculate this scaling described here...
        //    https://www.cubic.org/docs/hermite.htm
        //
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
        currentOutboundFactor =
            ((double)(2.0 * keyframes[currentPoint].timing)) /
            ((double)(previousDeltaTime + keyframes[currentPoint].timing));
        if (currentPoint < keyframeCount - 2)
            nextInboundFactor =
                ((double)(2.0 * keyframes[currentPoint].timing)) /
                ((double)(keyframes[currentPoint].timing + keyframes[currentPoint + 1].timing));
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

  double v00 = (6.0 * tt) - (6.0 * t);
  double v10 = (3.0 * tt) - (4.0 * t) + 1.0;
  double v01 = (-6.0 * tt) + (6.0 * t);
  double v11 = (3.0 * tt) - (2.0 * t);
//   double timeFactor = t / 60
  currentTargetVelocity.set(
      interpolateCubicHermite(v00, v10, v01, v11,
          position1->getX(), tangent1->getX() * tangent1Factor, position2->getX(), tangent2->getX() * tangent2Factor) / 60.0,
      interpolateCubicHermite(v00, v10, v01, v11,
          position1->getY(), tangent1->getY() * tangent1Factor, position2->getY(), tangent2->getY() * tangent2Factor) / 60.0);
}

double interpolateCubicHermite(
    double c00, double c10, double c01, double c11,
    double p1, double t1, double p2, double t2)
{
    return (c00 * p1) + (c10 * t1) + (c01 * p2) + (c11 * t2);
}
