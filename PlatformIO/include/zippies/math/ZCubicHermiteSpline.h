
#ifndef _ZCUBICHERMITESPLINE_H_
#define _ZCUBICHERMITESPLINE_H_

#include "ZVector2.h"
#include "ZMatrix2.h"

typedef struct _ZippyWaypoint
{
    double x;
    double y;
    double orientation;
    unsigned long timing;
} ZippyWaypoint;

class ZCubicHermiteSpline
{

private:
    const ZippyWaypoint* keyframes;
    int keyframeCount;
    ZVector2** positions;
    ZVector2** tangents;

    unsigned long startTime;
    unsigned long previousDeltaTime;
    int currentPoint;
    unsigned long currentPointStartTime;
    double currentOutboundFactor;
    double nextInboundFactor;
    ZVector2 currentTargetPosition;
    ZVector2 currentTargetVelocity;
    ZMatrix2 currentTargetMatrix;

    void calculateTangent(
        const ZippyWaypoint* position0,
        const ZippyWaypoint* position1,
        const ZippyWaypoint* position2,
        ZVector2* tangent);

    void interpolate(
        double t,
        const ZVector2* position1,
        const ZVector2* tangent1,
        double tangent1Factor,
        const ZVector2* position2,
        const ZVector2* tangent2,
        double tangent2Factor);
    double interpolateCubicHermite(
        double c00, double c10, double c01, double c11,
        double p1, double t1, double p2, double t2);

public:
    ZCubicHermiteSpline(const ZippyWaypoint* keyframes, int keyframeCount);

    void start(unsigned long st);
    void interpolate(unsigned long currentTime);

    const ZVector2* getTargetPosition() const { return &currentTargetPosition; };
    const ZVector2* getTargetVelocity() const { return &currentTargetVelocity; };
    const ZMatrix2* getTargetMatrix() const { return &currentTargetMatrix; };

    bool isCompleted() {
        return currentPoint >= keyframeCount;
    }

    ~ZCubicHermiteSpline() {
        for (int i = 0; i < keyframeCount; i++) {
            delete positions[i];
            delete tangents[i];
        }
        delete[] positions;
        delete[] tangents;
    }

};

#endif