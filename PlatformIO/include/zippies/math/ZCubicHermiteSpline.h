
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

double interpolateCubicHermite(
    double c00, double c10, double c01, double c11,
    double p1, double t1, double p2, double t2);

class ZCubicHermiteSpline
{

private:
    const ZippyWaypoint* keyframes = NULL;
    int keyframeCount = 0;
    ZVector2** positions = NULL;
    ZVector2** tangents = NULL;

    unsigned long startTime;
    unsigned long previousDeltaTime;
    int currentPoint;
    unsigned long currentPointStartTime;
    double currentOutboundFactor;
    double nextInboundFactor;
    ZVector2 currentTargetPosition;
    ZVector2 currentTargetVelocity;

    void deleteSpline();
    void generateSpline();
    void calculateTangent(
        const ZippyWaypoint* position0,
        double time01,
        const ZippyWaypoint* position1,
        double time12,
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

public:
    ZCubicHermiteSpline();
    ZCubicHermiteSpline(const ZippyWaypoint* keyframes, int keyframeCount);

    void setKeyframes(const ZippyWaypoint* keyframes, int keyframeCount);
    void start(unsigned long st);
    void interpolate(unsigned long currentTime);

    const ZVector2* getTargetPosition() const { return &currentTargetPosition; };
    const ZVector2* getTargetVelocity() const { return &currentTargetVelocity; };

    bool isCompleted() { return currentPoint >= keyframeCount - 1; }

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