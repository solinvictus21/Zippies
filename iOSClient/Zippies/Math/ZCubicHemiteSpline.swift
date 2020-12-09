
import Foundation

class ZCubicHermiteSpline: NSObject
{
    
    fileprivate let positions: [ZVector2]
    fileprivate let tangents: [ZVector2]
    fileprivate let times: [Double]

    fileprivate var previousDeltaTime: Double = 0.0
    fileprivate var currentPoint: Int = 0
    fileprivate var currentOutboundFactor: Double = 0.0
    fileprivate var nextInboundFactor: Double = 0.0
    fileprivate var currentPointStartTime: Double = 0.0
    fileprivate var isStarted: Bool = false

    let TotalTime: Double
    
    var started: Bool { get { return isStarted } }

    init(
        positions: [ZVector2],
        tangents: [ZVector2],
        times: [Double])
    {
        self.positions = positions
        self.tangents = tangents
        self.times = times
//        print("positions: ", positions.count)
//        print("tangents: ", tangents.count)
//        print("times: ", times.count)
        var totalTime = 0.0
        for i in times {
            totalTime += i
            //Debug.Log("total time: " + TotalTime)
        }
        self.TotalTime = totalTime
    }

    func Start(startTime: Double)
    {
        guard !isStarted && positions.count > 1 else {
//            print("not enough positions")
            return
        }

        previousDeltaTime = 0.0
        currentPoint = 0
        currentOutboundFactor = 1.0
        if currentPoint < times.count - 1 {
            nextInboundFactor = (2.0 * times[currentPoint]) / (times[currentPoint] + times[currentPoint + 1])
        }
        else {
            nextInboundFactor = 1.0
        }
        currentPointStartTime = startTime
        isStarted = true
//        print("started")
    }
    
    func Stop()
    {
        isStarted = false
    }

    func Interpolate(currentTime: Double, position: ZVector2, tangent: ZVector2)
    {
        guard isStarted else {
            return
        }

        var deltaTime = currentTime - currentPointStartTime
//        print("CT: ", currentTime)
        while (deltaTime >= times[currentPoint])
        {
            previousDeltaTime = times[currentPoint]
            deltaTime -= previousDeltaTime
            currentPointStartTime += previousDeltaTime
            currentPoint += 1
            if currentPoint == times.count
            {
                position.set(positions[positions.count - 1])
                tangent.set(tangents[tangents.count - 1])
                isStarted = false
                return
            }
            currentOutboundFactor = (2.0 * times[currentPoint]) / (previousDeltaTime + times[currentPoint])
            if currentPoint < times.count - 1 {
                nextInboundFactor = (2.0 * times[currentPoint]) / (times[currentPoint] + times[currentPoint + 1])
            }
            else {
                nextInboundFactor = 1.0
            }
        }

        interpolate(
            t: deltaTime / times[currentPoint],
            position1: positions[currentPoint],
            tangent1: tangents[currentPoint],
            tangent1Factor: currentOutboundFactor,
            position2: positions[currentPoint + 1],
            tangent2: tangents[currentPoint + 1],
            tangent2Factor: nextInboundFactor,
            outputPosition: position,
            outputVelocity: tangent)
    }

    fileprivate func interpolate(
        t: Double,
        position1: ZVector2,
        tangent1: ZVector2,
        tangent1Factor: Double,
        position2: ZVector2,
        tangent2: ZVector2,
        tangent2Factor: Double,
        outputPosition: ZVector2,
        outputVelocity: ZVector2)
    {
        let tt = t * t
        let t2 = 2.0 * t
        let u = 1.0 - t
        let uu = u * u
        let p00 = (1.0 + t2) * uu
        let p10 = t * uu
        let p01 = tt * (3.0 - t2)
        let p11 = tt * (t - 1.0)
        outputPosition.set(
            interpolateCubicHermite(p00, p10, p01, p11,
                position1.getX(), tangent1.getX() * tangent1Factor, position2.getX(), tangent2.getX() * tangent2Factor),
            interpolateCubicHermite(p00, p10, p01, p11,
                position1.getY(), tangent1.getY() * tangent1Factor, position2.getY(), tangent2.getY() * tangent2Factor))

        let v00 = ( 6.0 * tt) - ( 6.0 * t)
        let v10 = ( 3.0 * tt) - ( 4.0 * t) + 1.0
        let v01 = (-6.0 * tt) + ( 6.0 * t)
        let v11 = ( 3.0 * tt) - ( 2.0 * t)
        outputVelocity.set(
            interpolateCubicHermite(v00, v10, v01, v11,
                position1.getX(), tangent1.getX() * tangent1Factor, position2.getX(), tangent2.getX() * tangent2Factor),
            interpolateCubicHermite(v00, v10, v01, v11,
                position1.getY(), tangent1.getY() * tangent1Factor, position2.getY(), tangent2.getY() * tangent2Factor))
    }

    private func interpolateCubicHermite(
        _ c00: Double, _ c10: Double, _ c01: Double, _ c11: Double,
        _ p1: Double, _ t1: Double, _ p2: Double, _ t2: Double) -> Double
    {
        return (c00 * p1) + (c10 * t1) + (c01 * p2) + (c11 * t2)
    }

}

func BuildCubicHermiteSpline(keyframes: [PathKeyFrame]) -> ZCubicHermiteSpline
{
    var positions = [ZVector2]()
    var tangents = [ZVector2]()
    var times = [Double]()

    //calculate the tangents at each point
    var currentPosition = ZVector2(0.0, 0.0)
    positions.append(currentPosition)
//    print("P: ", currentPosition)

    var nextPosition = ZVector2(keyframes[0].x, keyframes[0].y)
    tangents.append(
        calculateTangent(
            position0: currentPosition,
            time01: keyframes[0].time,
            position1: currentPosition,
            orientation1: 0.0,
            time12: keyframes[0].time,
            position2: nextPosition))

    var previousPosition = currentPosition
    currentPosition = nextPosition
    for i in 0...keyframes.count-1
    {
        //time to reach next position
        times.append(keyframes[i].time)
        
        //next position
//        print("T: ", keyframes[i].time, "P: ", currentPosition)
        positions.append(currentPosition)

        //next tangent
        let nextTime: Double
        if i < keyframes.count - 1 {
            nextTime = keyframes[i + 1].time
            nextPosition = ZVector2(keyframes[i + 1].x, keyframes[i + 1].y)
        }
        else {
            nextTime = keyframes[i].time
            nextPosition = currentPosition
        }
        tangents.append(
            calculateTangent(
                position0: previousPosition,
                time01: keyframes[i].time,
                position1: currentPosition,
                orientation1: DEG2RAD * keyframes[i].orientation,
                time12: nextTime,
                position2: nextPosition))

        previousPosition = currentPosition
        currentPosition = nextPosition
    }

    return ZCubicHermiteSpline(
        positions: positions,
        tangents: tangents,
        times: times)
}

fileprivate func calculateTangent(
    position0: ZVector2,
    time01: Double,
    position1: ZVector2,
    orientation1: Double,
    time12: Double,
    position2: ZVector2) -> ZVector2
{
    //Catmull Rom
    /*
    let tangentX = (2.0 * (position2.getX() - position0.getX())) / (time01 + time12)
    let tangentY = (2.0 * (position2.getY() - position0.getY())) / (time01 + time12)
    let sin1 = sin(orientation1)
    let cos1 = cos(orientation1)
    let tangentMagnitude = abs((tangentX * sin1) + (tangentY * cos1))
    return ZVector2(
            sin1 * tangentMagnitude,
            cos1 * tangentMagnitude)
     */
    let time02 = time01 + time12
    let tangentX =
        ((position1.getX() - position0.getX()) / time01) -
        ((position2.getX() - position0.getX()) / time02) +
        ((position2.getX() - position1.getX()) / time12)
    let tangentY =
        ((position1.getY() - position0.getY()) / time01) -
        ((position2.getY() - position0.getY()) / time02) +
        ((position2.getY() - position1.getY()) / time12)

    let sin1 = sin(orientation1)
    let cos1 = cos(orientation1)
    let tangentMagnitude = abs((tangentX * sin1) + (tangentY * cos1))
    return ZVector2(
            sin1 * tangentMagnitude,
            cos1 * tangentMagnitude)
}

