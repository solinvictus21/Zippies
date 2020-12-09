
import Foundation

let PI_2: Double = Double.pi / 2.0
let _2PI: Double = 2.0 * Double.pi
let RAD2DEG: Double = 180.0 / Double.pi
let DEG2RAD: Double = Double.pi / 180.0

func snapAngle(_ angle: Double) -> Double
{
    if (angle <= -Double.pi) {
        return angle + (2.0 * Double.pi)
    }
    else if (angle > Double.pi) {
        return angle - (2.0 * Double.pi)
    }
    return angle
}

func subtractAngles(_ a1: Double, _ a2: Double) -> Double
{
    return snapAngle(a1 - a2)
}

func addAngles(_ a1: Double, _ a2: Double) -> Double
{
    return snapAngle(a1 + a2)
}

fileprivate func bezierEaseInOut(
    _ t: Double,
    _ a: Double,
    _ b: Double) -> Double
{
  let u = 1.0 - t
  let u2 = u * u
  let t2 = t * t
  let t3 = t2 * t

  let u2t3 = 3.0 * u2 * t
  let ut23 = 3.0 * u * t2
  return (u2t3 * a) + (ut23 * b) + t3
}

fileprivate func bezierEaseInOut2(
    _ t: Double,
    _ a: Double,
    _ b: Double) -> Double
{
  let t2 = t * t
  let t3 = t2 * t
  let mt = 1-t
  let mt2 = mt * mt
  return (3*a*mt2*t) + (3*b*mt*t2) + t3
}

fileprivate func bezierEaseInOut3(
    _ t: Double,
    _ a: Double) -> Double
{
  let t2 = t * t
  let mt = 1-t
  return (a*2*mt*t) + t2
}

