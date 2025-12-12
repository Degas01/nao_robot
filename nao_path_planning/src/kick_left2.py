from naoqi import ALProxy
import motion as mot
import math
import time

def kick_left(motionProxy, postureProxy):
    # Enable stiffness and balance control
    motionProxy.setStiffnesses("Body", 1.0)
    motionProxy.wbEnable(True)
    motionProxy.wbFootState("Fixed", "Legs")
    motionProxy.wbEnableBalanceConstraint(True, "Legs")
    motionProxy.wbGoToBalance("RLeg", 1.0)
    motionProxy.wbFootState("Free", "LLeg")

    names = list()
    times = list()
    keys = list()

    names.append("LHipYawPitch")
    times.append([2.60000, 5.20000])
    keys.append([[-0.00456, [3, -0.86667, 0.00000], [3, 0.86667, 0.00000]], [0.01538, [3, -0.86667, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("LHipRoll")
    times.append([2.60000, 5.20000])
    keys.append([[0.13810, [3, -0.86667, 0.00000], [3, 0.86667, 0.00000]], [0.13964, [3, -0.86667, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("LHipPitch")
    times.append([2.60000, 5.00000, 5.20000])
    keys.append([[-0.28528, [3, -0.86667, 0.00000], [3, 0.80000, 0.00000]], [-0.56549, [3, -0.80000, 0.10950], [3, 0.06667, -0.00913]], [-0.64117, [3, -0.06667, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("LKneePitch")
    times.append([2.60000, 5.00000, 5.20000])
    keys.append([[1.02007, [3, -0.86667, 0.00000], [3, 0.80000, 0.00000]], [1.91812, [3, -0.80000, 0.00000], [3, 0.06667, 0.00000]], [0.97558, [3, -0.06667, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("LAnklePitch")
    times.append([2.60000, 5.00000, 5.20000])
    keys.append([[-0.69955, [3, -0.86667, 0.00000], [3, 0.80000, 0.00000]], [-0.46251, [3, -0.80000, -0.17606], [3, 0.06667, 0.01467]], [-0.12736, [3, -0.06667, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("LAnkleRoll")
    times.append([2.60000, 5.20000])
    keys.append([[0.00311, [3, -0.86667, 0.00000], [3, 0.86667, 0.00000]], [0.00311, [3, -0.86667, 0.00000], [3, 0.00000, 0.00000]]])

    try:
        motionProxy.angleInterpolationBezier(names, times, keys)
    except BaseException as err:
        print(err)

    # Return to standing posture
    motionProxy.wbEnable(False)
    postureProxy.goToPosture("StandInit", 0.5)

def main():
    NAO_IP = "127.0.0.1"  # Use localhost for Virtual Robot
    NAO_PORT = 50603

    motionProxy = ALProxy("ALMotion", NAO_IP, NAO_PORT)
    postureProxy = ALProxy("ALRobotPosture", NAO_IP, NAO_PORT)

    kick_left(motionProxy, postureProxy)

if __name__ == "__main__":
    main()
