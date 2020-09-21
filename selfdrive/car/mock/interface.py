#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.swaglog import cloudlog
import cereal.messaging as messaging
from selfdrive.car import gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase

# mocked car interface to work with chffrplus
TS = 0.01  # 100Hz
YAW_FR = 0.2 # ~0.8s time constant on yaw rate filter
# low pass gain
LPG = 2 * 3.1415 * YAW_FR * TS / (1 + 2 * 3.1415 * YAW_FR * TS)


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    cloudlog.debug("Using Mock Car Interface")

    # TODO: subscribe to phone sensor
    self.sensor = messaging.sub_sock('sensorEvents')
    self.gps = messaging.sub_sock('gpsLocation')

    self.speed = 0.
    self.prev_speed = 0.
    self.yaw_rate = 0.
    self.yaw_rate_meas = 0.

  @staticmethod
  def compute_gb(accel, speed):
    return accel

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)
    ret.carName = "mock"
    ret.safetyModel = car.CarParams.SafetyModel.noOutput
    ret.mass = 1700.
    ret.rotationalInertia = 2500.
    ret.wheelbase = 2.70
    ret.centerToFront = ret.wheelbase * 0.5
    ret.steerRatio = 13. # reasonable
    ret.tireStiffnessFront = 1e6    # very stiff to neglect slip
    ret.tireStiffnessRear = 1e6     # very stiff to neglect slip



    ret.atomTuning.cvKPH    = [0.] 
    ret.atomTuning.cvBPV    = [[150., 255.]]  # CV
    ret.atomTuning.cvsMaxV  = [[255., 230.]]
    ret.atomTuning.cvsdUpV  = [[3,3]]
    ret.atomTuning.cvsdDnV  = [[7,5]]
    
    ret.atomTuning.sRKPH     = [30, 40, 80]   # Speed  kph
    ret.atomTuning.sRBPV     = [[0.],      [0.],      [0.]     ]
    ret.atomTuning.sRlqrkiV      = [[0.005],   [0.015],   [0.02]   ]
    ret.atomTuning.sRlqrscaleV   = [[2000],    [1900.0],  [1850.0] ]
    ret.atomTuning.sRpidKiV      = [[0.02,0.01,0.02],[0.03,0.02,0.03],[0.03,0.02,0.03]]
    ret.atomTuning.sRpidKpV      = [[0.20,0.15,0.20],[0.25,0.20,0.25],[0.25,0.20,0.25]]
    ret.atomTuning.sRsteerRatioV = [[13.95,13.85,13.95],[13.95,13.85,13.95],[13.95,13.85,13.95]]
    ret.atomTuning.sRsteerActuatorDelayV = [[0.25,0.5,0.25],[0.25,0.8,0.25],[0.25,0.8,0.25]]
  
    ret.lateralsRatom.deadzone = 0.1
    ret.lateralsRatom.steerOffset = 0
    ret.lateralsRatom.cameraOffset = 0
    ret.steerRateCost = 0.4
    ret.steerLimitTimer = 0.8

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # get basic data from phone and gps since CAN isn't connected
    sensors = messaging.recv_sock(self.sensor)
    if sensors is not None:
      for sensor in sensors.sensorEvents:
        if sensor.type == 4:  # gyro
          self.yaw_rate_meas = -sensor.gyro.v[0]

    gps = messaging.recv_sock(self.gps)
    if gps is not None:
      self.prev_speed = self.speed
      self.speed = gps.gpsLocation.speed

    # create message
    ret = car.CarState.new_message()

    # speeds
    ret.vEgo = self.speed
    ret.vEgoRaw = self.speed
    a = self.speed - self.prev_speed

    ret.aEgo = a
    ret.brakePressed = a < -0.5

    ret.standstill = self.speed < 0.01
    ret.wheelSpeeds.fl = self.speed
    ret.wheelSpeeds.fr = self.speed
    ret.wheelSpeeds.rl = self.speed
    ret.wheelSpeeds.rr = self.speed

    self.yawRate = LPG * self.yaw_rate_meas + (1. - LPG) * self.yaw_rate
    curvature = self.yaw_rate / max(self.speed, 1.)
    ret.steeringAngle = curvature * self.CP.steerRatio * self.CP.wheelbase * CV.RAD_TO_DEG

    events = []
    ret.events = events

    return ret.as_reader()

  def apply(self, c, sm, CP):
    # in mock no carcontrols
    return []
