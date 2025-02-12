import os
import math
from common.realtime import sec_since_boot, DT_MDL
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LAT
from selfdrive.controls.lib.lane_planner import LanePlanner
from selfdrive.config import Conversions as CV
from common.params import Params
from common.numpy_fast import interp
import cereal.messaging as messaging
from cereal import log
from selfdrive.car.hyundai.interface import CarInterface
import common.log as trace1

from selfdrive.car.hyundai.values import Buttons

import common.MoveAvg as ma

LaneChangeState = log.PathPlan.LaneChangeState
LaneChangeDirection = log.PathPlan.LaneChangeDirection


LOG_MPC = os.environ.get('LOG_MPC', True)

LANE_CHANGE_SPEED_MIN = 40 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.PathPlan.Desire.none,
    LaneChangeState.preLaneChange: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeDone: log.PathPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.PathPlan.Desire.none,
    LaneChangeState.preLaneChange: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.PathPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.PathPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeDone: log.PathPlan.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.PathPlan.Desire.none,
    LaneChangeState.preLaneChange: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.PathPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.PathPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeDone: log.PathPlan.Desire.laneChangeRight,
  },
}


def calc_states_after_delay(states, v_ego, steer_angle, curvature_factor, steer_ratio, delay):
  states[0].x = v_ego * delay
  states[0].psi = v_ego * curvature_factor * math.radians(steer_angle) / steer_ratio * delay
  return states


class PathPlanner():
  def __init__(self, CP):
    self.LP = LanePlanner()

    self.last_cloudlog_t = 0
    self.steer_rate_cost = CP.steerRateCost
    self.steerRatio = CP.steerRatio    
    

    self.setup_mpc()
    self.solution_invalid_cnt = 0

    self.steerRatio_last = 0

    self.params = Params()

    # Lane change 
    self.lane_change_enabled = self.params.get('LaneChangeEnabled') == b'1'
    self.lane_change_auto_delay = self.params.get_OpkrAutoLanechangedelay()  #int( self.params.get('OpkrAutoLanechangedelay') )

    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_run_timer = 0.0
    self.lane_change_wait_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.prev_one_blinker = False
 
    # atom
    self.trPATH = trace1.Loger("path")
    self.trLearner = trace1.Loger("Learner")
    self.trpathPlan = trace1.Loger("pathPlan")

    self.atom_timer_cnt = 0
    self.atom_steer_ratio = None
    self.atom_sr_boost_bp = [0., 0.]
    self.atom_sr_boost_range = [0., 0.]

    self.carParams_valid = False

    self.m_avg = ma.MoveAvg()

  def limit_ctrl(self, value, limit, offset ):
      p_limit = offset + limit
      m_limit = offset - limit
      if value > p_limit:
          value = p_limit
      elif  value < m_limit:
          value = m_limit
      return value

  def limit_ctrl1(self, value, limit1, limit2, offset ):
      p_limit = offset + limit1
      m_limit = offset - limit2
      if value > p_limit:
          value = p_limit
      elif  value < m_limit:
          value = m_limit
      return value     

  def setup_mpc(self):
    self.libmpc = libmpc_py.libmpc
    self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, self.steer_rate_cost)

    self.mpc_solution = libmpc_py.ffi.new("log_t *")
    self.cur_state = libmpc_py.ffi.new("state_t *")
    self.cur_state[0].x = 0.0
    self.cur_state[0].y = 0.0
    self.cur_state[0].psi = 0.0
    self.cur_state[0].delta = 0.0

    self.angle_steers_des = 0.0
    self.angle_steers_des_mpc = 0.0
    self.angle_steers_des_prev = 0.0
    self.angle_steers_des_time = 0.0


  def atom_tune( self, v_ego_kph, sr_value,  atomTuning ):  # 조향각에 따른 변화.
    self.sr_KPH = atomTuning.sRKPH
    self.sr_BPV = atomTuning.sRBPV
    self.sr_steerRatioV = atomTuning.sRsteerRatioV
    self.sr_SteerRatio = []

    nPos = 0
    for steerRatio in self.sr_BPV:  # steerRatio
      self.sr_SteerRatio.append( interp( sr_value, steerRatio, self.sr_steerRatioV[nPos] ) )
      nPos += 1
      if nPos > 20:
        break

    steerRatio = interp( v_ego_kph, self.sr_KPH, self.sr_SteerRatio )

    return steerRatio

  def atom_actuatorDelay( self, v_ego_kph, sr_value, atomTuning ):
    self.sr_KPH = atomTuning.sRKPH
    self.sr_BPV = atomTuning.sRBPV
    self.sr_ActuatorDelayV = atomTuning.sRsteerActuatorDelayV
    self.sr_ActuatorDelay = []

    nPos = 0
    for steerRatio in self.sr_BPV:
      self.sr_ActuatorDelay.append( interp( sr_value, steerRatio, self.sr_ActuatorDelayV[nPos] ) )
      nPos += 1
      if nPos > 10:
        break

    actuatorDelay = interp( v_ego_kph, self.sr_KPH, self.sr_ActuatorDelay )

    return actuatorDelay


  def  atom_steer( self, sr_value, sr_up, sr_dn ):
    delta =  sr_value - self.steerRatio_last

    sr_up = min( abs(delta), sr_up )
    sr_dn = min( abs(delta), sr_dn )
    steerRatio = self.steerRatio_last
    if delta > 0:
      steerRatio += sr_up
    elif delta < 0:
      steerRatio -= sr_dn


    self.steerRatio_last = steerRatio
    return steerRatio

  def update(self, sm, pm, CP, VM):
    self.atom_timer_cnt += 1
    if self.atom_timer_cnt > 1000:
      self.atom_timer_cnt = 0

    cruiseState  = sm['carState'].cruiseState
    leftBlindspot = sm['carState'].leftBlindspot
    rightBlindspot = sm['carState'].rightBlindspot


    

    lateralsRatom = CP.lateralsRatom
    atomTuning = CP.atomTuning

    #if atomTuning is None or lateralsRatom is None:
    #print('carparams={} steerRatio={}  carParams_valid={}'.format(sm.updated['carParams'], sm['carParams'].steerRatio, self.carParams_valid ) )

    if not self.carParams_valid and sm['carParams'].steerRatio:  # sm.updated['carParams']:
      self.carParams_valid = True

    if self.carParams_valid:
      lateralsRatom = sm['carParams'].lateralsRatom
      atomTuning = sm['carParams'].atomTuning


    v_ego = sm['carState'].vEgo
    angle_steers = sm['carState'].steeringAngle
    steeringPressed  = sm['carState'].steeringPressed
    steeringTorque = sm['carState'].steeringTorque
    active = sm['controlsState'].active
    model_sum = sm['controlsState'].modelSum

    v_ego_kph = v_ego * CV.MS_TO_KPH

    self.steerRatio = sm['liveParameters'].steerRatio
    angle_offset = sm['liveParameters'].angleOffset
    angleOffsetAverage = sm['liveParameters'].angleOffsetAverage
    stiffnessFactor = sm['liveParameters'].stiffnessFactor

    #if (self.atom_timer_cnt % 100) == 0:
    #  str_log3 = 'angleOffset={:.1f} angleOffsetAverage={:.3f} steerRatio={:.2f} stiffnessFactor={:.3f} '.format( angle_offset, angleOffsetAverage, self.steerRatio, stiffnessFactor )
    #  self.trLearner.add( 'LearnerParam {}  carParams={}'.format( str_log3, self.carParams_valid ) )       

    if lateralsRatom.learnerParams:
      pass
    else:
      # atom
      if self.carParams_valid:
        self.steer_rate_cost = sm['carParams'].steerRateCost   
        self.steerRatio = sm['carParams'].steerRatio        
      else:
        self.steer_rate_cost = CP.steerRateCost
        self.steerRatio = CP.steerRatio
   

      #xp = [-5,0,5]
      #fp = [0.4, 0.7, 0.4] 
      #self.steer_rate_cost = interp( angle_steers, xp, fp )
      steerRatio = self.atom_tune( v_ego_kph, angle_steers, atomTuning )
      self.steerRatio = self.atom_steer( steerRatio, 2, 1)

    #actuatorDelay = CP.steerActuatorDelay
    steerActuatorDelay = self.atom_actuatorDelay( v_ego_kph, angle_steers, atomTuning )

    # Run MPC
    self.angle_steers_des_prev = self.angle_steers_des_mpc
    VM.update_params(stiffnessFactor, self.steerRatio )  
    curvature_factor = VM.curvature_factor(v_ego)



    self.LP.parse_model(sm['model'])

    # Lane change logic
    one_blinker = sm['carState'].leftBlinker != sm['carState'].rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if sm['carState'].leftBlinker:
      self.lane_change_direction = LaneChangeDirection.left
    elif sm['carState'].rightBlinker:
      self.lane_change_direction = LaneChangeDirection.right

    if (not active) or (self.lane_change_run_timer > LANE_CHANGE_TIME_MAX) or (not one_blinker) or (not self.lane_change_enabled):
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      l_poly = self.LP.l_poly[3]
      r_poly = self.LP.r_poly[3]
      c_prob = l_poly + r_poly
      torque_applied = steeringPressed and \
                        ((steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or \
                          (steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

      blindspot_detected = ((leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                            (rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))                          

      lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob

      # State transitions
      # off
      if cruiseState.cruiseSwState == Buttons.CANCEL:
        self.lane_change_state = LaneChangeState.off
        self.lane_change_ll_prob = 1.0
        self.lane_change_wait_timer = 0

      elif self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.lane_change_wait_timer = 0

      # pre
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        self.lane_change_wait_timer += DT_MDL

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
        elif not blindspot_detected and (torque_applied or (self.lane_change_auto_delay and self.lane_change_wait_timer > self.lane_change_auto_delay)):
          self.lane_change_state = LaneChangeState.laneChangeStarting

      # starting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        xp = [40,50,60,70]
        fp2 = [0.2,0.6,1.2,1.5]
        lane_time = interp( v_ego_kph, xp, fp2 )        
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - lane_time*DT_MDL, 0.0)
        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # finishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
        if self.lane_change_ll_prob > 0.99  and  abs(c_prob) < 0.3:
          self.lane_change_state = LaneChangeState.laneChangeDone

      # done
      elif self.lane_change_state == LaneChangeState.laneChangeDone:
        if not one_blinker:
          self.lane_change_state = LaneChangeState.off



    if self.lane_change_state in [LaneChangeState.off, LaneChangeState.preLaneChange]:
      self.lane_change_run_timer = 0.0
    else:
      self.lane_change_run_timer += DT_MDL
      


    self.prev_one_blinker = one_blinker

    desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Turn off lanes during lane change
    if desire == log.PathPlan.Desire.laneChangeRight or desire == log.PathPlan.Desire.laneChangeLeft:
      self.LP.l_prob *= self.lane_change_ll_prob
      self.LP.r_prob *= self.lane_change_ll_prob
    self.LP.update_d_poly(v_ego , lateralsRatom.cameraOffset )

    # account for actuation delay
    self.cur_state = calc_states_after_delay(self.cur_state, v_ego, angle_steers - angle_offset, curvature_factor, VM.sR, steerActuatorDelay )

    v_ego_mpc = max(v_ego, 5.0)  # avoid mpc roughness due to low speed
    self.libmpc.run_mpc(self.cur_state, self.mpc_solution,
                        list(self.LP.l_poly), list(self.LP.r_poly), list(self.LP.d_poly),
                        self.LP.l_prob, self.LP.r_prob, curvature_factor, v_ego_mpc, self.LP.lane_width)



    # reset to current steer angle if not active or overriding
    if active:
      delta_desired = self.mpc_solution[0].delta[1]
      rate_desired = math.degrees(self.mpc_solution[0].rate[0] * VM.sR)
    else:
      delta_desired = math.radians(angle_steers - angle_offset) / VM.sR
      rate_desired = 0.0

    self.cur_state[0].delta = delta_desired

    self.angle_steers_des_mpc = float(math.degrees(delta_desired * VM.sR) + angle_offset)
    org_angle_steers_des = self.angle_steers_des_mpc

    # atom
    if steeringPressed:
      delta_steer = org_angle_steers_des - angle_steers
      xp = [-255,0,255]
      fp2 = [5,0,5]
      limit_steers = interp( steeringTorque, xp, fp2 )
      if steeringTorque < 0:  # right
        if delta_steer > 0:
          self.angle_steers_des_mpc = self.limit_ctrl( org_angle_steers_des, limit_steers, angle_steers )
      elif steeringTorque > 0:  # left
        if delta_steer < 0:
          self.angle_steers_des_mpc = self.limit_ctrl( org_angle_steers_des, limit_steers, angle_steers )

    elif v_ego_kph < 15:  # 30
      xp = [3,10,15]
      fp2 = [3,5,7]
      limit_steers = interp( v_ego_kph, xp, fp2 )
      self.angle_steers_des_mpc = self.limit_ctrl( org_angle_steers_des, limit_steers, angle_steers )
    elif v_ego_kph > 60: 
      pass
    elif abs(angle_steers) > 10: # angle steer > 10
      # 2.
      xp = [-10,-5,0,5,10]    # 5  10=>28 15=>35, 30=>52
      fp1 = [3,8,10,20,10]    # +
      fp2 = [10,20,10,8,3]    # -
      limit_steers1 = interp( model_sum, xp, fp1 )  # +
      limit_steers2 = interp( model_sum, xp, fp2 )  # -
      self.angle_steers_des_mpc = self.limit_ctrl1( org_angle_steers_des, limit_steers1, limit_steers2, angle_steers )
      
    
    delta_steer = self.angle_steers_des_mpc - angle_steers
    ANGLE_LIMIT = 8
    if delta_steer > ANGLE_LIMIT:
      p_angle_steers = angle_steers + ANGLE_LIMIT
      self.angle_steers_des_mpc = p_angle_steers
    elif delta_steer < -ANGLE_LIMIT:
      m_angle_steers = angle_steers - ANGLE_LIMIT
      self.angle_steers_des_mpc = m_angle_steers
    

    

    #  Check for infeasable MPC solution
    mpc_nans = any(math.isnan(x) for x in self.mpc_solution[0].delta)
    t = sec_since_boot()
    if mpc_nans:
      self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, self.steer_rate_cost)
      self.cur_state[0].delta = math.radians(angle_steers - angle_offset) / VM.sR
      

      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    #self.trPATH.add( 'mpc_nans ={}  libmpc  steer_rate_cost={}  delta={}   angle_steers={}'.format( mpc_nans, self.steer_rate_cost, self.cur_state[0].delta, angle_steers ) )

    if self.mpc_solution[0].cost > 20000. or mpc_nans:   # TODO: find a better way to detect when MPC did not converge
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0
    plan_solution_valid = self.solution_invalid_cnt < 3

    plan_send = messaging.new_message('pathPlan')
    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState', 'liveParameters', 'model'])
    plan_send.pathPlan.laneWidth = float(self.LP.lane_width)
    plan_send.pathPlan.dPoly = [float(x) for x in self.LP.d_poly]
    plan_send.pathPlan.lPoly = [float(x) for x in self.LP.l_poly]
    plan_send.pathPlan.lProb = float(self.LP.l_prob)
    plan_send.pathPlan.rPoly = [float(x) for x in self.LP.r_poly]
    plan_send.pathPlan.rProb = float(self.LP.r_prob)

    plan_send.pathPlan.angleSteers = float(self.angle_steers_des_mpc)
    plan_send.pathPlan.rateSteers = float(rate_desired)
    plan_send.pathPlan.angleOffset = float(angleOffsetAverage)
    plan_send.pathPlan.mpcSolutionValid = bool(plan_solution_valid)
    plan_send.pathPlan.paramsValid = bool(sm['liveParameters'].valid)

    plan_send.pathPlan.desire = desire
    plan_send.pathPlan.laneChangeState = self.lane_change_state
    plan_send.pathPlan.laneChangeDirection = self.lane_change_direction
    plan_send.pathPlan.steerRatio = self.steerRatio
    plan_send.pathPlan.steerActuatorDelay = steerActuatorDelay
    pm.send('pathPlan', plan_send)

    #if self.solution_invalid_cnt > 0:
    #  str_log3 = 'v_ego_kph={:.1f} angle_steers_des_mpc={:.1f} angle_steers={:.1f} solution_invalid_cnt={:.0f} mpc_solution={:.1f}/{:.0f}'.format( v_ego_kph, self.angle_steers_des_mpc, angle_steers, self.solution_invalid_cnt, self.mpc_solution[0].cost, mpc_nans )
    #  self.trpathPlan.add( 'pathPlan {}  LOG_MPC={}'.format( str_log3, LOG_MPC ) )


    if LOG_MPC:
      dat = messaging.new_message('liveMpc')
      dat.liveMpc.x = list(self.mpc_solution[0].x)
      dat.liveMpc.y = list(self.mpc_solution[0].y)
      dat.liveMpc.psi = list(self.mpc_solution[0].psi)
      dat.liveMpc.delta = list(self.mpc_solution[0].delta)
      dat.liveMpc.cost = self.mpc_solution[0].cost
      pm.send('liveMpc', dat)
