#!/usr/bin/env python

import rospy
from vkf6 import VKF
import numpy as np
import math
from threading import Lock
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Header
from std_msgs.msg import Float64
from state_estimator.msg import velocity_states
def rotation_matrix(angle):
  m = np.array([[math.cos(angle), math.sin(angle)],[-math.sin(angle), math.cos(angle)]])
  return m

def observe_yaw(x_in):
  obs=np.zeros(1)
  obs[0]=x_in[2]
  return obs

def observe_yaw_jacobian(x_in):
  H=np.zeros([1,4])
  H[0][2]=1;
  return H

def observe_w(x_in):
  obs=np.zeros(1)
  obs[0]=x_in[3]
  return obs

def observe_w_jacobian(x_in):
  H=np.zeros([1,4])
  H[0][3]=1;
  return H
 
def get_driving_force(w_cmd, w, u, v, r, delta):
  #Define Constants
  m = 4.9565
  C_w = 3.41
  C_af1 = 30
  C_af2 = 1
  C_ar1 = 50
  C_ar2 = 1
  l_f = 0.203
  l_r = 0.1

  #Calculate Intermediate parameters
  dw = C_w*(w_cmd - u)
  front_mass_ratio = l_r/(l_f+l_r)
  v_f = v + l_f*r
  v_r = v - l_r*r
  column = np.dot(rotation_matrix(-delta), np.array([u, v_f]))
  u_wf = column[0]
  v_wf = column[1]
  u_wr = u
  v_wr = v_r
  a_f = np.arctan2(v_wf,u_wf+0.2)
  a_r = np.arctan2(v_wr,u_wr+0.2)
  #Calculate Force
  F = np.zeros(4)
  F[0] = front_mass_ratio*m*dw
  F[1] = (1-front_mass_ratio)*m*dw
  F[2] = -C_af1*math.tanh(C_af2*a_f)
  F[3] = -C_ar1*math.tanh(C_ar2*a_r)
  return F

def iterate_x(x_in, timestep, inputs):
  # Vehicle Constants
  m = 4.9565
  I_zz = 0.11
  l_f = 0.203
  l_r = 0.1
  C_w = 3.41
  # Input variables
  w_cmd = inputs[0]
  steering_angle = inputs[1]
  # Read from previous state variables
  u_prev = x_in[0];
  v_prev = x_in[1];
  r_prev = x_in[2];
  w_prev = x_in[3];
  w_dot = C_w*(w_cmd - w_prev)
  #Read forces
  F = get_driving_force(w_cmd, w_prev, u_prev, v_prev, r_prev, steering_angle)
  F_xwf = F[0]
  F_xwr = F[1]
  F_ywf = F[2]
  F_ywr = F[3]
  #Calculate derivatives
  u_dot = (((math.cos(steering_angle)*F_xwf)+F_xwr-(math.sin(steering_angle)*F_ywf))/m)+(v_prev*r_prev)
  v_dot = (((math.sin(steering_angle)*F_xwf)+F_ywr+(math.cos(steering_angle)*F_ywf))/m)-(u_prev*r_prev)
  r_dot = ((l_f*((math.sin(steering_angle)*F_xwf)+(math.cos(steering_angle)*F_ywf)))-(l_r*F_ywr))/I_zz
  x_out = np.zeros(4)
  x_out[0] = u_prev + (u_dot*timestep)
  x_out[1] = v_prev + (v_dot*timestep)
  x_out[2] = r_prev + (r_dot*timestep)
  x_out[3] = w_prev + (w_dot*timestep)
  return x_out 

def definition():
  global state_estimator 
  global mutex
  global estimator_rate
  global pub_state
  global imu_r
  global new_imu_measurement
  global new_w_measurement
  global varray
  varray = np.zeros(1)
  new_imu_measurement = False
  new_w_measurement = False
  mutex = Lock()
  estimator_rate = 50
  pub_state = rospy.Publisher('/state_estimator_open/states', velocity_states, queue_size=10)

  # Initial Covariance
  c = np.eye(4)
  c[0][0] = 0.0444
  c[0][1] = -0.0029
  c[0][2] = 0.0005
  c[0][3] = 0.0422
  c[1][0] = -0.0029
  c[1][1] = 0.0007
  c[1][2] = 0.0038
  c[1][3] = -0.0027
  c[2][0] = 0.0005
  c[2][1] = 0.0038
  c[2][2] = 0.0436
  c[2][3] = 0.0002
  c[3][0] = 0.0422
  c[3][1] = -0.0027
  c[3][2] = -0.0002
  c[3][3] = 0.0434

  state_estimator = VKF(4, 2, np.zeros(4), np.zeros(2), c, 0.8, 1.0, 2.0, iterate_x)

def update_w_cmd(w_cmd):
  global mutex
  global state_estimator
  mutex.acquire()
  w_cmd = (w_cmd.data)/2072.0
  if (w_cmd >= 0):
    state_estimator.update_inputs([w_cmd],[0])
  else:
    state_estimator.set_state([-0.001],[0])
    state_estimator.set_state([0],[1])
    state_estimator.set_state([0],[2])
    state_estimator.set_state([0],[3])
    state_estimator.update_inputs([0],[0])
    state_estimator.update_inputs([0],[1])

  mutex.release()

def update_steering_angle(delta):
  global mutex
  global state_estimator
  global servo_offset
  delta = delta.data
  delta = (delta-servo_offset)/0.8298*0.84
  states_now = state_estimator.get_state()
  mutex.acquire()
  if (states_now[0] >= 0):
    state_estimator.update_inputs([delta],[1])
  mutex.release()

def update_r(data):
  global new_imu_measurement
  global imu_r
  global mutex
  global last_imu_measurement_time
  global estimator_rate
  d_time = rospy.Time.now() - last_imu_measurement_time
  if ((d_time.to_sec() < 1.0/estimator_rate/2.0) or (abs(data.angular_velocity.z)>3)):
    return
  mutex.acquire()
  imu_r = data.angular_velocity.z
  new_imu_measurement = True
  mutex.release()

def update_w(data):
  global new_w_measurement
  global sensor_w
  global mutex
  global last_w_measurement_time
  global estimator_rate
  d_time = rospy.Time.now() - last_w_measurement_time
  if (d_time.to_sec() < 1.0/estimator_rate/2.0):
    return
  mutex.acquire()
  sensor_w = data.state.speed/2072.0
  new_w_measurement = True
  mutex.release()

def gen_state():
  global state_estimator
  global varray
  states = state_estimator.get_state()
  msg = velocity_states()
  msg.header = Header()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = 'base_link' 
  if np.size(varray) < 10:
    varray = np.append(varray,states[1])
    msg.v = states[1]
  else:
    varray = np.append(varray,states[1])
    varray = np.delete(varray,0)
    msg.v = np.mean(varray)
  if states[0] < 0:
    msg.u = 0
  else:  
    msg.u = states[0]
  msg.r = states[2]
  msg.w = states[3]
  return msg

def general_update(data):
  global last_predict_time
  global mutex
  global state_estimator
  global pub_state
  global imu_r
  global new_imu_measurement 
  global new_w_measurement
  mutex.acquire()
  
  d_time = rospy.Time.now() - last_predict_time
  last_predict_time += d_time
  state_estimator.predict(d_time.to_sec())
  states_now = state_estimator.get_state()
  if (new_imu_measurement):
    #state_estimator.update(imu_r,observe_yaw,observe_yaw_jacobian,0.05)
    state_estimator.update([2],np.array([imu_r]),0.05)
    new_imu_measurement = False
  if (new_w_measurement):
    #state_estimator.update(sensor_w,observe_w,observe_w_jacobian,0.05)
    state_estimator.update([3],np.array([sensor_w]),0.05)
    new_w_measurement = False
  if (states_now[0] < 0):
    state_estimator.set_state([-0.001],[0])
    state_estimator.set_state([0],[1])
    state_estimator.set_state([0],[2])
    state_estimator.set_state([0],[3])
    state_estimator.update_inputs([0],[0])
    state_estimator.update_inputs([0],[1])

  pub_state.publish(gen_state())
  mutex.release() 
 
def main():
  global last_predict_time
  global last_imu_measurement_time
  global last_w_measurement_time
  global servo_offset
  rospy.init_node('state_estimator')
  last_predict_time = rospy.Time.now()
  last_imu_measurement_time = last_predict_time
  last_w_measurement_time = last_predict_time
  servo_offset = rospy.get_param('/roahm/servo_offset', 0.50)
  sub1 = rospy.Subscriber('/imu/data',Imu, update_r,queue_size=100)
  sub2 = rospy.Subscriber('/vesc/sensors/core', VescStateStamped, update_w,queue_size=100)
  sub3 = rospy.Subscriber('/vesc/commands/servo/position', Float64, update_steering_angle,queue_size=100)
  sub4 = rospy.Subscriber('/vesc/commands/motor/speed', Float64, update_w_cmd,queue_size=100)
  estimator = rospy.Timer(rospy.Duration(1.0/estimator_rate), general_update)
  rospy.spin()

if __name__ == "__main__":
  definition()
  main()
