#!/usr/bin/env python
import rospy
from vkf6 import VKF
import numpy as np
import math
import threading
from tf.transformations import euler_from_quaternion
import tf2_ros as tf2
import tf2_msgs.msg
from threading import Lock
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_msgs.msg import Int8
from std_msgs.msg import Bool
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
  H[0][2]=1
  return H

def observe_w(x_in):
  obs=np.zeros(1)
  obs[0]=x_in[3]
  return obs

def observe_w_jacobian(x_in):
  H=np.zeros([1,4])
  H[0][3]=1
  return H
 
def get_driving_force_speed_control(w_cmd, w, u, v, r, delta):
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


def get_driving_force_current_control(I_cmd, u, v, r, delta):
  #Define Constants
  m = 4.9565
  C_af1 = 30
  C_af2 = 1
  C_ar1 = 50
  C_ar2 = 1
  l_f = 0.203
  l_r = 0.1

  # k_tau = 0.4454 #torque constant
  k_tau = 0.4 #torque constant
  C1f = 6.0897 #friction constant 1
  C2f = 10.5921 #friction constant 2

  #Calculate Intermediate parameters
  front_mass_ratio = l_r/(l_f+l_r)
  v_f = v + l_f*r
  v_r = v - l_r*r
  column = np.dot(rotation_matrix(-delta), np.array([u, v_f]))
  u_wf = column[0]
  v_wf = column[1]
  u_wr = u
  v_wr = v_r
  a_f = np.arctan2(v_wf,(u_wf**2 + 0.005)**0.5)
  a_r = np.arctan2(v_wr,(u_wr**2 + 0.005)**0.5)
  #Calculate Force
  
  F = np.zeros(4)
  F[0] = front_mass_ratio*(k_tau*I_cmd - C1f*np.tanh(C2f*u))
  pub_debug.publish(F[0])
  F[1] = (1.0-front_mass_ratio)*(k_tau*I_cmd - C1f*np.tanh(C2f*u))
  F[2] = -C_af1*np.tanh(C_af2*a_f)
  F[3] = -C_ar1*np.tanh(C_ar2*a_r)

  return F

def iterate_x(x_in, timestep, inputs):
  global drive_mode
  global pub_debug
  # Vehicle Constants
  m = 4.9565
  I_zz = 0.11
  l_f = 0.203
  l_r = 0.1
  C_w = 3.41
  # Input variables
  # w_cmd = inputs[0]

  I_cmd = inputs[0]
  steering_angle = inputs[1]
  # Read from previous state variables
  u_prev = x_in[0]
  v_prev = x_in[1]
  r_prev = x_in[2]
  w_prev = x_in[3]

  #Read forces
  F = [0, 0, 0, 0]
  # if drive_mode == 0:
  #   F = get_driving_force_speed_control(w_cmd, w_prev, u_prev, v_prev, r_prev, steering_angle)

  # elif drive_mode == 1:
  #   F = get_driving_force_current_control(I_cmd, u_prev, v_prev, r_prev, steering_angle)
  F = get_driving_force_current_control(I_cmd, u_prev, v_prev, r_prev, steering_angle)
  
  F_xwf = F[0]
  F_xwr = F[1]
  F_ywf = F[2]
  F_ywr = F[3]


  #Calculate derivatives
  u_dot = (((math.cos(steering_angle)*F_xwf)+F_xwr-(math.sin(steering_angle)*F_ywf))/m)+(v_prev*r_prev)
  v_dot = (((math.sin(steering_angle)*F_xwf)+F_ywr+(math.cos(steering_angle)*F_ywf))/m)-(u_prev*r_prev)
  r_dot = ((l_f*((math.sin(steering_angle)*F_xwf)+(math.cos(steering_angle)*F_ywf)))-(l_r*F_ywr))/I_zz
  #w_dot is different depending on current control or speed control

  # w_dot = 0
  # if drive_mode == 0:
  #   w_dot = C_w*(w_cmd - w_prev)
  # elif drive_mode == 1:
  w_dot = u_dot

  #Calculate updates
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
  global pub_state2
  global pub_state3
  global pub_debug
  global imu_r
  global new_imu_measurement
  global new_w_measurement
  global varray
  global u_slam
  global v_slam
  global r_slam
  global x_prev
  global y_prev
  global h_prev
  global time_prev
  global new_slam_uvr_update
  global slam_counter
  global drive_mode
  global current_cmd

  slam_counter = 0
  x_prev = 0
  y_prev = 0
  h_prev = 0
  u_slam = 0
  v_slam = 0
  r_slam = 0
  drive_mode  = 0
  current_cmd = 0


  time_prev = rospy.get_rostime()
  time_prev = time_prev.to_sec()
  varray = np.zeros(1)
  new_imu_measurement = False
  new_w_measurement = False
  new_slam_uvr_update = False
  mutex = Lock()
  estimator_rate = 50
  pub_state = rospy.Publisher('/state_estimator/states', velocity_states, queue_size=10)
  pub_state2 = rospy.Publisher('/state_estimator_freq', Int8, queue_size=10)
  pub_state3 = rospy.Publisher('/slam_u', Float64, queue_size=10)
  pub_debug = rospy.Publisher('/F_predicted', Float64, queue_size=10)

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
  c = c*1.4

  state_estimator = VKF(4, 2, np.zeros(4), np.zeros(2), c, 0.8, 1.0, 2.0, iterate_x)

def update_w_cmd(w_cmd):
  global mutex
  global state_estimator

  mutex.acquire()
  w_cmd = (w_cmd.data)/2272.0
  state_estimator.update_inputs([w_cmd],[0])

  mutex.release()

def update_current_cmd(I_cmd):
  global mutex
  global state_estimator

  mutex.acquire()
  state_estimator.update_inputs([I_cmd.state.current_motor],[0])

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

def update_drive_mode(data):
  print("Switching drive mode")
  global drive_mode
  drive_mode = data.data
  if drive_mode == 1:
    print("Switching to current control mode")
  elif drive_mode == 0:
    print("Switching to speed control mode")


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
  sensor_w = data.state.speed/2272.0
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
  global new_slam_uvr_update
  global u_slam
  global v_slam
  global r_slam
  global pub_state3

  mutex.acquire()
  
  d_time = rospy.Time.now() - last_predict_time
  last_predict_time += d_time
  state_estimator.predict(d_time.to_sec())
  states_now = state_estimator.get_state()
  if (new_slam_uvr_update):
    pub_state3.publish(u_slam)
    state_estimator.update([0],np.array([u_slam]),0.08)
    state_estimator.update([1],np.array([v_slam]),0.08)
    # state_estimator.update([2],np.array([r_slam]),0.13)
    new_slam_uvr_update = False

  if (new_imu_measurement):
    state_estimator.update([2],np.array([imu_r]),0.019)
    new_slam_uvr_update = False

  if (new_w_measurement):
    # state_estimator.update([3],np.array([sensor_w]),0.05)
    # print('w sensor updated')
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

def get_slam_uvr(data):
    global x_prev
    global y_prev
    global h_prev
    global time_prev

    global u_slam
    global v_slam
    global r_slam
    global new_slam_uvr_update
    global slam_counter


    for trans in data.transforms :
        if trans.header.frame_id == "map" or trans.header.frame_id == "/map":
            if trans.child_frame_id == "base_link" or trans.child_frame_id == "/base_link":
                now = trans.header.stamp.to_sec()
                dt = now - time_prev
                if dt > 0.02:
                  slam_counter = 0
                  pub_state2.publish(1) 
                  x_curr = trans.transform.translation.x
                  y_curr = trans.transform.translation.y
                  q = trans.transform.rotation

                  #get heading
                  euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
                  roll = euler[0]
                  pitch = euler[1]
                  yaw = euler[2]
                  h_curr = yaw-3.1415/2

                  #get uvr
                  dxdt = (x_curr-x_prev)/dt
                  dydt = (y_curr-y_prev)/dt
                  dhdt = (h_curr-h_prev)/dt

                  dxdy = np.array([dxdt, dydt])
                  rotmat_2d = np.array([[np.cos(-h_curr), -np.sin(-h_curr)],
                                  [np.sin(-h_curr), np.cos(-h_curr)]])
                  uv = rotmat_2d.dot(dxdy)

                  #update prevs
                  time_prev = now
                  x_prev = x_curr
                  y_prev = y_curr
                  h_prev = h_curr



                  #update uvr_slam
                  if(u_slam != uv[0]):
                    u_slam = uv[0]
                    new_slam_uvr_update = True
                  if(v_slam != uv[1]):
                    v_slam = uv[1]
                    new_slam_uvr_update = True
                  if(r_slam != dhdt):
                    r_slam = dhdt
                    new_slam_uvr_update = True


      





 
def main():
  global last_predict_time
  global last_imu_measurement_time
  global last_w_measurement_time
  global servo_offset
  print("hello")

  rospy.init_node('state_estimator_slam')
  definition()
  last_predict_time = rospy.Time.now()
  last_imu_measurement_time = last_predict_time
  last_w_measurement_time = last_predict_time
  if rospy.has_param('/roahm/servo_offset'):
    servo_offset = rospy.get_param('/roahm/servo_offset', 0.50) 
  else:
    rospy.logerr("No servo offset param")
  sub1 = rospy.Subscriber('/imu/data',Imu, update_r,queue_size=1) ## imu should not be subccribed since it is already getting updates from slam, which is imu + slam 
  sub2 = rospy.Subscriber('/vesc/sensors/core', VescStateStamped, update_w,queue_size=1)
  sub3 = rospy.Subscriber('/vesc/commands/servo/position', Float64, update_steering_angle,queue_size=1)
  # sub4 = rospy.Subscriber('/vesc/commands/motor/speed', Float64, update_w_cmd,queue_size=1)
  sub5 = rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, get_slam_uvr,queue_size=1)
  sub6 = rospy.Subscriber('/drive_mode', Int8, update_drive_mode,queue_size=1)
  sub7 = rospy.Subscriber('/vesc/sensors/core', Float64, update_current_cmd,queue_size=1)

  estimator = rospy.Timer(rospy.Duration(1.0/estimator_rate), general_update)

  rospy.spin()

if __name__ == "__main__":

  main()
