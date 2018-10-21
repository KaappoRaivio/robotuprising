import ev3dev
from ev3dev.auto import *
from math import sin, cos, tan, atan2
from math import radians
from math import pi
import sys
import time
import paho.mqtt.client as mqtt

import precgyro


SERVER_ADDRESS = "192.168.43.1"
SERVER_ADDRESS = "iot.eclipse.org"

CORRECT_WITH_GYRO = True


ROTATION_ERROR_MARGIN = 5.0

# phi_dot_X : angular speed of motor X
# r : wheel radius
# gamma_gbl : heading direction, actually not needed for navigation
# theta_gbl : angle between first wheel and the heading direction
# R = robot center to wheel centre dist


class Driver:

    class OmniMotor(ev3dev.core.Motor):
        def __init__(self, output):
            super().__init__(output)

    def __init__(self, output_A, output_B, output_C, output_D):
        self.wheel_r = 58 / 2
        self.x_gbl = 0.0
        self.y_gbl = 0.0
        self.theta_gbl = radians(30.0)
        self.theta_dot_gbl = radians(0.0)
        self.gamma_gbl = radians(0.0)
        self.x_spd_loc = 50.0
        self.y_spd_loc = 50.0
        self.theta_loc = radians(0.0)
        self.theta_dot_loc = radians(0.0)

        self.R = 65

        self.alpha1 = radians(0.0)
        self.alpha2 = radians(120)
        self.alpha3 = radians(120 * 2)


        self.phi_dot_A = 0.0
        self.phi_dot_B = 0.0
        self.phi_dot_C = 0.0

        self.motor_A = self.OmniMotor(output_A)
        self.motor_B = self.OmniMotor(output_B)
        self.motor_C = self.OmniMotor(output_C)
        self.motor_grabber = self.OmniMotor(output_D)

        self.mA = self.motor_A
        self.mB = self.motor_B
        self.mC = self.motor_C
        self.mGrabber = self.motor_grabber

        print("count_per_rot of mA/B/C: ", self.mA.count_per_rot,
              self.mB.count_per_rot,
              self.mC.count_per_rot)

        self.mA.mm_per_count = self.wheel_r * 2*pi / self.mA.count_per_rot
        self.mB.mm_per_count = self.wheel_r * 2*pi / self.mB.count_per_rot
        self.mC.mm_per_count = self.wheel_r * 2*pi / self.mC.count_per_rot
        print("mm_per_count of mA/B/C: ", self.mA.mm_per_count,
                                          self.mB.mm_per_count,
                                          self.mC.mm_per_count)

        self.mA.count_per_rad = self.mA.count_per_rot / 2 / pi
        self.mB.count_per_rad = self.mB.count_per_rot / 2 / pi
        self.mC.count_per_rad = self.mC.count_per_rot / 2 / pi
        print("count_per_rad of mA/B/C: ", self.mA.count_per_rad,
                                           self.mB.count_per_rad,
                                           self.mC.count_per_rad)

        self.mA.mm_per_rad = self.mA.mm_per_count * self.mA.count_per_rad
        self.mB.mm_per_rad = self.mB.mm_per_count * self.mB.count_per_rad
        self.mC.mm_per_rad = self.mC.mm_per_count * self.mC.count_per_rad
        print("mm_per_rad of mA/B/C: ", self.mA.mm_per_rad,
                                        self.mB.mm_per_rad,
                                        self.mC.mm_per_rad)



        #self.m_spd_factor = 100

    def reset_spd_var(self):
        self.phi_dot_A = 0.0
        self.phi_dot_B = 0.0
        self.phi_dot_C = 0.0

    def update_phi_dots_from_loc_vel(self):
        r = self.wheel_r
        gamma_gbl = self.gamma_gbl
        theta_gbl = self.theta_gbl
        theta_dot_loc = self.theta_dot_loc
        R = self.R

        alpha1 = self.alpha1
        alpha2 = self.alpha2
        alpha3 = self.alpha3

        x_spd_loc = self.x_spd_loc
        y_spd_loc = self.y_spd_loc


        # copied from book
        # phi_dot_A = (-sin(theta_gbl) * cos(theta_gbl) * x_spd_loc +
        #         cos(theta_gbl)**2 * y_spd_loc +
        #         R * theta_dot_gbl) \
        #        / r
        #
        # phi_dot_B = (-sin(theta_gbl + alpha2) * cos(theta_gbl) * x_spd_loc +
        #         cos(theta_gbl + alpha2) * cos(theta_gbl) * y_spd_loc +
        #         R * theta_dot_gbl) \
        #        / r
        #
        # phi_dot_C = (-sin(theta_gbl + alpha3) * cos(theta_gbl) * x_spd_loc +
        #         cos(theta_gbl + alpha3) * cos(theta_gbl) * y_spd_loc +
        #         R * theta_dot_gbl) \
        #        / r

        # derived by me
        phi_dot_A = (
                (cos(theta_gbl+alpha1) * sin(gamma_gbl) - sin(theta_gbl+alpha1) * cos(gamma_gbl)) * x_spd_loc +
                (sin(theta_gbl+alpha1) * sin(gamma_gbl) + cos(theta_gbl+alpha1) * cos(gamma_gbl)) * y_spd_loc +
                R * theta_dot_loc) \
               / r

        phi_dot_B = (
                (cos(theta_gbl+alpha2) * sin(gamma_gbl) - sin(theta_gbl+alpha2) * cos(gamma_gbl)) * x_spd_loc +
                (sin(theta_gbl+alpha2) * sin(gamma_gbl) + cos(theta_gbl+alpha2) * cos(gamma_gbl)) * y_spd_loc +
                R * theta_dot_loc) \
               / r

        phi_dot_C = (
                (cos(theta_gbl+alpha3) * sin(gamma_gbl) - sin(theta_gbl+alpha3) * cos(gamma_gbl)) * x_spd_loc +
                (sin(theta_gbl+alpha3) * sin(gamma_gbl) + cos(theta_gbl+alpha3) * cos(gamma_gbl)) * y_spd_loc +
                R * theta_dot_loc) \
               / r



        # negated because the motors are mounted in reverse direction as dreived in math
        self.phi_dot_A = -phi_dot_A
        self.phi_dot_B = -phi_dot_B
        self.phi_dot_C = -phi_dot_C

        print("phi_dot_A/B/C: ", phi_dot_A, phi_dot_B, phi_dot_C)
        print("wheel_spd_A/B/C: ", phi_dot_A*r, phi_dot_B*r, phi_dot_C*r)

        return phi_dot_A, phi_dot_B, phi_dot_C

    def update_gamma(self, time_step):
        gamma_old = self.gamma_gbl
        self.gamma_gbl += self.theta_dot_loc * time_step
        print("gamma_gbl updated: ", gamma_old ," to ", gamma_gbl)


    # function turn_motors_by_duration is changed,
    # this function may be broke as a result
    def drive_by_loc_vel(self, x_spd_loc, y_spd_loc, theta_dot_loc, duration):
        self.x_spd_loc = x_spd_loc
        self.y_spd_loc = y_spd_loc
        self.theta_dot_loc = theta_dot_loc

        self.update_phi_dots_from_loc_vel()
        self.turn_motors_by_duration(duration)

    #important

    # test cmd
    # driver.drive_by_loc_dxdy(100, 0, 2)
    # dx_loc, dy_loc in mm
    # duration in sec
    def drive_by_loc_dxdy(self, dx_loc, dy_loc, duration):
        print("encoder before A/B/C: ", driver.mA.position, driver.mB.position, driver.mC.position)

        print("drive_by_dxdy: ", dx_loc, dy_loc)

        # angle which x_loc and y_loc made: tan(y/x)
        #import pdb; pdb.set_trace()
        ang_a = atan2(dy_loc, dx_loc)
        print("ang_a: ", ang_a)

        # linear speed, the speed which EV3 translate
        l_spd = ( (dx_loc**2 + dy_loc**2)**(0.5) ) / duration
        print("l_spd: ", l_spd)

        self.x_spd_loc = l_spd * cos(ang_a)
        self.y_spd_loc = l_spd * sin(ang_a)
        print("x_spd_loc, y_spd_loc: ", self.x_spd_loc, self.y_spd_loc)

        self.update_phi_dots_from_loc_vel()

        count_A = self.phi_dot_A * duration * self.mA.count_per_rad
        count_B = self.phi_dot_B * duration * self.mB.count_per_rad
        count_C = self.phi_dot_C * duration * self.mC.count_per_rad
        self.turn_motors_by_count(count_A, count_B, count_C, duration)
        #self.turn_motors_by_duration(duration)

        # clean up state after moving
        self.x_spd_loc = 0
        self.y_spd_loc = 0
        print("encoder after A/B/C: ", driver.mA.position, driver.mB.position, driver.mC.position)



    def drive_by_loc_rot(self, rot, duration):
        rot_before = precgyro.angle()

        print("encoder before A/B/C: ", driver.mA.position, driver.mB.position, driver.mC.position)

        dtheta_loc = rot / 360 * 2 * pi
        print("drive_by_rot rot, dtheta_loc: ", rot, dtheta_loc)

        #import pdb; pdb.set_trace()
        # rotation speed, the speed which EV3 rotate
        r_spd = dtheta_loc / duration
        self.theta_dot_loc = r_spd
        print("r_spd: ", r_spd)
        self.update_phi_dots_from_loc_vel()

        count_A = self.phi_dot_A * duration * self.mA.count_per_rad
        count_B = self.phi_dot_B * duration * self.mB.count_per_rad
        count_C = self.phi_dot_C * duration * self.mC.count_per_rad
        self.turn_motors_by_count(count_A, count_B, count_C, duration)
        #self.turn_motors_by_duration(duration)

        # clear up state after moving
        self.theta_dot_loc = 0

        rot_after = precgyro.angle()
        error = rot_after - rot_before + rot

        if abs(error) > ROTATION_ERROR_MARGIN and CORRECT_WITH_GYRO:
            self.drive_by_loc_rot(error, 0.5)


        print("encoder after A/B/C: ", driver.mA.position, driver.mB.position, driver.mC.position)


    # work in progress
    # test cmd
    # driver.drive_by_loc_pose(100, 0, 0, 2)
    # dx_loc, dy_loc in mm
    # duration in sec
    # t_step_size in sec
    def drive_by_loc_pose(self, dx_loc, dy_loc, dtheta_loc, duration, time_step=0.1):

        # angle which x_loc and y_loc made: tan(y/x)
        ang_a = atan2(dy_loc, dx_loc)
        print("ang_a: ", ang_a)

        # linear speed, the speed which EV3 translate
        l_spd = ( (dx_loc**2 + dy_loc**2)**(0.5) ) / duration
        r_spd = dtheta_loc / duration
        print("l_spd, r_spd: ", l_spd, r_spd)

        self.x_spd_loc = l_spd * cos(ang_a)
        self.y_spd_loc = l_spd * sin(ang_a)
        self.theta_dot_loc = r_spd
        print("x_spd_loc, y_spd_loc: ", self.x_spd_loc, self.y_spd_loc)
        print("r_spd: ", r_spd)


        count_A = self.phi_dot_A * duration * self.mA.count_per_rad
        count_B = self.phi_dot_B * duration * self.mB.count_per_rad
        count_C = self.phi_dot_C * duration * self.mC.count_per_rad

        count_largest_target = max(count_A, count_B, count_C)
        count_largest_passed = 0

        time_passed = 0
        print("entering loop")
        while time_passed < duration:
            print("start of loop")

            self.update_phi_dots_from_loc_vel()
            self.turn_motors_by_count_step(count_A, count_B, count_C)
            self.update_gamma(time_step)

            time_passed += time_step




    # should be del / improved? or not?
    # maybe should take the phi_dot_A/B/C parameter explicitly?
    def turn_motors_by_duration(self, duration, t_buffer=0.5):
        print("checking if speed_sp is larger than 900")
        speed_max = 400
        speed_min = -400
        speed_sp_A = self.motor_A.count_per_rad * self.phi_dot_A
        speed_sp_B = self.motor_B.count_per_rad * self.phi_dot_B
        speed_sp_C = self.motor_C.count_per_rad * self.phi_dot_C
        print("speed_sp_A/B/C: ", speed_sp_A, speed_sp_B, speed_sp_C)
        if self.motor_A.count_per_rad * self.phi_dot_A > speed_max or \
           self.motor_A.count_per_rad * self.phi_dot_A < speed_min or \
           self.motor_B.count_per_rad * self.phi_dot_B > speed_max or \
           self.motor_B.count_per_rad * self.phi_dot_B < speed_min or \
           self.motor_C.count_per_rad * self.phi_dot_C > speed_max or \
           self.motor_C.count_per_rad * self.phi_dot_C < speed_min : 
            print("over speed, not moving.") 
            return

        print("start moving")
        self.mA.run_timed(
            time_sp=duration*1000,
            speed_sp= self.motor_A.count_per_rad * self.phi_dot_A,
            stop_action='brake')
        self.mB.run_timed(
            time_sp=duration*1000,
            speed_sp= self.motor_B.count_per_rad * self.phi_dot_B,
            stop_action='brake')
        self.mC.run_timed(
            time_sp=duration*1000,
            speed_sp= self.motor_C.count_per_rad * self.phi_dot_C,
            stop_action='brake')

        self.mA.wait_until_not_moving(timeout=(duration+t_buffer)*1000)
        self.mB.wait_until_not_moving(timeout=(duration+t_buffer)*1000)
        self.mC.wait_until_not_moving(timeout=(duration+t_buffer)*1000)

        #if not self.mA.wait_until_not_moving(timeout=(duration+t_buffer)*1000):
        #    self.mA.stop(stop_action='brake')
        #if not self.mB.wait_until_not_moving(timeout=(duration+t_buffer)*1000):
        #    self.mB.stop(stop_action='brake')
        #if not self.mC.wait_until_not_moving(timeout=(duration+t_buffer)*1000):
        #    self.mC.stop(stop_action='brake')

        self.mA.reset()
        self.mB.reset()
        self.mC.reset()
        print("done moving")
        #self.motor_A.run_timed(
        #    time_sp=duration*1000,
        #    speed_sp=int(
        #        (self.motor_A.count_per_rot / 2.0/pi) * self.phi_dot_A))
        #self.motor_B.run_timed(
        #    time_sp=duration*1000,
        #    speed_sp=int(
        #        (self.motor_B.count_per_rot / 2.0/pi) * self.phi_dot_B))
        #self.motor_C.run_timed(
        #    time_sp=duration*1000,
        #    speed_sp=int(
        #        (self.motor_C.count_per_rot / 2.0/pi) * self.phi_dot_C))
        # self.motor_B.run_timed(
        #     time_sp=duration, speed_sp=int(self.phi_dot_B * self.m_spd_factor))
        # self.motor_C.run_timed(
        #     time_sp=duration, speed_sp=int(self.phi_dot_C * self.m_spd_factor))

    def turn_motors_by_count(self, count_A, count_B, count_C, duration, t_buffer=0.5):
        print("resetting the mA/B/C before we move because we are stupid ;D")
        self.mA.reset()
        self.mB.reset()
        self.mC.reset()
        print("checking if speed_sp is larger than 900")
        speed_max = 400
        speed_min = -400
        speed_max = 1050
        speed_min = -1050
        speed_sp_A = self.motor_A.count_per_rad * self.phi_dot_A
        speed_sp_B = self.motor_B.count_per_rad * self.phi_dot_B
        speed_sp_C = self.motor_C.count_per_rad * self.phi_dot_C
        print("speed_sp_A/B/C: ", speed_sp_A, speed_sp_B, speed_sp_C)
        if self.motor_A.count_per_rad * self.phi_dot_A > speed_max or \
           self.motor_A.count_per_rad * self.phi_dot_A < speed_min or \
           self.motor_B.count_per_rad * self.phi_dot_B > speed_max or \
           self.motor_B.count_per_rad * self.phi_dot_B < speed_min or \
           self.motor_C.count_per_rad * self.phi_dot_C > speed_max or \
           self.motor_C.count_per_rad * self.phi_dot_C < speed_min : 
            print("over speed, not moving.") 
            return
        print("speed_sp of mA/B/C: ",
            (self.motor_A.count_per_rot / 2.0 / pi) * self.phi_dot_A,
            (self.motor_B.count_per_rot / 2.0 / pi) * self.phi_dot_B,
            (self.motor_C.count_per_rot / 2.0 / pi) * self.phi_dot_C)

        print("count_A/B/C: ", count_A, count_B, count_C)

        print("start moving")
        self.motor_A.run_to_rel_pos(
            position_sp=count_A,
            speed_sp=speed_sp_A,
            stop_action='hold')
        self.motor_B.run_to_rel_pos(
            position_sp=count_B,
            speed_sp=speed_sp_B,
            stop_action='hold')
        self.motor_C.run_to_rel_pos(
            position_sp=count_C,
            speed_sp=speed_sp_C,
            stop_action='hold')

        if not self.mA.wait_until_not_moving(timeout=(duration+t_buffer)*1000):
            self.mA.stop(stop_action='hold')
        if not self.mB.wait_until_not_moving(timeout=(duration+t_buffer)*1000):
            self.mB.stop(stop_action='hold')
        if not self.mC.wait_until_not_moving(timeout=(duration+t_buffer)*1000):
            self.mC.stop(stop_action='hold')

        print("done moving")


    def turn_motors_by_time_step(self, time_step):

            print("speed_sp of mA/B/C: ",
                (self.motor_A.count_per_rot / 2.0 / pi) * self.phi_dot_A,
                (self.motor_B.count_per_rot / 2.0 / pi) * self.phi_dot_B,
                (self.motor_C.count_per_rot / 2.0 / pi) * self.phi_dot_C)


            self.motor_A.run_to_rel_pos(
                time_sp=time_step*1000,
                speed_sp=
                    (self.motor_A.count_per_rot / 2.0/pi) * self.phi_dot_A )
            self.motor_B.run_to_rel_pos(
                time_sp=time_step*1000,
                speed_sp=
                    (self.motor_B.count_per_rot / 2.0/pi) * self.phi_dot_B )
            self.motor_C.run_to_rel_pos(
                time_sp=time_step*1000,
                speed_sp=
                    (self.motor_C.count_per_rot / 2.0/pi) * self.phi_dot_C )


def clamp(n, minn, maxn):
  return max(min(maxn, n), minn)


def on_connect(client, userdata, flags, rc):
  print("Connected with result code "+str(rc))
  client.subscribe("local")

def on_connect2(client2, userdata, flags, rc):
    print("connect2: Connected with result code " + str(rc))
    client2.subscribe("fastlane")


def on_message(client, userdata, msg):
  global CORRECT_WITH_GYRO
  #if msg.payload.decode() == "Hello world!":
  #  print("Yes!")
  #  client.disconnect()
  m = msg.payload.decode()

  if m[0]=='x' in m and len(m.split(" ")) > 3:
    #driver.reset_spd_var()
    print("received MQTT: " + m)

    dx = float(m.split(" ")[1])
    dy = float(m.split(" ")[3])
    rot = float(m.split(" ")[5])
    dt = float(m.split(" ")[7])
    print("taking action of MQTT cmd")
    driver.drive_by_loc_dxdy(dx, dy, dt)
    driver.drive_by_loc_rot(rot, dt)
    print("completed MQTT cmd")

  elif m.split(" ")[0] == 'manual_stop':
    print("received MQTT: manual_stop")
    print("taking action of MQTT cmd")
    driver.mA.stop(stop_action='hold')
    driver.mB.stop(stop_action='hold')
    driver.mC.stop(stop_action='hold')
    client.reinitialise(SERVER_ADDRESS, 1883, 60)
    print("completed MQTT cmd")



  elif m.split(" ")[0] == 'grab':
    print("received MQTT: grab")
    # driver.motor_grabber.run_to_abs_pos(position_sp=driver.motor_grabber.count_per_rot/-5, speed_sp=driver.motor_grabber.count_per_rot)
    driver.motor_grabber.run_to_abs_pos(position_sp=driver.motor_grabber.count_per_rot/-2, speed_sp=driver.motor_grabber.count_per_rot)


  elif m.split(" ")[0] == 'release':
    driver.motor_grabber.run_to_abs_pos(position_sp=0, speed_sp=driver.motor_grabber.count_per_rot)

  elif m.split(" ")[0] == "reset_rotation":
    driver.drive_by_loc_rot(precgyro.angle(), max(precgyro.angle()/10, 1.0))

  elif m.split(" ")[0] == "wait":
    print("received MQTT: wait")
    time.sleep( float( m.split(" ")[-1] ) )

  elif m.split(" ")[0] == "self_reset":
    client.reinitialise(SERVER_ADDRESS, 1883, 60)
    mA.reset()
    mB.reset()
    mC.reset()
  elif m.split(" ")[0] == "calibrate_gyro":
      precgyro.reset()
  elif m.split(" ")[0] == "disable_correction":
      CORRECT_WITH_GYRO = False
  elif m.split(" ")[0] == "enable_correction":
      CORRECT_WITH_GYRO = True


def on_message2(client, userdata, msg):
    m = msg.payload.decode()

    if m == "masterstop":
        client.reinitialise(SERVER_ADDRESS, 1883, 60)
        mA.reset()
        mB.reset()
        mC.reset()




mA = Motor()
mB = Motor()
mC = Motor()
driver = Driver(OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D)

precgyro.reset()
print("Hello world")
print("================================================================================")

#print("resetting motor positions, speed_sp and stop actions")
#mA.run_to_abs_pos(position_sp=000, speed_sp=1050, stop_action='hold')
#mB.run_to_abs_pos(position_sp=000, speed_sp=1050, stop_action='hold')
#mC.run_to_abs_pos(position_sp=000, speed_sp=1050, stop_action='hold')
#mA.wait_until_not_moving()
#mB.wait_until_not_moving()
#mC.wait_until_not_moving()
print("resetting motors")
mA.reset()
mB.reset()
mC.reset()
#time.sleep(2)
print("resetting done")

#print("trying test movements, please observe if anything goes wrong")
#driver.drive_by_loc_dxdy(000, 480, 1)
#driver.drive_by_loc_dxdy(480, 000, 1)
#driver.drive_by_loc_dxdy(-480, 000, 1)
#driver.drive_by_loc_dxdy(000, -480, 1)
#
#time.sleep(2)
#
#driver.drive_by_loc_dxdy(000, 58*pi, 1)
#driver.drive_by_loc_dxdy(58*pi, 000, 1)
#driver.drive_by_loc_dxdy(-58*pi, 000, 1)
#driver.drive_by_loc_dxdy(000, -58*pi, 1)

#driver.drive_by_loc_dxdy(100, 000, 1)
#time.sleep(2)
#driver.drive_by_loc_dxdy(200, 000, 1)
#time.sleep(2)
#driver.drive_by_loc_dxdy(250, 000, 1)
#time.sleep(2)
#driver.drive_by_loc_dxdy(300, 000, 1)
#time.sleep(2)
#driver.drive_by_loc_dxdy(400, 000, 1)
#time.sleep(2)
#driver.drive_by_loc_dxdy(600, 000, 1)
#time.sleep(2)
#driver.drive_by_loc_dxdy(1600, 000, 1)
#time.sleep(2)
#print("testing ended")
print("================================================================================")



if len(sys.argv) > 1:
    driver.drive_by_loc_vel(
        float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), int(sys.argv[4]))


client = mqtt.Client()
client.connect(SERVER_ADDRESS, 1883, 60)

client.on_connect = on_connect
client.on_message = on_message

client.loop_forever()

client2 = mqtt.Client()
client2.connect(SERVER_ADDRESS, 1883, 60)

client2.on_connect = on_connect2
client2.on_message = on_message2

client2.loop_forever()



