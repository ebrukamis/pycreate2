##############################################
# Author: Ebru Emir
# Date: 2021/06/04
##############################################
from pycreate2.OI import MOTORS
from random import randint
from  pycreate2 import Create2
import time
import math

# Create a robot instance.
port = '/dev/ttyUSB0' # serial port name
baud = {'default': 115200,
        'alt': 19200
        }
bot = Create2(port, baud['default'])    

# Start the Create2
bot.start()
print('Robot is starting...')

# Put the Create2 into 'safe' mode so we can drive it
# This will still provide some protection
bot.safe()

# Get behavior combination inputs from the user
print('-'*70)
flowLaban = input("Enter the flow component of Laban Efforts [0: bound, 1:free]:\n")
spaceLaban = input("Enter the space component of Laban Efforts [0: direct, 1:indirect]:\n")
timeLaban = input("Enter the time component of Laban Efforts [0: sudden, 1:sustained]:\n")
weightLaban = input("Enter the weight component of Laban Efforts [0: strong, 1:light]:\n")
combination = []
# 1) Flow - Path Planning and Range of Motion: {bound:0, free:1} 
# bound: linear path planning and theta = 90
# free: random path planning and 45 <= theta <= 135
# linear path: direct drive, turn 90 towards a direction, direct drive, turn 90 again, direct drive again
# random path:  direct drive until an obstacle, change direction in a random angle and direct drive again
if flowLaban == "0":
        combination.append(0)
elif flowLaban == "1":
        combination.append(1)
# 2) Space - Radius of Curvature: {direct:0, indirect:1}
# direct: radius of curvature is 0 mm, in other words has corner points
# indirect: radius of curvature is 165 mm
if spaceLaban == "0":
        combination.append(0)
elif spaceLaban == "1":
        combination.append(1)
# 3) Time - Velocity: {sudden:0, sustained:1}
# sudden: Direct drive = 200mm/s and Rotational turn = 160mm/s
# sustained: Direct drive = 100mm/s and Rotational turn = 80mm/s 
if timeLaban == "0":
        combination.append(0)
elif timeLaban == "1":
        combination.append(1)
#4) Weight - Speed of vacuum: {strong:0, light:1}
# strong: 100% vacuum duty cycle
# light: 50% vacuum duty cycle 
if weightLaban == "0":
        combination.append(0)
elif weightLaban == "1":
        combination.append(1)
print("Combination inputs are saved: ")
print(combination)
print('-'*70)

# set the inputs to variables
flow_level      = int(combination[0])
space_level     = int(combination[1])
time_level      = int(combination[2])
weight_level    = int(combination[3])

# Path planning is random (45 <= theta <= 135)
if flow_level == 1: 
        # start the motors (side brush, main brush and vacuum) 
        vacuum_pwm = int(127*(1-weight_level/2))
        main_brush_pwm = int(127*(1-weight_level/2))
        side_brush_pwm = int(127*(1-weight_level/2))
        bot.pwm_motors(main_brush_pwm, side_brush_pwm, vacuum_pwm)                                 
        # set the velocity values
        velocity_frw = int(200-100*time_level)
        velocity_turn = int(160-80*time_level)
        # set the duration of turn (dt = 2*pi*r*theta/360/V)
        w_radius = 113.4 # inside the wheels (mm)
        t_spin_l = 2*math.pi*w_radius*135/360/velocity_turn
        t_spin_m = 2*math.pi*w_radius*90/360/velocity_turn
        t_spin_s = 2*math.pi*w_radius*45/360/velocity_turn
        t_spin_2 = math.pi*343*135/360/velocity_turn
        t_spin_3 = math.pi*343*90/360/velocity_turn
        t_spin_4 = math.pi*343*45/360/velocity_turn
        # set the duration of each step
        t_step_1 = []
        t_step_2 = []
        step_1 = [1905, 1905, 635, 1092.2, 1524, 1016, 1651, 889] 
        step_2 = [1727, 1651, 635, 1092,   1193.6, 761.8, 1651, 838] if time_level == 1 else [1727, 1727, 457, 1092,   1193.6, 761.8, 1473, 838] 
                #path used for conditions [1-1-1-x] 
                #path used for conditions [1-1-0-x] 
        for i in range(8):
                t_step_1.append(int(step_1[i]/velocity_frw)-2*time_level)
                t_step_2.append(int((step_2[i])/velocity_frw)-2*time_level)
        
        if space_level == 1:
                # [1-1-x-x] - step_2
                for i in range(8):
                        # define the path
                        bot.drive_direct(velocity_frw,velocity_frw)
                        time.sleep(t_step_2[i])
                        if i == 0 or i == 3:
                                # spin right/clockwise
                                bot.drive(velocity_turn, -171) 
                                time.sleep(t_spin_2*4/3)
                        if i == 1 or i == 2:
                                # spin right/clockwise
                                bot.drive(velocity_turn, -171) 
                                time.sleep(t_spin_3*4/3)
                        if i == 4 or i == 6:
                                # spin left/counterclockwise
                                bot.drive(velocity_turn, 171) 
                                time.sleep(t_spin_3*4/3)
                        if i == 5: 
                                # spin left/counterclockwise
                                bot.drive(velocity_turn, 171) 
                                time.sleep(t_spin_2*4/3)
        elif space_level == 0:
                # [1-0-x-x] - step_1
                for i in range(8):
                        # define the path
                        bot.drive_direct(velocity_frw,velocity_frw)
                        time.sleep(t_step_1[i])
                        if i == 0 or i == 3:
                                # spin right/clockwise
                                bot.drive_direct(-velocity_turn, velocity_turn) 
                                time.sleep(t_spin_l)
                        if i == 1 or i == 2:
                                # spin right/clockwise
                                bot.drive_direct(-velocity_turn, velocity_turn) 
                                time.sleep(t_spin_m)
                        if i == 4 or i == 6:
                                # spin left/counterclockwise
                                bot.drive_direct(velocity_turn, -velocity_turn) 
                                time.sleep(t_spin_m)
                        if i == 5: 
                                # spin left/counterclockwise
                                bot.drive_direct(velocity_turn, -velocity_turn) 
                                time.sleep(t_spin_l)
# Path planning is linear (theta = 90)
elif flow_level == 0:
        # start the motors (side brush, main brush and vacuum) 
        vacuum_pwm = int(127*(1-weight_level/2))
        main_brush_pwm = int(127*(1-weight_level/2))
        side_brush_pwm = int(127*(1-weight_level/2))
        bot.pwm_motors(main_brush_pwm, side_brush_pwm, vacuum_pwm)                                 
        # set the velocity values
        velocity_frw = int(200-100*time_level)
        velocity_turn = int(160-80*time_level)
        # set the duration of turn (dt = 2*pi*r*theta/360/V)
        w_radius = 113.4 # inside the wheels (mm)
        diameter = 330.2 # the whole robot (mm)
        t_spin = 2*math.pi*w_radius*90/360/velocity_turn
        t_spin_2 = math.pi*343*180/360/velocity_turn
        # set the duration of straight drive
        distance = 2184.4 # distance of straight drive (mm)
        distance_s = 1905 # distance of straight drive (mm)
        distance_xs = 1651 # distance of straight drive (mm)
        t_straight = int(distance/velocity_frw)-4*time_level
        t_straight_s = int(distance_s/velocity_frw)-4*time_level
        t_straight_xs = int(distance_xs/velocity_frw)-4*time_level
        t_step = int(diameter*1.3/velocity_frw)

        if space_level == 0:
                # define the movement path
                for i in range(5):
                        # go straight
                        bot.drive_direct(velocity_frw,velocity_frw)
                        time.sleep(t_straight)
                        # turn/spin
                        if i==0 or i==2:
                                # spin right/clockwise
                                bot.drive_direct(-velocity_turn, velocity_turn)
                                time.sleep(t_spin)
                                # go straight between spins
                                bot.drive_direct(velocity_frw, velocity_frw)
                                time.sleep(t_step)
                                # spin right/clockwise
                                bot.drive_direct(-velocity_turn, velocity_turn)
                                time.sleep(t_spin)
                        if i==1 or i==3:
                                # spin left/counterclockwise
                                bot.drive_direct(velocity_turn, -velocity_turn)
                                time.sleep(t_spin)
                                # go straight between spins
                                bot.drive_direct(velocity_frw, velocity_frw)
                                time.sleep(t_step)
                                # spin left/counterclockwise
                                bot.drive_direct(velocity_turn, -velocity_turn)
                                time.sleep(t_spin)
                        if i==4:
                                # stop the motors
                                bot.pwm_motors(0,0,0)
        elif space_level == 1:
                # define the movement path
                bot.drive_direct(velocity_frw,velocity_frw)
                time.sleep(t_straight_s)
                # spin right/clockwise
                bot.drive(velocity_turn, -171) 
                time.sleep(t_spin_2*4/3)

                bot.drive_direct(velocity_frw,velocity_frw)
                time.sleep(t_straight_xs)
                # spin left/counterclockwise
                bot.drive(velocity_turn, 171) 
                time.sleep(t_spin_2*4/3)

                bot.drive_direct(velocity_frw,velocity_frw)
                time.sleep(t_straight_xs)
                # spin right/clockwise
                bot.drive(velocity_turn, -171) 
                time.sleep(t_spin_2*4/3)

                bot.drive_direct(velocity_frw,velocity_frw)
                time.sleep(t_straight_xs)
                # spin left/counterclockwise
                bot.drive(velocity_turn, 171) 
                time.sleep(t_spin_2*4/3)

                #define the path
                bot.drive_direct(velocity_frw,velocity_frw)
                time.sleep(t_straight_s)
                # stop the motors
                bot.pwm_motors(0,0,0)

# Stop driving the bot
bot.drive_stop()

# Finished Cleaning song
song = [72, 8, 72, 8, 76, 8, 79, 16, 76, 8, 79, 32]
song_num = 1
bot.createSong(song_num, song)
time.sleep(0.1)
how_long = bot.playSong(song_num)
time.sleep(how_long)
