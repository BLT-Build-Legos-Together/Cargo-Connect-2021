#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFiles
import json

ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
medium_motor = Motor(Port.A)

light_sensor = ColorSensor(Port.S4)
secondary_light_sensor = ColorSensor(Port.S3)

ev3.speaker.set_speech_options("en", "m1", 100, 75)

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=117)
robot.settings(700, 300, 250, 250)
#Start up and intialize the screen, speakers, motors, sensors and the robot drivebase.

def rgb_reflection():
    x = light_sensor.rgb()
    return ((x[0]+x[1]+x[2])/3)
# Give an average of all three colors lighted up by the first light sensor to get an reliable average of the light reflection.

def secondary_rgb_reflection():
    x = secondary_light_sensor.rgb()
    return ((x[0]+x[1]+x[2])/3)
# Give an average of all three colors lighted up by the second light sensor to get an reliable average of the light reflection.


rgb_reflection()
secondary_rgb_reflection() # Not using these values for anything, but turning on all three lights on the sensor for better calibration purposes.

calibrate = True
prompt = True
firstblack = True
firstwhite = True
secondblack = True
secondwhite = True

# Set up mission variables
cargo_plane = True
truck_bridge = True
truck2truck = True
homedelivery = True

# Set all these variables to True so they activate their respective function until told they are False.

ev3.screen.clear()
ev3.screen.draw_text(3, 20, "   Do you want to")
ev3.screen.draw_text(5, 40, "       calibrate?")
ev3.screen.draw_text(6, 60, "(Press Up for Yes")
ev3.screen.draw_text(8, 80, "and Down for No)")
# Display on the screen if we want to calibrate the light sensors.
while prompt:
    if Button.UP in ev3.buttons.pressed():
        prompt = False
    if Button.DOWN in ev3.buttons.pressed():
        calibrate = False
        prompt = False
        wait(225)
        # Wait to get into the program UI because then if there is no wait it'll immediately register as you wanting to do program #5.

# This while loop checks if you want to calibrate or not, and if you press up it will take you to the calibrate loop.

if calibrate:
    ev3.screen.clear()
    ev3.screen.draw_text(15, 45, "Place right light")
    ev3.screen.draw_text(15, 65, "sensor on black!")
    # Display on the screen to put the right light sensor on black and press the center button to confirm.

    while firstblack:
        if Button.CENTER in ev3.buttons.pressed():
            value_dict = {
                "black" : rgb_reflection(),
                "white" : 100
            }
            json_object = json.dumps(value_dict)
            with open("light.json", "w") as light:
                light.write(json_object)
            wait(275)
            firstblack = False
    # Saves this value as black for the right light sensor to a json file that can be saved even when the program terminates.

    ev3.screen.clear()
    ev3.screen.draw_text(15, 45, "Place right light")
    ev3.screen.draw_text(15, 65, "sensor on white!")
    # Display on the screen to put the right light sensor on white and press the center button to confirm.

    while firstwhite:
        if Button.CENTER in ev3.buttons.pressed():
            value_dict["white"] = (rgb_reflection())
            json_object = json.dumps(value_dict)
            with open("light.json", "w") as light:
                light.write(json_object)
            wait(275)
            firstwhite = False
    # Saves this value as white for the right light sensor to a json file that can be saved even when the program terminates.

    ev3.screen.clear()
    ev3.screen.draw_text(15, 45, "Place left light")
    ev3.screen.draw_text(15, 65, "sensor on black!")
    # Display on the screen to put the left light sensor on black and press the center button to confirm.

    while secondblack:
        if Button.CENTER in ev3.buttons.pressed():
            value_dict2 = {
                "black" : secondary_rgb_reflection(),
                "white" : 100
            }
            json_object = json.dumps(value_dict2)
            with open("secondary_light.json", "w") as light:
                light.write(json_object)
            wait(275)
            secondblack = False
    # Saves this value as black for the left light sensor to a second json file that can be saved even when the program terminates.
    
    ev3.screen.clear()
    ev3.screen.draw_text(15, 45, "Place left light")
    ev3.screen.draw_text(15, 65, "sensor on white!")
    # Display on the screen to put the left light sensor on white and press the center button to confirm.

    while secondwhite:
        if Button.CENTER in ev3.buttons.pressed():
            value_dict2["white"] = (secondary_rgb_reflection())
            json_object = json.dumps(value_dict2)
            with open("secondary_light.json", "w") as light:
                light.write(json_object)
            wait(275)
            secondwhite = False
    # Saves this value as white for the left light sensor to a second json file that can be saved even when the program terminates.

# This big if statement checks the light sensor values when you put the sensors on white/black, and then tells the line follow function what values are considered black and what values are considered white.

def line_follow(rotations, speed, kp, kd): # A pd line follow function (using only proporiontal and the derivitive).
    control = 0
    last_deviation = 0
    left_motor.reset_angle(0) # Reset left motor so no previous angle measurements interfere
    right_motor.reset_angle(0) # Reset right  motor so no previous angle measurements interfere
    with open("light.json", "r") as light: # Open our json file of the light sensor readings when they are on black/white
        light_dict = json.load(light)
    middle =  ((light_dict["black"]) + (light_dict["white"]))/2 # Take the middle of the black and white readings 
    #so you follow the middle of a line.
    # --v This is not allowed in lego LabView, and with micropython we were able to line follow more accurately.
    while ((left_motor.angle())+(right_motor.angle()))/2 < 360*(rotations): # Check if the robot goes more than target every tick
        deviation = rgb_reflection() - middle # Calculate deviation from the middle of the line
        proportional = (kp) * deviation # Find out how much the robot needs to turn based off of deviation.
        derivitive = (deviation - last_deviation) * (kd)
        turn_rate = proportional + derivitive
        last_deviation = deviation # get this to be recursive
        robot.drive((speed), turn_rate) # Actually turn the robot to correct the errors.
    robot.stop() # Once target is reached, tell the motors that it is done so it's ready to do the next task.


def secondary_line_follow(rotations, speed, kp, kd): # A pd line follow function (using only proporiontal and the derivitive).
    control = 0
    last_deviation = 0
    left_motor.reset_angle(0) # Reset left motor so no previous angle measurements interfere
    right_motor.reset_angle(0) # Reset right  motor so no previous angle measurements interfere
    with open("secondary_light.json", "r") as light:
        light_dict2 = json.load(light)
    # Open up the second json file to find the values of black and white for the left light sensor
    middle =  (((light_dict2["black"]) + (light_dict2["white"]))/2) # Take the average of readings between both motors for more accurate measurement.
    # --v This is not allowed in lego LabView, and with micropython we were able to line follow more accurately.
    while ((left_motor.angle())+(right_motor.angle()))/2 < 360*(rotations): # Check if the robot goes more than target every tick
        deviation = secondary_rgb_reflection() - middle # Calculate deviation from the middle of the line
        proportional = (kp) * deviation # Find out how much the robot needs to turn based off of deviation.
        derivitive = (deviation - last_deviation) * (kd)
        turn_rate = proportional + derivitive
        control += 1
        if control==5:
            control = 0
        if control==4:
            last_deviation = deviation # get this to be recursive
        robot.drive((speed), turn_rate) # Actually turn the robot to correct the errors.
    robot.stop() # Once target is reached, tell the motors that it is done so it's ready to do the next task.

ev3.screen.clear()


while cargo_plane or truck_bridge or truck2truck or homedelivery:
# When any of the mission runs haven't been done, this master program will not terminate. The program will only end when all of the missions have been done.
    ev3.screen.draw_text(0, 0, "Pick Program")
    ev3.screen.draw_text(85, 28, "1")
    ev3.screen.draw_text(41, 60, "2")
    ev3.screen.draw_text(85, 60, "3")
    ev3.screen.draw_text(130, 60, "4")
    ev3.screen.draw_text(85, 92, "5")
    # ^ to draw a little map of which button goes to each program

    if Button.LEFT in ev3.buttons.pressed(): # If you press the left button, then our solution to the cargo airplane and unused capacity missions will run.
        
        robot.straight(303)
        robot.stop()

        line_follow(1, 175, -0.4, 1)

        left_motor.reset_angle(0)
        left_motor.run_target(500, 260, then=Stop.BRAKE, wait="True")
        left_motor.stop()

        line_follow(1.5, 150, -0.4, 1)

        medium_motor.reset_angle(0)
        medium_motor.run_target(300, 145, then=Stop.BRAKE, wait="False")
        medium_motor.stop()

        left_motor.reset_angle(0)
        left_motor.run_target(500, 83, then=Stop.BRAKE, wait="True")
        left_motor.stop()

        robot.straight(45)
        robot.straight(-5)
        robot.stop()

        medium_motor.reset_angle(0)
        medium_motor.run_target(100, -110, then=Stop.BRAKE, wait="True")
        medium_motor.stop()

        robot.straight(-410)
        robot.stop()

        right_motor.reset_angle(0)
        right_motor.run_target(500, 72, then=Stop.BRAKE, wait="True")
        right_motor.stop()

        medium_motor.reset_angle(0)
        medium_motor.run_target(300, 144, then=Stop.BRAKE, wait="True")
        medium_motor.reset_angle(0)
        medium_motor.run_target(500, -160, then=Stop.BRAKE, wait="True")
        medium_motor.stop()

        robot.straight(-300)
        robot.turn(5)
        robot.stop()

        wait(2000)

        robot.settings(1500, 1500, 1500, 1500)
        robot.straight(400)
        robot.straight(-400)
        robot.stop()
        
        robot.settings(700, 300, 250, 250)

        cargo_plane = False # The master while loop depends on whether all the missions are not done. Setting this to false tells the loop that the cargo_plane is finsished.

    if Button.UP in ev3.buttons.pressed(): # If you press the top button, the our solution to the truck, bridge, and helicopter missions will run.
        
        robot.straight(130) # Go straight to the line
        robot.stop() # Stop the robot drivebase to give access for the line_follow function to control the motors.
        
        line_follow(1.55, 175, -0.4, 1) # Line follow for 1.55 rotations at 175 about 175 milimeters per second. 

        robot.straight(120) # Go straight for room to turn
        robot.stop() # Stop the robot drivebase for motor control

        left_motor.reset_angle(0) # Clear earlier data
        left_motor.run_target(500, 180, then=Stop.BRAKE, wait="True") # Turn the robot with one motor to align with the next line
        left_motor.stop() # Stop having the left_motor be active.

        medium_motor.reset_angle(0) # Reset the angle on the medium motor so we can accurately measure it in the next step.
        medium_motor.run_target(300, -135, then=Stop.BRAKE, wait="True") #  The medium motor turns down the handle and allows the robot to latch onto the truck.
        medium_motor.stop() # Tell the program that we are done moving the medium motor and we can move on to later steps.

        line_follow(1.4, 135, -0.4, 1) # Follow the line for 1.3 rotations so the truck will latch onto the bridge.

        medium_motor.reset_angle(0) # Reset the angle on the medium motor so we can accurately measure it in the next step.
        medium_motor.run_target(300, 35, then=Stop.BRAKE, wait="True") # The medium motor turn on and unlatch the robot from the truck
        medium_motor.stop() # Tell the program that we are done moving the medium motor and we can move on to later steps.

        line_follow(0.9, 175, -0.4, 1) # Follow the line for 1 rotation to hit one part of the bridge down
        
        medium_motor.reset_angle(0) # Reset the angle on the medium motor so we can accurately measure it in the next step.
        medium_motor.run_target(300, 110, then=Stop.BRAKE, wait="True") # The medium motor turn on and allow the arm to go over the bridge
        medium_motor.stop() # Tell the program that we are done moving the medium motor and we can move on to later steps.

        line_follow(0.7, 175, -0.4, 1) # Line follow in front of the bridge

        medium_motor.reset_angle(0) # Reset the angle on the medium motor so we can accurately measure it in the next step.
        medium_motor.run_target(300, -120, then=Stop.BRAKE, wait="True") # The medium motor turn on and move it down so it can hit down the bridge
        medium_motor.stop() # Tell the program that we are done moving the medium motor and we can move on to later steps.

        robot.straight(-320) # Go backwards to hit the bridge down
        robot.stop()

        line_follow(0.5, 30, -0.55, 1) # Line follow carefully in order to re-align the robot.

        right_motor.reset_angle(0)
        right_motor.run_target(500, 355, then=Stop.BRAKE, wait="True") # Turn the robot with one motor to be perpendicular to the cargo boat.
        right_motor.stop()

        robot.straight(70) # Go forward towards the cargo boat
        robot.stop()

        left_motor.reset_angle(0)
        left_motor.run_target(500, 360, then=Stop.BRAKE, wait="True") # Turn to be parallel to the cargo boat.
        left_motor.stop()

        robot.straight(255) # Go forward to activate the cargo boat and move the cargo.
        robot.stop()

        robot.straight(-160) # Go backwards from the cargo boat.
        robot.turn(20) # Turn to go on trajectory for the line
        robot.straight(260) # Go forward to catch the line.
        robot.turn(-45) # Turn to be on the correct angle of the line (easier on the line_follow correction, allows for more speed)
        robot.stop()

        line_follow(1.9, 175, -0.4, 1) # Line follow up to the air drop

        robot.straight(120) # Go forward to activate the air drop
        robot.straight(-400) # Come back from the air drop
        robot.turn(-130) # Turn to face the transportation journey for the truck.
        robot.straight(275) # Go forwards towards the line
        robot.stop()

        line_follow(1, 100, 0.4, 1) # Line follow to re-align the robot

        robot.turn(-5) # Turn a little bit for optimal angle for activating the transportation journey of the truck bridge
        robot.straight(140) # Go straight to come close to the transportation journey of the truck.
        robot.stop()

        medium_motor.reset_angle(0)
        medium_motor.run_until_stalled(-100, then=Stop.BRAKE, duty_limit=40)
        medium_motor.stop()

        robot.straight(225) # Move the truck forward.
        robot.straight(-100) # Come back from the transportation journey.
        robot.stop()

        left_motor.reset_angle(0)
        left_motor.run_target(300, -180, then=Stop.BRAKE, wait="True") # Turn towards home.
        left_motor.stop()

        robot.straight(275) # Go forward to the y-value of the blue hinged container
        robot.turn(45) # Turn towards the blue hinged container.
        robot.straight(1000) # Go inside home with the hinged container.
        robot.stop()

        truck_bridge = False # The master while loop depends on whether all the missions are not done. Setting this to false tells the loop that the truck_bridge run is finsished.

    if Button.RIGHT in ev3.buttons.pressed(): # If you press the CENTER button, the our solution to connecting the trucks, putting the grey blocks, and the parking mission will run.
        robot.straight(563)

        robot.straight(-173)

        robot.turn(-45)

        robot.straight(277)
        robot.stop()

        left_motor.reset_angle(0)
        left_motor.run_target(500, 175, then=Stop.BRAKE, wait="True")
        left_motor.stop()

        robot.straight(450)
        robot.stop()

        right_motor.reset_angle(0)
        right_motor.run_target(500, 255, then=Stop.BRAKE, wait="True")
        right_motor.stop()

        robot.straight(303)
        robot.turn(-85)
        robot.stop()

        robot.settings(50, 50, 50, 50)
        robot.straight(190)
        robot.stop()
        robot.settings(700, 300, 250, 250)

        truck2truck = False

    if Button.DOWN in ev3.buttons.pressed():
        medium_motor.reset_angle(0)
        medium_motor.run_until_stalled(100, then=Stop.BRAKE, duty_limit=50)
        medium_motor.stop()

    if Button.CENTER in ev3.buttons.pressed(): # Runs our solution to the home delivery, innovation project, large delivery, and air delivery.
        medium_motor.reset_angle(0)
        medium_motor.run_until_stalled(-100, then=Stop.BRAKE, duty_limit=50)
        medium_motor.stop()

        robot.straight(275)
        robot.stop()

        line_follow(0.9, 150, 0.4, 1)
        line_follow(0.25, 50, 0.6, 1)

        robot.turn(-35)

        robot.straight(200)

        robot.turn(35)

        robot.stop()

        robot.settings(1200, 300, 500, 250)
        robot.straight(620)
        robot.stop()
        robot.settings(700, 300, 250, 250)

        robot.turn(-80)

        robot.straight(75)
        robot.turn(-5)

        robot.straight(-200)
        robot.stop()

        robot.settings(400, 300, 250, 250)
        robot.straight(-200)
        robot.stop()
        robot.settings(700, 300, 250, 250)

        robot.straight(150)

        robot.turn(125)

        robot.straight(225)

        robot.turn(-43)

        robot.straight(40)

        robot.stop()

        medium_motor.reset_angle(0)
        medium_motor.run_until_stalled(100, then=Stop.BRAKE, duty_limit=50)
        medium_motor.stop()

        robot.settings(1500, 750, 500, 250)

        robot.straight(-100)
        robot.turn(40)
        
        robot.straight(-350)
        
        robot.turn(-40)

        robot.straight(-1500)

        robot.stop()
        robot.settings(700, 300, 250, 250)


    # This is an angle tracker for each motor, used when moving the robot freehand and trying to see how far we should move the robot.
    
    #if Button.DOWN in ev3.buttons.pressed(): 
    #    ev3.screen.clear()
    #    while True:
    #        ev3.screen.draw_text(5, 5, "Left motor angle:" + str(left_motor.angle()), text_color=Color.BLACK, background_color=None)
    #        ev3.screen.draw_text(5, 50, "Right motor angle:" str(right_motor.angle()), text_color=Color.BLACK, background_color=None)
    #        wait(10)
    #        ev3.screen.clear()