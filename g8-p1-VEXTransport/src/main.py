# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Stella Wang and Angela Lin                                                        #
# 	Created:      5/5/2026, 9:40:08 AM                                         #
# 	Description:  Code for POE RECbot activities                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

# ------------ Robot Configuration Code --------------------------------------------------
rightMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)  #Right drivetrain motor
leftMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)    #Left drivetrain motor
# Set the leftMotor to reverse so that when driving forward (or reverse) it turns 
# in the same direction as the right motor
drivetrain = DriveTrain(leftMotor, rightMotor)                  #Start both motors simultaneously
liftMotor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)   #Liftarm motor
inertial_1 = Inertial(Ports.PORT5)                              #intertial sensor
bumpSwitch = Bumper(brain.three_wire_port.a)                    #bumper switch
# ----------------------------------------------------------------------------------------

#-------------------------Helper Functions------------------------------------------------
def bump():
    """
    Hold the program's execution until the button is pressed
    """

    while(bumpSwitch.pressing() == False):
        wait(10, MSEC)                  #debounce the button (10 ms)

        brain.screen.set_cursor(1, 1)   #place cursor in upper left corner
        brain.screen.print("Press the button to start the program")
        pass
    
    brain.screen.clear_line(1)          #clear the text on row 1
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Program executed")
    wait(1, SECONDS)                    #wait 1 second before proceeding

def inertialCalibration():
    """
    Calibrate the inertial sensor
    A wait time of 2 seconds is required
    This function should be called at the start of the program's execution
    """

    brain.screen.clear_screen() # Clear the brain's screen
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Calibrating the inertial sensor")
    brain.screen.set_cursor(2,1)
    brain.screen.print("Don't move the robot!")
    inertial_1.calibrate()      # Calibrate the inertial sensor

    wait(2, SECONDS)            # Wait for the calibration to complete

    brain.screen.clear_line(1)  # Clear the text on row 1
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Inertial calibration completed")

def testInertial():
    """
    Test the inertial sensor by having it display heading and total rotation
    data. Pressing the bump switch will end the test.
    """

    brain.screen.clear_screen()
    while(bumpSwitch.pressing() == False):
        wait(10, MSEC) # Debounce the switch
        brain.screen.set_cursor(5,1)
        brain.screen.print("Heading:  " + str(inertial_1.heading()))
        brain.screen.set_cursor(6,1)
        brain.screen.print("Rotation:  " + str(inertial_1.rotation()))
        brain.screen.set_cursor(8,1)
        brain.screen.print("Press the button for the bump switch to exit")
    brain.screen.clear_row(8)
    brain.screen.set_cursor(8,1)
    brain.screen.print("Inertial test terminated")

def driveStraightData(e):
    """
    1. Report position, rotation, and error values to the screen while driving
    2. Parameter: e = error value (setpoint - rotation)
    """

    brain.screen.set_cursor(1,1)
    brain.screen.print("Position: " + str(leftMotor.position())) #Return the current motor count

    brain.screen.set_cursor(2,1)
    brain.screen.print("Rotation: " + str(inertial_1.rotation())) #Return current rotation value

    brain.screen.set_cursor(3,1)
    brain.screen.print("Error: " + str(e)) #Return the current error

def stopMotors():
    """
    Stop both motors at the same time
    """
    drivetrain.stop()
    wait(0.5, SECONDS) #Wait 0.5 seconds for the system to stabilize

def driveStraight(distance, setpoint, motorVelocity):
    """
    1. distance = distance to travel in inches
    2. setpoint = 0-degrees of rotation for driving straight
    3. motorVelocity = the velocity of the motors (+) => forward, (-) => reverse
    """

    inertial_1.reset_rotation() #Reset the rotation before each driving action

    # Set the stopping mode for the motors
    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)

    kP = 0.04       #Proportional constant for driving straight
                    #Used to calculate the correction term to maintain course
                    #If too small, correction will occur too slowly
                    #If too large, overcorrection will occur
                    #Determine the best value by iteratively testing
    wheelDiameter = 4   #Wheel dia. = 4 inches

    #Calculate the distance in terms of encoder ticks (1 tick = 1 degree)
    #Distance (ticks) = (Distance in inches / Wheel Circumference) * 360
    wheelCircumference = wheelDiameter * math.pi     #Wheel circumference
    distance = (distance / wheelCircumference) * 360 #Distance in ticks

    #Reset the motor encoder count to zero before driving
    leftMotor.set_position(0, DEGREES)
    rightMotor.set_position(0, DEGREES)

    #Drive forward if the motor velocity > 0
    if (motorVelocity > 0):
        #While loop to track the distance traveled
        while(leftMotor.position() < distance):
            error = (setpoint - inertial_1.rotation()) #Calculate error
            correction = kP * error                    #Motor velocity correction

            #Correct motor velocity
            #If error > 0 (setpoint > rotation) => drifting left
            #If error < 0 (setpoint < rotation) => drifting right

            leftMotor.set_velocity((motorVelocity + correction), PERCENT)
            rightMotor.set_velocity((motorVelocity - correction), PERCENT)

            # Spin motors
            drivetrain.drive(FORWARD)

            driveStraightData(error) #Display position, rotation, and error
        
        #Stop the motors when the desired distance is reached
        stopMotors()

    # Drive straight in reverse if the motor velocity < 0
    else:

        distance *= -1  #distance = distance * -1
        #While loop to track the distance traveled
        while(leftMotor.position() > distance):
            error = (setpoint - inertial_1.rotation()) #Calculate error
            correction = kP * error                    #Motor velocity correction

            #Correct motor velocity
            #If error > 0 (setpoint > rotation) => drifting left
            #If error < 0 (setpoint < rotation) => drifting right

            leftMotor.set_velocity((motorVelocity + correction), PERCENT)
            rightMotor.set_velocity((motorVelocity - correction), PERCENT)

            # Spin motors
            drivetrain.drive(FORWARD)

            driveStraightData(error) #Display position, rotation, and error
        
        #Stop the motors when the desired distance is reached
        stopMotors()

def turnData(turnError, derivative):
    """
    Print the current heading, turning error, and derivative values
    """

    brain.screen.set_cursor(1,1)
    brain.screen.print("Heading: " + str(inertial_1.heading())) #Return the current heading

    brain.screen.set_cursor(2,1)
    brain.screen.print("Error: " + str(abs(turnError))) #Return current turning error

    brain.screen.set_cursor(3,1)
    brain.screen.print("Derivative: " + str(abs(derivative))) #Return the current derivative
    
def pointTurn(setPoint):
    """
    1. Perform a point turn using the inertial sensor and proportional and derivative control
    2. Argument: Desired heading (setPoint)
    """

    brain.screen.clear_screen() #Clear the brain's screen

    #Set stopping mode for the turn
    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)

    # Calculate the difference between the setpoint and current heading
    difference = setPoint - inertial_1.heading()

    # Want to minimize the amount of turn required
    if (setPoint > inertial_1.heading()):
        if (abs(difference) <= 180):       #Turn CW
            clockwise = True
        else:
            clockwise = False              #Turn CCW
    else:
        if (abs(difference) <= 180):       #Turn CCW
            clockwise = False
        else:
            clockwise = True               #Turn CW

    #Define kP and kD values for the CW and CCW turns
    if (clockwise):
        kP = 0.14   #Values if clockwise
        kD = 0.41
    else:           #Values if counterclockwise
        kP = 0.14
        kD = 0.49
    
    #Define the maximum velocity and previous error terms
    maxVelocity = 50    #Units: %
    previousError = 0.0 #Error from the previous iteration of the control loop

    while (True):
        turnError = setPoint - inertial_1.heading()
        derivative = turnError - previousError

        #Stop the motors and exit the control loop when the error and
        # derivative terms are sufficiently small to ensure the
        # setpoint was reached without oscillation 
        if ((abs(turnError) < 1) and (abs(derivative) < 0.2)):
            stopMotors() #Stop the motors
            break        #Leave the loop

        # Proportional and derivative correction calculations
        turnCorrection = (kP * turnError) + (kD * derivative)

        # Limit the corrective term to make sure we don't exceed the maximum velocity
        if (abs(turnCorrection) > 1):
            turnCorrection = 1

        turnVelocity = turnCorrection * maxVelocity

        #Set the motor velocities
        if (clockwise):           #Turn CW
            leftMotor.set_velocity(turnVelocity)
            rightMotor.set_velocity(-1 * turnVelocity)
        else:                     #Turn CCW
            leftMotor.set_velocity(-1 * turnVelocity)
            rightMotor.set_velocity(turnVelocity)

        # Spin the motors
        leftMotor.spin(FORWARD)
        rightMotor.spin(FORWARD)

        turnData(turnError, derivative) #Print heading, error, and derivative values

        previousError = turnError #Update the previous error term

        wait(20, MSEC)

def liftArm(motorVelocity, liftAngle):
    # Configure the motor to hold its position 
    liftMotor.set_stopping(HOLD)

    liftMotor.set_velocity(motorVelocity, PERCENT)  #Set lift arm motor velocity
    
    gearRatio = 5       #60T to 12T
    motorAngularDisplacement = liftAngle * gearRatio #Calculate motor axle's angular displacement

    #Spin motor forward for the given angular displacement in degrees
    liftMotor.spin_for(FORWARD, motorAngularDisplacement, DEGREES)
    wait(0.5, SECONDS)

def decelerate():
    step = 3
    if (leftMotor.position() < 5 & rightMotor.position() < 5):
        while(leftMotor.velocity() > 50 & rightMotor.velocity() > 50):
            leftMotor.set_velocity(leftMotor.velocity()-step)
            rightMotor.set_velocity(rightMotor.velocity()-step)

# -----------------------Define main() function-----------------------
def main():
    """
    The main() function is the program that will be executed by the brain
    """
    bump()                     # Call bump() to execute the program
    inertialCalibration()      # Calibrate the inertial sensor

    driveStraight(94, 0, 100) #Drive to stand
        #decelerate()              #Decelerate the robot
    liftArm(20, 40)            #Lift ball from stand
        #decelerate()
    driveStraight(11, 0, -40)   #Drive backwards
    pointTurn(90)              #Point turn 90 degrees
        #Decelerate function
    driveStraight(67, 0, 70)   #Drive horizontally to box
    pointTurn(48)             #Orient robot to face box
        #Decelerate function
    driveStraight(10, 0, 60)    #Drive forward to box
    liftArm(20, -40)           #Lower ball into box
    liftArm(20, 90)            #Lift arm back up to 90 degrees
    pointTurn(137)             #Turn 45 degrees
        #Decelerate function
    driveStraight(21, 0, -90)    #Drive forward to third path
    pointTurn(90)              #Orient robot to drive straight
        #Decelerate function
    driveStraight(39, 0, -100)   #Drive straight to final spot


#---------------------------------------------------------------------
main()
