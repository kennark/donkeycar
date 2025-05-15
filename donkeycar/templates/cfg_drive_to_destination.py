
import os


#
# FILE PATHS
#
CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, 'data')

#
# VEHICLE loop
#
DRIVE_LOOP_HZ = 10      # the vehicle loop will pause if faster than this speed.
MAX_LOOPS = None        # the vehicle loop can abort after this many iterations, when given a positive integer.


#
# CAMERA configuration
#
CAMERA_TYPE = "PICAM"   # (PICAM|WEBCAM|CVCAM|CSIC|V4L|D435|MOCK|IMAGE_LIST)
IMAGE_W = 160
IMAGE_H = 120
IMAGE_DEPTH = 3         # default RGB=3, make 1 for mono
CAMERA_FRAMERATE = DRIVE_LOOP_HZ
CAMERA_VFLIP = False
CAMERA_HFLIP = False
CAMERA_INDEX = 0  # used for 'WEBCAM' and 'CVCAM' when there is more than one camera connected
# For CSIC camera - If the camera is mounted in a rotated position, changing the below parameter will correct the output frame orientation
CSIC_CAM_GSTREAMER_FLIP_PARM = 0 # (0 => none , 4 => Flip horizontally, 6 => Flip vertically)
BGR2RGB = False  # true to convert from BRG format to RGB format; requires opencv

DONKEY_GYM = False

#
# DRIVE_TRAIN_TYPE
# These options specify which chasis and motor setup you are using.
# See Actuators documentation https://docs.donkeycar.com/parts/actuators/
# for a detailed explanation of each drive train type and it's configuration.
# Choose one of the following and then update the related configuration section:
#
# "PWM_STEERING_THROTTLE" uses two PWM output pins to control a steering servo and an ESC, as in a standard RC car.
# "MM1" Robo HAT MM1 board
# "SERVO_HBRIDGE_2PIN" Servo for steering and HBridge motor driver in 2pin mode for motor
# "SERVO_HBRIDGE_3PIN" Servo for steering and HBridge motor driver in 3pin mode for motor
# "DC_STEER_THROTTLE" uses HBridge pwm to control one steering dc motor, and one drive wheel motor
# "DC_TWO_WHEEL" uses HBridge in 2-pin mode to control two drive motors, one on the left, and one on the right.
# "DC_TWO_WHEEL_L298N" using HBridge in 3-pin mode to control two drive motors, one of the left and one on the right.
# "MOCK" no drive train.  This can be used to test other features in a test rig.
# (deprecated) "SERVO_HBRIDGE_PWM" use ServoBlaster to output pwm control from the PiZero directly to control steering,
#                                  and HBridge for a drive motor.
# (deprecated) "PIGPIO_PWM" uses Raspberrys internal PWM
# (deprecated) "I2C_SERVO" uses PCA9685 servo controller to control a steering servo and an ESC, as in a standard RC car
#
DRIVE_TRAIN_TYPE = "PWM_STEERING_THROTTLE"

#
# PWM_STEERING_THROTTLE drivetrain configuration
#
# Drive train for RC car with a steering servo and ESC.
# Uses a PwmPin for steering (servo) and a second PwmPin for throttle (ESC)
# Base PWM Frequence is presumed to be 60hz; use PWM_xxxx_SCALE to adjust pulse with for non-standard PWM frequencies
#
PWM_STEERING_THROTTLE = {
    "PWM_STEERING_PIN": "PIGPIO.BCM.13",   # PWM output pin for steering servo
    "PWM_STEERING_SCALE": 1.0,              # used to compensate for PWM frequency differents from 60hz; NOT for adjusting steering range
    "PWM_STEERING_INVERTED": False,         # True if hardware requires an inverted PWM pulse
    "PWM_THROTTLE_PIN": "PIGPIO.BCM.18",   # PWM output pin for ESC
    "PWM_THROTTLE_SCALE": 1.0,              # used to compensate for PWM frequence differences from 60hz; NOT for increasing/limiting speed
    "PWM_THROTTLE_INVERTED": False,         # True if hardware requires an inverted PWM pulse
    "STEERING_LEFT_PWM": 380,               #pwm value for full left steering
    "STEERING_RIGHT_PWM": 285,              #pwm value for full right steering
    "THROTTLE_FORWARD_PWM": 305,            #pwm value for max forward throttle
    "THROTTLE_STOPPED_PWM": 260,            #pwm value for no movement
    "THROTTLE_REVERSE_PWM": 240,            #pwm value for max reverse throttle
}
# pwm values, use `donkey calibrate` to measure value for your car (donkey calibrate --pwm-pin={pin name})



#
# LIDAR
#
USE_LIDAR = True

LIDAR_LOWER_LIMIT = 190 # angles that will be recorded. Use this to block out obstructed areas on your car, or looking backwards.
LIDAR_UPPER_LIMIT = 170

LIDAR_SCAN_TIME = 1.0                   # maximum age of lidar scan values in seconds to be considered valid
LIDAR_MEASUREMENT_SPREAD = 8            # how many degrees to spread the measurements, for error prevention

LIDAR_MIN_LOOKAHEAD = 1.0

LIDAR_VALIDATION_TOLERANCES = {         # validation tolerances for testing
    0: 0.01,
    3000: 0.02,
    5000: 0.025
}


DESTINATION_CIRCLE_SIZE = 5.0   # radius in where to stop the car in meters

MAX_STEERING_ANGLE = 20         # maximum steering angle in degrees, measured manually 
CAR_WHEELBASE = 0.18            # distance between front and rear wheels in meters


#
# Input controllers
#
#WEB CONTROL
WEB_CONTROL_PORT = int(os.getenv("WEB_CONTROL_PORT", 8887))  # which port to listen on when making a web controller
WEB_INIT_MODE = "user"              # which control mode to start in. one of user|local_angle|local. Setting local will start in ai mode.

#JOYSTICK
USE_JOYSTICK_AS_DEFAULT = False      #when starting the manage.py, when True, will not require a --js option to use the joystick
JOYSTICK_MAX_THROTTLE = 0.5         #this scalar is multiplied with the -1 to 1 throttle value to limit the maximum throttle. This can help if you drop the controller or just don't need the full speed available.
JOYSTICK_STEERING_SCALE = 1.0       #some people want a steering that is less sensitve. This scalar is multiplied with the steering -1 to 1. It can be negative to reverse dir.
AUTO_RECORD_ON_THROTTLE = False     #if true, we will record whenever throttle is not zero. if false, you must manually toggle recording with some other trigger. Usually circle button on joystick.
CONTROLLER_TYPE = 'F710'            #(ps3|ps4|xbox|pigpio_rc|nimbus|wiiu|F710|rc3|MM1|custom) custom will run the my_joystick.py controller written by the `donkey createjs` command
USE_NETWORKED_JS = False            #should we listen for remote joystick control over the network?
NETWORK_JS_SERVER_IP = None         #when listening for network joystick control, which ip is serving this information
JOYSTICK_DEADZONE = 0.01            # when non zero, this is the smallest throttle before recording triggered.
JOYSTICK_THROTTLE_DIR = -1.0         # use -1.0 to flip forward/backward, use 1.0 to use joystick's natural forward/backward
JOYSTICK_DEVICE_FILE = "/dev/input/js0" # this is the unix file use to access the joystick.



#
# LOGGING
#
HAVE_CONSOLE_LOGGING = True
LOGGING_LEVEL = 'INFO'          # (Python logging level) 'NOTSET' / 'DEBUG' / 'INFO' / 'WARNING' / 'ERROR' / 'FATAL' / 'CRITICAL'
LOGGING_FORMAT = '%(message)s'  # (Python logging format - https://docs.python.org/3/library/logging.html#formatter-objects



#
# gps
#
HAVE_GPS = True             # True to read gps position
GPS_SERIAL = '/dev/ttyACM0' # serial device path, like '/dev/ttyAMA1' or '/dev/ttyUSB0'
GPS_SERIAL_BAUDRATE = 115200
GPS_NMEA_PATH = None        # File used to record gps, like "nmea.csv".
                            # If this is set then when waypoints are recorded then
                            # the underlying NMEA sentences will also be saved to
                            # this file along with their time stamps.  Then when
                            # the path is loaded and played in auto-pilot mode then
                            # the NMEA sentences that were recorded will be played back.
                            # This is for debugging and tuning the PID without having
                            # to keep driving the car.
GPS_DEBUG = False  # set to True to log UTM position (beware; lots of logging!)

PATH_DEBUG = False
PATH_SCALE = 10.0
PATH_OFFSET = (255, 255)

PID_P = -0.005                      # proportional mult for PID path follower
PID_I = 0.000                       # integral mult for PID path follower
PID_D = 0.000                       # differential mult for PID path follower

PID_D_DELTA = 0.25                  # amount the inc/dec function will change the D value
PID_P_DELTA = 0.25                  # amount the inc/dec function will change the P value


#
# Assign path follow functions to buttons.
# You can use game pad buttons OR web ui buttons ('web/w1' to 'web/w5')
# Use None use the game controller default
# NOTE: the cross button is already reserved for the emergency stop
#
SAVE_PATH_BTN = "circle"        # button to save path
LOAD_PATH_BTN = "x"             # button (re)load path
RESET_ORIGIN_BTN = "square"     # button to press to move car back to origin
ERASE_PATH_BTN = "triangle"     # button to erase path
TOGGLE_RECORDING_BTN = "option" # button to toggle recording mode
INC_PID_D_BTN = None            # button to change PID 'D' constant by PID_D_DELTA
DEC_PID_D_BTN = None            # button to change PID 'D' constant by -PID_D_DELTA
INC_PID_P_BTN = "R2"            # button to change PID 'P' constant by PID_P_DELTA
DEC_PID_P_BTN = "L2"            # button to change PID 'P' constant by -PID_P_DELTA

USE_SSD1306_128_32 = False    # Enable the SSD_1306 OLED Display
SSD1306_128_32_I2C_ROTATION = 0 # 0 = text is right-side up, 1 = rotated 90 degrees clockwise, 2 = 180 degrees (flipped), 3 = 270 degrees
SSD1306_RESOLUTION = 1 # 1 = 128x32; 2 = 128x64
SSD1306_128_32_I2C_BUSNUM = 1 # I2C bus number