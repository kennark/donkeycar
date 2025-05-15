#!/usr/bin/env python3
"""
Scripts to drive the car to a destination using GNSS coordinates.

Usage:
    manage.py (drive) [--js] [--log=INFO] [--camera=(single|stereo)]
 

Options:
    -h --help          Show this screen.
    --js               Use physical joystick.
    -f --file=<file>   A text file containing paths to tub files, one per line. Option may be used more than once.
    --meta=<key:value> Key/Value strings describing describing a piece of meta data about this drive. Option may be used more than once.

Starts in user mode.
 - User mode can drive manually using controller or web ui.
 - Set coordinates using web ui.

 - Turn on autopilot mode, and the car will drive to the destination.
 - Auto steer mode will set throttle to 0, steering will work.


"""

from distutils.log import debug
import os
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

import time

try:
    import cv2
except:
    pass


from docopt import docopt

import donkeycar as dk
from donkeycar.parts.controller import JoystickController
from donkeycar.parts.path import SinglePointDirection, SinglePointPilot, LidarCorrection
from donkeycar.parts.transform import PIDController
from donkeycar.parts.kinematics import SimpleBicycle
from donkeycar.templates.complete import add_camera, add_user_controller, add_drivetrain, DriveMode, UserPilotCondition
from donkeycar.parts.logger import LoggerPart
from donkeycar.parts.lidar import RPLidar2, ScanFilter, LidarValidator, LidarVisualizer

def drive(cfg, use_joystick=False, camera_type='single'):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag
    `threaded`.  All parts are updated one after another at the framerate given
    in cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely
    manner. Parts may have named outputs and inputs. The framework handles
    passing named outputs to parts requesting the same named input.
    '''

    #Initialize car
    V = dk.vehicle.Vehicle()
   
    #Initialize logging before anything else to allow console logging
    if cfg.HAVE_CONSOLE_LOGGING:
        logger.setLevel(logging.getLevelName(cfg.LOGGING_LEVEL))
        ch = logging.StreamHandler()
        ch.setFormatter(logging.Formatter(cfg.LOGGING_FORMAT))
        logger.addHandler(ch)

    #
    # gps outputs ['pos/x', 'pos/y']
    #
    add_gps(V, cfg)
    #V.add(GpsMock(), outputs=['pos/x', 'pos/y', 'gps/heading', 'gps/speed'])

    #
    # setup primary camera
    #
    add_camera(V, cfg, camera_type)

    #
    # add the user input controller(s)
    # - this will add the web controller
    # - it will optionally add any configured 'joystick' controller
    #
    ctr = add_user_controller(V, cfg, use_joystick)#, input_image = 'map/image')


    #
    # maintain run conditions for user mode and autopilot mode parts.
    #
    V.add(UserPilotCondition(),
          inputs=['user/mode', "cam/image_array", "cam/image_array"],
          outputs=['run_user', "run_pilot", "ui/image_array"])

    # This will find the closest direction to the destination
    direction_finder = SinglePointDirection()
    V.add(direction_finder, inputs=['pos/x', 'pos/y', 'destination'], outputs=["destination/ideal_direction", "destination/distance"], run_condition="run_pilot")

    pid = PIDController(p=cfg.PID_P, i=cfg.PID_I, d=cfg.PID_D)
    pilot = SinglePointPilot(pid, dest_circle=cfg.DESTINATION_CIRCLE_SIZE)
    V.add(pilot, inputs=['destination/ideal_direction', 'destination/distance', 'gps/heading', 'user/mode'], outputs=['pilot/steering', 'pilot/throttle'], run_condition="run_pilot")
    

    if cfg.USE_LIDAR:
        lidar = RPLidar2(min_angle=cfg.LIDAR_LOWER_LIMIT, max_angle=cfg.LIDAR_UPPER_LIMIT)
        V.add(lidar, outputs=['lidar/measurements'], threaded=True)

        filt = ScanFilter(min_angle=cfg.LIDAR_LOWER_LIMIT, max_angle=cfg.LIDAR_UPPER_LIMIT, measurement_spread = cfg.LIDAR_MEASUREMENT_SPREAD, time_window=cfg.LIDAR_SCAN_TIME)
        V.add(filt, inputs=['lidar/measurements'], outputs=['lidar/filtered_measurements'])

        #V.add(LidarValidator(tolerances=cfg.LIDAR_VALIDATION_TOLERANCES), inputs=['lidar/filtered_measurements'])
        #V.add(LidarVisualizer(resolution=5), inputs=['lidar/filtered_measurements'])

        bicycle = SimpleBicycle(wheelbase=cfg.CAR_WHEELBASE, max_steering_angle=cfg.MAX_STEERING_ANGLE)

        lidar_correction = LidarCorrection(bicycle=bicycle, max_steering_degree=cfg.MAX_STEERING_ANGLE, min_lookahead=cfg.LIDAR_MIN_LOOKAHEAD)
        V.add(lidar_correction, inputs=['pilot/steering', 'pilot/throttle', 'lidar/filtered_measurements', 'gps/speed'], outputs=['pilot/steering', 'pilot/throttle'], run_condition="run_pilot")
    

    #
    # Decide what inputs should change the car's steering and throttle
    # based on the choice of user or autopilot drive mode
    #
    V.add(DriveMode(),
          inputs=['user/mode', 'user/steering', 'user/throttle',
                  'pilot/steering', 'pilot/throttle'],
          outputs=['steering', 'throttle'])

    # V.add(LoggerPart(['user/mode', 'steering', 'throttle'], logger="drivemode"), inputs=['user/mode', 'steering', 'throttle'])

    #
    # Setup drivetrain
    #
    add_drivetrain(V, cfg)


    #
    # OLED display setup
    #
    if cfg.USE_SSD1306_128_32:
        from donkeycar.parts.oled import OLEDPart
        auto_record_on_throttle = cfg.USE_JOYSTICK_AS_DEFAULT and cfg.AUTO_RECORD_ON_THROTTLE
        oled_part = OLEDPart(cfg.SSD1306_128_32_I2C_ROTATION, cfg.SSD1306_RESOLUTION, auto_record_on_throttle)
        V.add(oled_part, inputs=['recording', 'tub/num_records', 'user/mode'], outputs=[], threaded=True)


    # Print Joystick controls
    if ctr is not None and isinstance(ctr, JoystickController):
        ctr.print_controls()


    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, 
        max_loop_count=cfg.MAX_LOOPS)


def add_gps(V, cfg):
    if cfg.HAVE_GPS:
        from donkeycar.parts.serial_port import SerialPort, SerialLineReader
        from donkeycar.parts.gps import GpsNmeaPositions, GpsLatestPosition, GpsNmeaHeadingAndSpeed, GpsLatestHeadingAndSpeed
        from donkeycar.parts.pipe import Pipe

        #
        # parts to
        # - read nmea lines from serial port
        # - convert nmea lines to positions
        # - retrieve the most recent position
        #
        serial_port = SerialPort(cfg.GPS_SERIAL, cfg.GPS_SERIAL_BAUDRATE)
        nmea_reader = SerialLineReader(serial_port)
        V.add(nmea_reader, outputs=['gps/nmea'], threaded=True)

        gps_positions = GpsNmeaPositions(debug=cfg.GPS_DEBUG)
        V.add(gps_positions, inputs=['gps/nmea'], outputs=['gps/positions'])
        gps_latest_position = GpsLatestPosition(debug=cfg.GPS_DEBUG)
        V.add(gps_latest_position, inputs=['gps/positions'], outputs=['gps/timestamp', 'gps/utm/longitude', 'gps/utm/latitude'])

        gps_heading = GpsNmeaHeadingAndSpeed(debug=cfg.GPS_DEBUG)
        V.add(gps_heading, inputs=['gps/nmea'], outputs=['gps/heading'])
        gps_latest_heading = GpsLatestHeadingAndSpeed(debug=cfg.GPS_DEBUG)
        V.add(gps_latest_heading, inputs=['gps/heading'], outputs=['gps/timestamp', 'gps/heading', 'gps/speed'])

        # rename gps utm position to pose values
        V.add(Pipe(), inputs=['gps/utm/longitude', 'gps/utm/latitude'], outputs=['pos/x', 'pos/y'])
    


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()

    log_level = args['--log'] or "INFO"
    numeric_level = getattr(logging, log_level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % log_level)
    logging.basicConfig(level=numeric_level)


    if args['drive']:
        drive(cfg, use_joystick=args['--js'], camera_type=args['--camera'])
