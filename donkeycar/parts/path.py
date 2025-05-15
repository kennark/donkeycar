import pickle
import math
import logging
import pathlib
import numpy
from PIL import Image, ImageDraw

from donkeycar.parts.transform import PIDController
from donkeycar.utils import norm_deg, dist, deg2rad, arr_to_img, is_number_type


class AbstractPath:
    def __init__(self, min_dist=1.):
        self.path = []  # list of (x,y) tuples
        self.min_dist = min_dist
        self.x = math.inf
        self.y = math.inf

    def run(self, recording, x, y):
        if recording:
            d = dist(x, y, self.x, self.y)
            if d > self.min_dist:
                logging.info(f"path point ({x},{y})")
                self.path.append((x, y))
                self.x = x
                self.y = y
        return self.path

    def length(self):
        return len(self.path)

    def is_empty(self):
        return 0 == self.length()

    def is_loaded(self):
        return not self.is_empty()

    def get_xy(self):
        return self.path

    def reset(self):
        self.path = []
        return True

    def save(self, filename):
        return False

    def load(self, filename):
        return False


class CsvPath(AbstractPath):
    def __init__(self, min_dist=1.):
        super().__init__(min_dist)

    def save(self, filename):
        if self.length() > 0:
            with open(filename, 'w') as outfile:
                for (x, y) in self.path:
                    outfile.write(f"{x}, {y}\n")
            return True
        else:
            return False

    def load(self, filename):
        path = pathlib.Path(filename)
        if path.is_file():
            with open(filename, "r") as infile:
                self.path = []
                for line in infile:
                    xy = [float(i.strip()) for i in line.strip().split(sep=",")]
                    self.path.append((xy[0], xy[1]))
            return True
        else:
            logging.info(f"File '{filename}' does not exist")
            return False

        self.recording = False

class CsvThrottlePath(AbstractPath):
    def __init__(self, min_dist: float = 1.0) -> None:
        super().__init__(min_dist)
        self.throttles = []

    def run(self, recording: bool, x: float, y: float, throttle: float) -> tuple:
        if recording:
            d = dist(x, y, self.x, self.y)
            if d > self.min_dist:
                logging.info(f"path point: ({x},{y}) throttle: {throttle}")
                self.path.append((x, y))
                self.throttles.append(throttle)
                self.x = x
                self.y = y
        return self.path, self.throttles

    def reset(self) -> bool:
        super().reset()
        self.throttles = []
        return True

    def save(self, filename: str) -> bool:
        if self.length() > 0:
            with open(filename, 'w') as outfile:
                for (x, y), v in zip(self.path, self.throttles):
                    outfile.write(f"{x}, {y}, {v}\n")
            return True
        else:
            return False

    def load(self, filename: str) -> bool:
        path = pathlib.Path(filename)
        if path.is_file():
            with open(filename, "r") as infile:
                self.path = []
                for line in infile:
                    xy = [float(i.strip()) for i in line.strip().split(sep=",")]
                    self.path.append((xy[0], xy[1]))
                    self.throttles.append(xy[2])
            return True
        else:
            logging.warning(f"File '{filename}' does not exist")
            return False


class RosPath(AbstractPath):
    def __init__(self, min_dist=1.):
        super().__init__(self, min_dist)

    def save(self, filename):
        outfile = open(filename, 'wb')
        pickle.dump(self.path, outfile)
        return True

    def load(self, filename):
        infile = open(filename, 'rb')
        self.path = pickle.load(infile)
        self.recording = False
        return True

class PImage(object):
    def __init__(self, resolution=(500, 500), color="white", clear_each_frame=False):
        self.resolution = resolution
        self.color = color
        self.img = Image.new('RGB', resolution, color=color)
        self.clear_each_frame = clear_each_frame

    def run(self):
        if self.clear_each_frame:
            self.img = Image.new('RGB', self.resolution, color=self.color)

        return self.img


class OriginOffset(object):
    '''
    Use this to set the car back to the origin without restarting it.
    '''

    def __init__(self, debug=False):
        self.debug = debug
        self.ox = 0.0
        self.oy = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.reset = None

    def run(self, x, y, closest_pt):
        """
        :param x: is current horizontal position
        :param y: is current vertical position
        :param closest_pt: is current cte/closest_pt
        :return: translated x, y and new index of closest point in path.
        """
        if is_number_type(x) and is_number_type(y):
            # if origin is None, set it to current position
            if self.reset:
                self.ox = x
                self.oy = y

            self.last_x = x
            self.last_y = y
        else:
            logging.debug("OriginOffset ignoring non-number")

        # translate the given position by the origin
        pos = (0, 0)
        if self.last_x is not None and self.last_y is not None and self.ox is not None and self.oy is not None:
            pos = (self.last_x - self.ox, self.last_y - self.oy)
        if self.debug:
            logging.info(f"pos/x = {pos[0]}, pos/y = {pos[1]}")

        # reset the starting search index for cte algorithm
        if self.reset:
            if self.debug:
                logging.info(f"cte/closest_pt = {closest_pt} -> None")
            closest_pt = None

        # clear reset latch
        self.reset = False

        return pos[0], pos[1], closest_pt

    def set_origin(self, x, y):
        logging.info(f"Resetting origin to ({x}, {y})")
        self.ox = x
        self.oy = y

    def reset_origin(self):
        """
        Reset the origin with the next value that comes in
        """
        self.ox = None
        self.oy = None
        self.reset = True

    def init_to_last(self):
        self.set_origin(self.last_x, self.last_y)


class PathPlot(object):
    '''
    draw a path plot to an image
    '''
    def __init__(self, scale=1.0, offset=(0., 0.0)):
        self.scale = scale
        self.offset = offset

    def plot_line(self, sx, sy, ex, ey, draw, color):
        '''
        scale dist so that max_dist is edge of img (mm)
        and img is PIL Image, draw the line using the draw ImageDraw object
        '''
        draw.line((sx,sy, ex, ey), fill=color, width=1)

    def run(self, img, path):
        
        if type(img) is numpy.ndarray:
            stacked_img = numpy.stack((img,)*3, axis=-1)
            img = arr_to_img(stacked_img)

        if path:
            draw = ImageDraw.Draw(img)
            color = (255, 0, 0)
            for iP in range(0, len(path) - 1):
                ax, ay = path[iP]
                bx, by = path[iP + 1]

                #
                # y increases going north, so handle this with scale
                #
                self.plot_line(ax * self.scale + self.offset[0],
                            ay * -self.scale + self.offset[1],
                            bx * self.scale + self.offset[0],
                            by * -self.scale + self.offset[1],
                            draw,
                            color)
        return img


class PlotCircle(object):
    '''
    draw a circle plot to an image
    '''
    def __init__(self,  scale=1.0, offset=(0., 0.0), radius=4, color = (0, 255, 0)):
        self.scale = scale
        self.offset = offset
        self.radius = radius
        self.color = color

    def plot_circle(self, x, y, rad, draw, color, width=1):
        '''
        scale dist so that max_dist is edge of img (mm)
        and img is PIL Image, draw the circle using the draw ImageDraw object
        '''
        sx = x - rad
        sy = y - rad
        ex = x + rad
        ey = y + rad

        draw.ellipse([(sx, sy), (ex, ey)], fill=color)


    def run(self, img, x, y):
        draw = ImageDraw.Draw(img)
        self.plot_circle(x * self.scale + self.offset[0],
                        y * -self.scale + self.offset[1],  # y increases going north
                        self.radius,
                        draw, 
                        self.color)

        return img

from donkeycar.la import Line3D, Vec3

class CTE(object):

    def __init__(self, look_ahead=1, look_behind=1, num_pts=None) -> None:
        self.num_pts = num_pts
        self.look_ahead = look_ahead
        self.look_behind = look_behind

    #
    # Find the index of the path element with minimal distance to (x,y).
    # This prefers the first element with the minimum distance if there
    # are more then one.
    #
    def nearest_pt(self, path, x, y, from_pt=0, num_pts=None):
        from_pt = from_pt if from_pt is not None else 0
        num_pts = num_pts if num_pts is not None else len(path)
        num_pts = min(num_pts, len(path))
        if num_pts < 0:
            logging.error("num_pts must not be negative.")
            return None, None, None

        min_pt = None
        min_dist = None
        min_index = None
        for j in range(num_pts):
            i = (j + from_pt) % len(path)
            p = path[i]
            d = dist(p[0], p[1], x, y)
            if min_dist is None or d < min_dist:
                min_pt = p
                min_dist = d
                min_index = i
        return min_pt, min_index, min_dist


    # TODO: update so that we look for nearest two points starting from a given point
    #       and up to a given number of points.  This will speed things up
    #       but more importantly it can be used to handle crossing paths.
    def nearest_two_pts(self, path, x, y):
        if path is None or len(path) < 2:
            logging.error("path is none; cannot calculate nearest points")
            return None, None

        distances = []
        for iP, p in enumerate(path):
            d = dist(p[0], p[1], x, y)
            distances.append((d, iP, p))
        distances.sort(key=lambda elem : elem[0])

        # get the prior point as start of segment
        iA = (distances[0][1] - 1) % len(path)
        a = path[iA]

        # get the next point in the path as the end of the segment
        iB = (iA + 2) % len(path)
        b = path[iB]
        
        return a, b

    def nearest_waypoints(self, path, x, y, look_ahead=1, look_behind=1, from_pt=0, num_pts=None):
        """
        Get the path elements around the closest element to the given (x,y)
        :param path: list of (x,y) points
        :param x: horizontal coordinate of point to check
        :param y: vertical coordinate of point to check
        :param from_pt: index start start search within path
        :param num_pts: maximum number of points to search in path
        :param look_ahead: number waypoints to include ahead of nearest point.
        :param look_behind: number of waypoints to include behind nearest point.
        :return: index of first point, nearest point and last point in nearest path segments
        """
        if path is None or len(path) < 2:
            logging.error("path is none; cannot calculate nearest points")
            return None, None, None

        if look_ahead < 0:
            logging.error("look_ahead must be a non-negative number")
            return None, None, None
        if look_behind < 0:
            logging.error("look_behind must be a non-negative number")
            return None, None, None
        if (look_ahead + look_behind) > len(path):
            logging.error("the path is not long enough to supply the waypoints")
            return None, None, None

        _pt, i, _distance = self.nearest_pt(path, x, y, from_pt, num_pts)

        # get  start of segment
        a = (i + len(path) - look_behind) % len(path)

        # get the end of the segment
        b = (i + look_ahead) % len(path)

        return a, i, b

    def nearest_track(self, path, x, y, look_ahead=1, look_behind=1, from_pt=0, num_pts=None):
        """
        Get the line segment around the closest point to the given (x,y)
        :param path: list of (x,y) points
        :param x: horizontal coordinate of point to check
        :param y: vertical coordinate of point to check
        :param from_pt: index start start search within path
        :param num_pts: maximum number of points to search in path
        :param look_ahead: number waypoints to include ahead of nearest point.
        :param look_behind: number of waypoints to include behind nearest point.
        :return: start and end points of the nearest track and index of nearest point
        """

        a, i, b = self.nearest_waypoints(path, x, y, look_ahead, look_behind, from_pt, num_pts)

        return (path[a], path[b], i) if a is not None and b is not None else (None, None, None)

    def run(self, path, x, y, from_pt=None):
        """
        Run cross track error algorithm
        :return: cross-track-error and index of nearest point on the path
        """
        cte = 0.
        i = from_pt

        a, b, i = self.nearest_track(path, x, y, 
                                     look_ahead=self.look_ahead, look_behind=self.look_behind, 
                                     from_pt=from_pt, num_pts=self.num_pts)
        
        if a and b:
            logging.info(f"nearest: ({a[0]}, {a[1]}) to ({x}, {y})")
            a_v = Vec3(a[0], 0., a[1])
            b_v = Vec3(b[0], 0., b[1])
            p_v = Vec3(x, 0., y)
            line = Line3D(a_v, b_v)
            err = line.vector_to(p_v)
            sign = 1.0
            cp = line.dir.cross(err.normalized())
            if cp.y > 0.0 :
                sign = -1.0
            cte = err.mag() * sign            
        else:
            logging.info(f"no nearest point to ({x},{y}))")
        return cte, i


class PID_Pilot(object):

    def __init__(
            self,
            pid: PIDController,
            throttle: float,
            use_constant_throttle: bool = False,
            min_throttle: float = None) -> None:
        self.pid = pid
        self.throttle = throttle
        self.use_constant_throttle = use_constant_throttle
        self.variable_speed_multiplier = 1.0
        self.min_throttle = min_throttle if min_throttle is not None else throttle

    def run(self, cte: float, throttles: list, closest_pt_idx: int) -> tuple:
        steer = self.pid.run(cte)
        if self.use_constant_throttle or throttles is None or closest_pt_idx is None:
            throttle = self.throttle
        elif throttles[closest_pt_idx] * self.variable_speed_multiplier < self.min_throttle:
            throttle = self.min_throttle
        else:
            throttle = throttles[closest_pt_idx] * self.variable_speed_multiplier
        logging.info(f"CTE: {cte} steer: {steer} throttle: {throttle}")
        return steer, throttle


class SinglePointDirection(object):
    def __init__(self) -> None:
        self.point = []
        self.direction = 0.0


    def run(self, x, y, dest):
        if len(dest) == 0:
            logging.error("Destination coordinates are missing")
            return None, None

        if x == 0.0 or y == 0.0:
            logging.error("Own coordinates are missing")
            return None

        # Calculates the ideal direction from the current x, y to destination x, y
        own_coords = (x, y)

        vect_x = dest[0] - own_coords[0]
        vect_y = dest[1] - own_coords[1]

        angle_radian = math.atan2(vect_x, vect_y)
        angle_degrees = math.degrees(angle_radian)

        distance = math.sqrt(vect_x * vect_x + vect_y * vect_y)

        return angle_degrees, distance


class SinglePointPilot(object):
    def __init__(self, pid: PIDController, dest_circle=5.0, throttle=0.8):
        self.pid = pid
        self.dest_circle = dest_circle
        self.throttle = throttle

    def run(self, ideal_direction, distance, curr_heading, mode):
        if curr_heading is None:
            logging.error("Current heading is missing")
            return None, None

        if ideal_direction is None:
            logging.error("Destination direction is missing")
            return None, None

        error = (ideal_direction - curr_heading + 180) % 360 - 180 # Error from target direction

        steer = self.pid.run(error)
        steer = max(-1, min(1, steer)) # Limit steer to [-1, 1]
        throttle = self.throttle

        if distance < self.dest_circle:
            logging.info("Reached destination point")
            throttle = 0
        elif mode == 'local_angle':
            throttle = 0
        return steer, throttle

class LidarCorrection(object):
    def __init__(self, bicycle, max_steering_degree=20, min_lookahead=1.0):
        self.max_steering_degree = max_steering_degree
        self.bicycle = bicycle

        self.min_lookahead = min_lookahead


    def run(self, curr_steer, curr_throttle, lidar_scan, curr_speed):
        if curr_steer is None or curr_throttle is None:
            return None, None

        if lidar_scan is None or len(lidar_scan) == 0:
            logging.warning("Lidar data is missing")
            return curr_steer, curr_throttle

        current_steering_angle = self.max_steering_degree * curr_steer
        curr_speed = curr_speed * 0.277778 # convert to m/s

        resulting_steer = curr_steer

        curr_speed = max(curr_speed, self.min_lookahead) # for low speeds, increase the future movement calculation distance
        multiplier = 1 - 0.4 * abs(curr_steer)
        calc_speed = curr_speed * multiplier # for high steering angles, calculate the future movenent with a lower speed value

        distance_after_1s_from_start, angle_deg_after_1s_from_start = self.bicycle.run(curr_steer, calc_speed)

        if lidar_scan[int(angle_deg_after_1s_from_start)] is None or lidar_scan[int(angle_deg_after_1s_from_start)] < distance_after_1s_from_start:
            # if the distance is too short, we need to steer away from it
            best_deviation = float("inf")
            for angle in range(-self.max_steering_degree, self.max_steering_degree + 1):
                normalized_steer = angle / self.max_steering_degree
                calc_speed = curr_speed * (1 - 0.4 * abs(normalized_steer))
                alt_distance, alt_angle = self.bicycle.run(normalized_steer, calc_speed)

                if lidar_scan[int(alt_angle)] is not None and lidar_scan[int(alt_angle)] > alt_distance:
                    deviation = (alt_angle - angle_deg_after_1s_from_start + 180) % 360 - 180
                    if abs(deviation) < best_deviation:
                        best_deviation = abs(angle - current_steering_angle)
                        resulting_steer = normalized_steer

            # No available movement in any direction (based on the calculated future movement)
            if best_deviation == float("inf") or lidar_scan[0] < 400: # less than 40 cm free space in front: stop the car
                logging.info("Stopping the car!! Remove obstacle in front")
                return 0, 0

        resulting_steer = max(-1, min(1, resulting_steer))

        return resulting_steer, curr_throttle