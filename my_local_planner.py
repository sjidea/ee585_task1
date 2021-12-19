#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
This module contains a local planner to perform
low-level waypoint following based on PID controllers.
"""

from collections import deque
import rospy
import math
import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from carla_waypoint_types.srv import GetWaypoint
from carla_msgs.msg import CarlaEgoVehicleControl
from vehicle_pid_controller import VehiclePIDController  # pylint: disable=relative-import
from misc import distance_vehicle, compute_magnitude_angle  # pylint: disable=relative-import
import carla
import carla_ros_bridge.transforms as trans

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    import frenet_optimal_trajectory 
except ImportError:
    raise

class Obstacle:
    def __init__(self):
        self.id = -1 # actor id
        self.vx = 0.0 # velocity in x direction
        self.vy = 0.0 # velocity in y direction
        self.vz = 0.0 # velocity in z direction
        self.ros_transform = None # transform of the obstacle in ROS coordinate
        self.carla_transform = None # transform of the obstacle in Carla world coordinate
        self.bbox = None # Bounding box w.r.t ego vehicle's local frame

class MyLocalPlanner(object):
    """
    LocalPlanner implements the basic behavior of following a trajectory of waypoints that is
    generated on-the-fly. The low-level motion of the vehicle is computed by using two PID
    controllers, one is used for the lateral control and the other for the longitudinal
    control (cruise speed).

    When multiple paths are available (intersections) this local planner makes a random choice.
    """

    # minimum distance to target waypoint as a percentage (e.g. within 90% of
    # total distance)
    MIN_DISTANCE_PERCENTAGE = 0.9

    def __init__(self, role_name, opt_dict=None):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param role_name: name of the actor
        :param opt_dict: dictionary of arguments with the following semantics:

            target_speed -- desired cruise speed in Km/h

            sampling_radius -- search radius for next waypoints in seconds: e.g. 0.5 seconds ahead

            lateral_control_dict -- dictionary of arguments to setup the lateral PID controller
                                    {'K_P':, 'K_D':, 'K_I'}

            longitudinal_control_dict -- dictionary of arguments to setup the longitudinal
                                         PID controller
                                         {'K_P':, 'K_D':, 'K_I'}
        """
        self.target_route_point = None
        self._current_waypoint = None
        self._vehicle_controller = None
        self._waypoints_queue = deque(maxlen=20000)
        self._buffer_size = 5
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        self._vehicle_yaw = None
        self._current_speed = None
        self._current_pose = None
        self._obstacles = []

        # get world and map for finding actors and waypoints
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        self.world = client.get_world()
        self.map = self.world.get_map()        

        self._target_point_publisher = rospy.Publisher(
            "/next_target", PointStamped, queue_size=1)

        rospy.wait_for_service('/carla_waypoint_publisher/{}/get_waypoint'.format(role_name))
        self._get_waypoint_client = rospy.ServiceProxy(
            '/carla_waypoint_publisher/{}/get_waypoint'.format(role_name), GetWaypoint)

        # initializing controller
        self._init_controller(opt_dict)

        try:
            self.csp = []
        except:
            print('cannot make csp as list')

        try:
            self.c_speed = 0.0
            self.c_d = 0.0
            self.c_d_d = 0.0
            self.c_d_dd = 0.0
            self.s0 = 0.0
        except:
            print('cannot make initial state')
        self.waypoint_list = []
        
        

    def get_obstacles(self, location, range):
        """
        Get a list of obstacles that are located within a certain distance from the location.
        
        :param      location: queried location # current pose position
        :param      range: search distance from the queried location
        :type       location: geometry_msgs/Point
        :type       range: float or double
        :return:    None
        :rtype:     None
        """
        self._obstacles = []
        actor_list = self.world.get_actors()
        for actor in actor_list:
            if "role_name" in actor.attributes:
                if actor.attributes["role_name"] == 'autopilot' or actor.attributes["role_name"] == "static":
                    carla_transform = actor.get_transform()
                    ros_transform = trans.carla_transform_to_ros_pose(carla_transform)
                    x = ros_transform.position.x
                    y = ros_transform.position.y
                    z = ros_transform.position.z 
                    distance = math.sqrt((x-location.x)**2 + (y-location.y)**2)
                    if distance < range:
                        # print("obs distance: {}").format(distance)
                        ob = Obstacle()
                        ob.id = actor.id
                        ob.carla_transform = carla_transform
                        ob.ros_transform = ros_transform
                        ob.vx = actor.get_velocity().x
                        ob.vy = actor.get_velocity().y
                        ob.vz = actor.get_velocity().z
                        ob.bbox = actor.bounding_box # in local frame
                        # print("x: {}, y: {}, z:{}").format(x, y, z)
                        # print("bbox x:{} y:{} z:{} ext: {} {} {}".format(ob.bbox.location.x, ob.bbox.location.y, ob.bbox.location.z, ob.bbox.extent.x, ob.bbox.extent.y, ob.bbox.extent.z))
                        self._obstacles.append(ob)

    def get_obstacles_for_speedup(self, location, range):
        obstacles = []
        actor_list = self.world.get_actors()
        distance = []
        for actor in actor_list:
            if "role_name" in actor.attributes:
                if actor.attributes["role_name"] == 'autopilot' or actor.attributes["role_name"] == "static":
                    carla_transform = actor.get_transform()
                    ros_transform = trans.carla_transform_to_ros_pose(carla_transform)
                    x = ros_transform.position.x
                    y = ros_transform.position.y
                    z = ros_transform.position.z 


                    current_waypoint = self.get_waypoint(location)
                    waypoint_xodr = self.map.get_waypoint_xodr(current_waypoint.road_id, current_waypoint.lane_id, current_waypoint.s)
                    (_ , angle) = compute_magnitude_angle(ros_transform.position, location, \
                                                                 - waypoint_xodr.transform.rotation.yaw * math.pi / 180.0)
                    if (angle < -math.pi/2 ) or (angle > math.pi/2):
                        distance.append(math.sqrt((x-location.x)**2 + (y-location.y)**2) < range)
        return not any(distance )

    def check_obstacle(self, point, obstacle):
        """
        Check whether a point is inside the bounding box of the obstacle

        :param      point: a location to check the collision (in ROS frame)
        :param      obstacle: an obstacle for collision check
        :type       point: geometry_msgs/Point
        :type       obstacle: object Obstacle
        :return:    true or false
        :rtype:     boolean   
        """
        carla_location = carla.Location()
        carla_location.x = point.x
        carla_location.y = -point.y
        carla_location.z = point.z
        
        vertices = obstacle.bbox.get_world_vertices(obstacle.carla_transform)
        
        vx = [v.x for v in vertices]
        vy = [v.y for v in vertices]
        vz = [v.z for v in vertices]
        return carla_location.x >= min(vx) and carla_location.x <= max(vx) \
                and carla_location.y >= min(vy) and carla_location.y <= max(vy) \
                and carla_location.z >= min(vz) and carla_location.z <= max(vz) 

    def get_coordinate_lanemarking(self, position):
        """
        Helper to get adjacent waypoint 2D coordinates of the left and right lane markings 
        with respect to the closest waypoint
        
        :param      position: queried position
        :type       position: geometry_msgs/Point
        :return:    left and right waypoint in numpy array
        :rtype:     tuple of geometry_msgs/Point (left), geometry_msgs/Point (right)
        """
        # get waypoints along road
        current_waypoint = self.get_waypoint(position)
        waypoint_xodr = self.map.get_waypoint_xodr(current_waypoint.road_id, current_waypoint.lane_id, current_waypoint.s)
        
        # find two orthonormal vectors to the direction of the lane
        yaw = math.pi - waypoint_xodr.transform.rotation.yaw * math.pi / 180.0
        v = np.array([1.0, math.tan(yaw)])
        norm_v = v / np.linalg.norm(v)
        right_v = np.array([-norm_v[1], norm_v[0]])
        left_v = np.array([norm_v[1], -norm_v[0]])
        
        # find two points that are on the left and right lane markings
        half_width = current_waypoint.lane_width / 2.0
        left_waypoint = np.array([current_waypoint.pose.position.x, current_waypoint.pose.position.y]) + half_width * left_v
        right_waypoint = np.array([current_waypoint.pose.position.x, current_waypoint.pose.position.y]) + half_width * right_v
        ros_left_waypoint = Point()
        ros_right_waypoint = Point()
        ros_left_waypoint.x = left_waypoint[0]
        ros_left_waypoint.y = left_waypoint[1]
        ros_right_waypoint.x = right_waypoint[0]
        ros_right_waypoint.y = right_waypoint[1]
        return ros_left_waypoint, ros_right_waypoint

    def get_waypoint(self, location):
        """
        Helper to get waypoint from a ros service
        """
        try:
            response = self._get_waypoint_client(location)
            return response.waypoint
        except (rospy.ServiceException, rospy.ROSInterruptException) as e:
            if not rospy.is_shutdown:
                rospy.logwarn("Service call failed: {}".format(e))

    def odometry_updated(self, odo):
        """
        Callback on new odometry
        """
        self._current_speed = math.sqrt(odo.twist.twist.linear.x ** 2 +
                                        odo.twist.twist.linear.y ** 2 +
                                        odo.twist.twist.linear.z ** 2) * 3.6

        self._current_pose = odo.pose.pose
        quaternion = (
            odo.pose.pose.orientation.x,
            odo.pose.pose.orientation.y,
            odo.pose.pose.orientation.z,
            odo.pose.pose.orientation.w
        )
        _, _, self._vehicle_yaw = euler_from_quaternion(quaternion)

    def _init_controller(self, opt_dict):
        """
        Controller initialization.

        :param opt_dict: dictionary of arguments.
        :return:
        """
        # default params
        args_lateral_dict = {
            'K_P': 1.95,
            'K_D': 0.01,
            'K_I': 1.4}
        args_longitudinal_dict = {
            'K_P': 0.2,
            'K_D': 0.05,
            'K_I': 0.1}

        # parameters overload
        if opt_dict:
            if 'lateral_control_dict' in opt_dict:
                args_lateral_dict = opt_dict['lateral_control_dict']
            if 'longitudinal_control_dict' in opt_dict:
                args_longitudinal_dict = opt_dict['longitudinal_control_dict']

        self._vehicle_controller = VehiclePIDController(args_lateral=args_lateral_dict,
                                                        args_longitudinal=args_longitudinal_dict)

    def set_global_plan(self, current_plan):
        """
        set a global plan to follow
        """
        self.target_route_point = None
        self._waypoint_buffer.clear()
        self._waypoints_queue.clear()
        x=[]
        y=[]
        for i, elem in enumerate(current_plan):
 
            '''to eliminate repeted points'''
            if self._waypoints_queue: # waypoints_queue is not empty
                prev_ = self._waypoints_queue[-1].position
                curr_ = elem.pose.position
                if not ((prev_.x == curr_.x) and (prev_.y == curr_.y)):
                    self.waypoint_list.append([curr_.x, curr_.y])
                    x.append(curr_.x)
                    y.append(curr_.y)
            '''end eliminating repeated points'''

            self._waypoints_queue.append(elem.pose)

        waypoint_np = np.array(self.waypoint_list).T
        self.waypoint_list = waypoint_np.tolist()
        # print("x = {}".format(len(self.waypoint_list[0])))
        # print("y = {}".format(len(self.waypoint_list[1])))

        try:
            self.csp = frenet_optimal_trajectory.generate_target_course(x, y)
        except:
            print('cannot make csp')

        try:
            self.c_speed = 10.0 / 3.6
            self.c_d = 2.0
            self.c_d_d = 0.0
            self.c_d_dd = 0.0
            self.s0 = 0.0
        except:
            print('cannot assign initial state values')

    def run_step(self, target_speed, current_speed, current_pose): # 
        """
        Execute one step of local planning which involves running the longitudinal
        and lateral PID controllers to follow the waypoints trajectory.
        """
        if not self._waypoint_buffer and not self._waypoints_queue:
            control = CarlaEgoVehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 0.5
            control.hand_brake = False
            control.manual_gear_shift = False

            rospy.loginfo("Route finished.")
            return control, True

        #   Buffering the waypoints
        if not self._waypoint_buffer:
            for i in range(self._buffer_size):
                if self._waypoints_queue:
                    self._waypoint_buffer.append(
                        self._waypoints_queue.popleft())
                    # print("lets see the result of popleft = {}".format(self._waypoint_buffer[-1]))
                else:
                    break

        # current vehicle waypoint
        self._current_waypoint = self.get_waypoint(current_pose.position)

        # get a list of obstacles surrounding the ego vehicle
        self.get_obstacles(current_pose.position, 70.0)


        # # Example 1: get two waypoints on the left and right lane marking w.r.t current pose
        # left, right = self.get_coordinate_lanemarking(current_pose.position)
        # print("\x1b[6;30;33m------Example 1------\x1b[0m")
        # print("Left: {}, {}; right: {}, {}".format(left.x, left.y, right.x, right.y))
        
        # # Example 2: check obstacle collision
        # print("\x1b[6;30;33m------Example 2------\x1b[0m")
        # point = Point()
        # point.x = 100.0
        # point.y = 100.0
        # point.z = 1.5
        # for ob in self._obstacles:
        #     print("id: {}, collision: {}".format(ob.id, self.check_obstacle(point, ob)))
        
        
        # check if ob is ok
        obs = [] 
        try:
            for ob in self._obstacles:
                ros_transform = trans.carla_transform_to_ros_pose(ob.carla_transform)
                # print("ob.bbox.location.x = {}".format(ob.bbox.location.x))
                obs.append([ros_transform.position.x, ros_transform.position.y])
        except:    
            print('ob is the problem')
        obs = np.array(obs)
        # print("obstacle position {}", obs)
        
        # if self.get_obstacles_for_speedup(current_pose.position, 120) and not self.get_obstacles_for_speedup(current_pose.position, 100):
        #     target_speed = 50
        # else:
        #     target_speed = 30
        # target_speed = 50
        # target_speed = 30

        try:
            path = frenet_optimal_trajectory.frenet_optimal_planning( \
                            self.csp, self.s0, self.c_speed, self.c_d, self.c_d_d, self.c_d_dd, \
                            obs)
        except:
            print('cannot get frenet path')
        # print("path = {}".format(path))

        # target waypoint        
        self.target_route_point = self._waypoint_buffer[0]
        # if path:
        self.target_route_point.position.x = path.x[1]
        self.target_route_point.position.y = path.y[1]
        target_point = PointStamped()
        target_point.header.frame_id = "map"
        target_point.point.x = self.target_route_point.position.x
        target_point.point.y = self.target_route_point.position.y
        target_point.point.z = self.target_route_point.position.z
        self._target_point_publisher.publish(target_point)

        # ''' 

        
        # print("target_speed = {}".format(target_speed))
        # '''
        
        # move using PID controllers
        control = self._vehicle_controller.run_step(
            target_speed, current_speed, current_pose, self.target_route_point)



        # purge the queue of obsolete waypoints
        max_index = -1

        sampling_radius = target_speed * 1 / 3.6  # 1 seconds horizon
        min_distance = sampling_radius * self.MIN_DISTANCE_PERCENTAGE

        for i, route_point in enumerate(self._waypoint_buffer):
            if distance_vehicle(
                    route_point, current_pose.position) < min_distance:
                max_index = i
        if max_index >= 0:
            for i in range(max_index + 1):
                self._waypoint_buffer.popleft()

        update_path = -1
        try:
            dist_x = path.x[1] - current_pose.position.x 
            dist_y = path.y[1] - current_pose.position.y
            update_path = (math.sqrt(dist_x * dist_x + dist_y * dist_y) < min_distance)
        except:
            print("cannot make update sig")            
        # print("dist = {}, min dist = {}".format(math.sqrt(dist_x * dist_x + dist_y * dist_y), min_distance))
        # if path:
        #     if update_path:
        #         try:
        #             self.s0 = path.s[1]
        #             self.c_d = path.d[1]
        #             self.c_d_d = path.d_d[1]
        #             self.c_d_dd = path.d_dd[1]
        #             self.c_speed = path.s_d[1]
        #         except:
        #             print('cannot update s0, c_d, c_d_d ...')
        # else:
        #     if update_path:
        #         self.s0 = path.s[1]
        #         self.c_d = path.d[1]
        #         self.c_d_d = path.d_d[1]
        #         self.c_d_dd = path.d_dd[1]
        #         self.c_speed = path.s_d[1]
        if update_path:
            try:
                self.s0 = path.s[1]
                self.c_d = path.d[1]
                self.c_d_d = path.d_d[1]
                self.c_d_dd = path.d_dd[1]
                self.c_speed = path.s_d[1]
            except:
                print('cannot update s0, c_d, c_d_d ...')


        return control, False