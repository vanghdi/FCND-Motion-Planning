import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import planning_engines

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from queue import Queue

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()
    LOCAL_PLANNING = auto()


class MotionPlanningWithLocalPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.global_waypoints = Queue()
        self.local_path = []
        self.local_planner = None
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.plan_local_path()
        elif self.flight_state == States.WAYPOINT:
            print('target, local', type(self.target_position), type(self.local_position))
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.local_path) > 0:
                    # move towards next local waypoint
                    self.waypoint_transition()
                elif not self.local_planner.end_reached():
                    # create next local plan
                    self.plan_local_path()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.LOCAL_PLANNING:
                if len(self.local_path) == 0:
                    # failed to find a path, try replanning
                    self.plan_path()
                else:
                    print("local path found:", self.local_path)
                    self.waypoint_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = np.array(self.local_path.pop(0))
        print('target position', self.target_position)
        print('%d waypoints remaining in this local path' % len(self.local_path))
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.local_path)
        self.connection._master.write(data)

    def plan_local_path(self):
        self.flight_state = States.LOCAL_PLANNING
        print("Searching for a local path ...")
        # update location will update the local view and determine a local goal
        print("local position: ", self.local_position)
        self.local_planner.update_position(self.local_position, ned=True)
        # search a path and prune it.
        self.local_path, _ = self.local_planner.plan()

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a global path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 3
        MAX_ALTITUDE = 10
        VOXEL_SIZE = 1

        self.target_position[2] = TARGET_ALTITUDE

        filename = 'colliders.csv'
        # TODO: read lat0, lon0 from colliders into floating point values
        global_home = planning_engines.get_global_home(filename)

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(global_home[0], global_home[1], global_home[2])

        # TODO: retrieve current global position
        # TODO: convert to current local position using global_to_local()
        current_local_pos = global_to_local(self.global_position, global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))

        print('local position from drone: {0}, local_position (global_to_local): {1}'.format(self.local_position,
                                                                                           np.array(current_local_pos)))

        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_lon, goal_lat = -122.39441123, 37.79141968 # main str between mission and howard
        #goal_lon, goal_lat = -122.4081522, 37.7942581 # this should be McDonalds
        #goal_lon, goal_lat = -122.398721, 37.7931154  # front street (a block from McDonalds)


        waypoints, data, north_offset, east_offset = planning_engines.get_main_plan(global_home,goal_lat, goal_lon,
                                                        self.local_position, SAFETY_DISTANCE, TARGET_ALTITUDE, filename)

        # store global waypoints and create local planner
        self.global_waypoints = Queue()
        for wp in waypoints:
            self.global_waypoints.put(wp)
        self.local_planner = planning_engines.Local3DPlanner(self.global_waypoints, data,
                                                             north_offset, east_offset, VOXEL_SIZE,
                                                             TARGET_ALTITUDE, MAX_ALTITUDE)

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanningWithLocalPlanning(conn)
    time.sleep(1)

    drone.start()
