import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import asyncio
asyncio_ensure_future = asyncio.ensure_future 

from planning_utils import a_star, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

import matplotlib.pyplot as plt


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

def are_three_points_collinear3(points, epsilon=1e-6):
    # Extract coordinates from the array
    x1, y1 = points[0]
    x2, y2 = points[1]
    x3, y3 = points[2]
    
    # Calculate the area using the determinant method
    determinant = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)

    # Check if the area is zero
    return abs(determinant) < epsilon


def prune_rec(path, start_idx, end_idx):
    span = end_idx - start_idx +1
    if span == 3:
        if are_three_points_collinear3(path[start_idx:end_idx+1]):
            return [path[start_idx], path[end_idx]]
        else:
            return path[start_idx:end_idx+1]

    mid = (start_idx + end_idx) // 2
    l_span = mid - start_idx +1
    r_span = end_idx - mid
    left = prune_rec(path, start_idx, mid) if l_span > 2 else path[start_idx:mid+1]
    right = prune_rec(path, mid+1, end_idx) if r_span > 2 else path[mid+1:end_idx+1]
    if len(left) > 1 and are_three_points_collinear3([left[-2], left[-1], right[0]]):
        left = left[:-1]
    if len(right) >1 and are_three_points_collinear3([left[-1], right[0], right[1]]):
        right = right[1:]
    return left + right

def prune_path(path):
    return prune_rec(path, 0, len(path)-1)

def draw_grid(grid, edges, start_ne, goal_ne, path):
    # plt.imshow(np.flip(grid, 0))
    plt.imshow(grid, origin='lower', cmap='Greys') 

    for e in edges:
        p1 = e[0]
        p2 = e[1]
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

        
    plt.plot(start_ne[1], start_ne[0], 'rx')
    plt.plot(goal_ne[1], goal_ne[0], 'rx')


    # Plotting the path
    if path is not None:
        for i in range(len(path)-1):
            plt.plot([path[i][1], path[i+1][1]], [path[i][0], path[i+1][0]], 'g-')  # Path in green


    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()

class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
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
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
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
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
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
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            first_line = f.readline().strip()
            lat0, lon0 = [float(x[5:]) for x in first_line.split(",")]
        # TODO: set home position to (lon0, lat0, 0)

        print("lat0 = {0}, lon0 = {1}".format(lat0, lon0))
        self.set_home_position(lon0, lat0, 0)
        # TODO: retrieve current global position
 
        # TODO: convert to current local position using global_to_local()
        
        local_position = global_to_local(self.global_position, self.global_home)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
       

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (int(local_position[0] - north_offset), int(local_position[1] - east_offset))
        # TODO: convert start position to current position rather than map center
        
        # Set goal as some arbitrary position on the grid that doesn't contain an obstacle

        # Define a goal location
        lon_lat_goal = (-122.395009, 37.790436, 5)
        goal_north_local, goal_east_local, _ = global_to_local(lon_lat_goal, self.global_home)        
        grid_goal = (int(goal_north_local - north_offset), int(goal_east_local - east_offset))
        print("Goal: ", grid_goal)

        # Define a random goal location
        # goal_north = np.random.randint(0, grid.shape[0])
        # goal_east = np.random.randint(0, grid.shape[1])
        # while grid[goal_north, goal_east] == 1:
        #     goal_north = np.random.randint(0, grid.shape[0])
        #     goal_east = np.random.randint(0, grid.shape[1])

        # grid_goal = (goal_north, goal_east)
            
        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path = a_star(grid, grid_start, grid_goal)
   
   
        print("Path found")
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # draw_grid(grid, [], grid_start, grid_goal, path)
        pruned_path = prune_path(path)
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

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
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
