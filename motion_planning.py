import argparse
import time
import msgpack
import matplotlib.pyplot as plt
from enum import Enum, auto

import numpy as np
import csv

from planning_utils import a_star, closest_point, create_graph, create_grid_and_edges, heuristic, create_grid, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


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
        # data = np.loadtxt('colliders.csv', delimiter=' ', dtype='Float64', usecols = (2,4), max_rows=1)
        with open('colliders.csv', 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                lat0 = row[0].split(" ")[1]
                long0 = row[1].split(" ")[2]
                break
        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(float(long0), float(lat0), 0)

        # TODO: retrieve current global position
        current_global_pos = self.global_position
 
        # TODO: convert to current local position using global_to_local()
        current_local_pos = global_to_local(current_global_pos, self.global_home)
        print("Current Local pos : ", current_local_pos)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        is_graph = True
        if (is_graph):

            # Define a grid for a particular altitude and safety margin around obstacles
            # grid, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
 
            grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
            print("")

            # Define starting point on the grid (this is just grid center)
            # grid_start = (-north_offset, -east_offset)
            # TODO: convert start position to current position rather than map center
            grid_start = (int(current_local_pos[0]) - north_offset, int(current_local_pos[1]) - east_offset)
            print("Grid start : ", grid_start)
            # Set goal as some arbitrary position on the grid
            # grid_goal = (-north_offset + 10, -east_offset + 10)
            # TODO: adapt to set goal as latitude / longitude position and convert
            # goal_global = [-122 + np.random.random(), 37 + np.random.random(), 0]
            goal_global = [-122.4002093, 37.79577523, 0]
            goal_local = global_to_local(goal_global, self.global_home)
            grid_goal = (int(goal_local[0]) - north_offset, int(goal_local[1]) - east_offset)
            print("Grid goal : ", grid_goal)
            
            # grid_goal = (680, 200)
            # print(local_to_global([680+north_offset, 200+east_offset, 0], self.global_home))

            G = create_graph(edges)
            start_graph = closest_point(G, grid_start)
            goal_graph = closest_point(G, grid_goal)

            path, _ = a_star(G, heuristic, start_graph, goal_graph, is_graph)
            path = prune_path(path, grid)
            path = [(round(cells[0]), round(cells[1])) for cells in path]

            plt.imshow(grid, origin='lower', cmap='Greys') 

            for e in edges:
                p1 = e[0]
                p2 = e[1]
                plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')
                
            plt.plot([grid_start[1], start_graph[1]], [grid_start[0], start_graph[0]], 'r-')
            for i in range(len(path)-1):
                p1 = path[i]
                p2 = path[i+1]
                plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'r-')
            plt.plot([grid_goal[1], goal_graph[1]], [grid_goal[0], goal_graph[0]], 'r-')
                
            plt.plot(grid_start[1], grid_start[0], 'gx', markersize=12)
            plt.plot(grid_goal[1], grid_goal[0], 'gx', markersize=12)

            plt.xlabel('EAST', fontsize=20)
            plt.ylabel('NORTH', fontsize=20)
            plt.show()

        else:
            # Define a grid for a particular altitude and safety margin around obstacles
            grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
            print("")

            # Define starting point on the grid (this is just grid center)
            # grid_start = (-north_offset, -east_offset)
            # TODO: convert start position to current position rather than map center
            grid_start = (int(current_local_pos[0]) - north_offset, int(current_local_pos[1]) - east_offset)

            # Set goal as some arbitrary position on the grid
            # grid_goal = (-north_offset + 10, -east_offset + 10)
            # TODO: adapt to set goal as latitude / longitude position and convert
            goal_global = [-122.4002093, 37.79577523, 0]
            goal_local = global_to_local(goal_global, self.global_home)
            grid_goal = (int(goal_local[0]) - north_offset, int(goal_local[1]) - east_offset)
            
            # Run A* to find a path from start to goal
            # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
            # or move to a different search space such as a graph (not done here)
            print('Local Start and Goal: ', grid_start, grid_goal)
            path, _ = a_star(grid, heuristic, grid_start, grid_goal, is_graph)
            
            # TODO: prune path to minimize number of waypoints
            # TODO (if you're feeling ambitious): Try a different approach altogether!
            path = prune_path(path, grid)

            # plt.imshow(grid, cmap='Greys', origin='lower')

            # plt.plot(grid_start[1], grid_start[0], 'rx', markersize=15)
            # plt.plot(grid_goal[1], grid_goal[0], 'rx', markersize=15)

            # pp = np.array(path)
            # plt.plot(pp[:, 1], pp[:, 0], 'g')
            # plt.scatter(pp[:, 1], pp[:, 0])

            # plt.xlabel('EAST')
            # plt.ylabel('NORTH')

            # plt.show()

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
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
