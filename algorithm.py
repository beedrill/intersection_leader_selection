# message type: traffic control message (1), selection initialization message (2), selection response message (3)
# traffic control message: 1,<time of message>,<id of leader>
# selection initialization message: 2,<time of message>,<id of proposor>
# selection response message: 3,<id of proposer>,<id of acceptor>,<direction of acceptor>,<position of acceptor>

import traci
import re
class AlgorithmManager():
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.id = vehicle.id
        self.simulator = vehicle.simulator
        self.log = self.simulator.log
        self.connection_manager = vehicle.connection_manager
        self.leader = "-1"
        self.leader_time = 0                   # The time length after a vehicle becomes the leader
        self.is_prev_leader = False
        self.is_group_leader = False
        self.silent_time = 0                   # The time length of no message received
        self.is_proposer = False               # Whether the vehicle is currently a proposer of a leader selection
        self.selection_time = 0                # The time lengtt after a leader selection is initialized
    
    def step(self):
        self.refresh_is_group_leader()

    def refresh_is_group_leader(self):
        self.is_group_leader = True
        for vid in self.simulator.vehicle_list:
            v = self.simulator.vehicle_list[vid]
            if self.id != vid and self.vehicle.original_lane == v.original_lane and self.vehicle.lane_position > v.lane_position:
                self.is_group_leader = False
                break

    def action(self):
        # If the vehicle is the leader
        if self.leader == self.id:
            self.leader_action()
        # If the vehicle is the group leader
        elif self.is_group_leader == True:
            self.group_leader_action()
        # If the vehicle is neither leader nor group leader, it only parses the most recent traffic control message
        else:
            self.non_leader_action()

    def leader_action(self):
        if self.leader_time == self.simulator.leader_time_threshold:
            successive_leader = self.leader_choose_successive_leader()
            print("successive_leader: " + successive_leader)
            # No matter whether the new leader exists or not, broadcast the result.
            control_message = str(1) + "," + str(self.simulator.time) + "," + successive_leader
            self.connection_manager.broadcast(control_message)
            self.is_prev_leader = True
            # If there is no successive leader, change the traffic light. Give itself the green light.
            if successive_leader == "-1":
                self.simulator.log.write(self.id + " no successive leader\n")
                if self.vehicle.direction == "east-west":
                    traci.trafficlight.setRedYellowGreenState("0", "rrrGGgrrrGGg")
                elif self.vehicle.direction == "north-south":
                    traci.trafficlight.setRedYellowGreenState("0", "GGgrrrGGgrrr")
            else:
                self.simulator.log.write(self.id + " select " + successive_leader + " as the successive leader\n")
        else:
            # Broadcast control message, tell others that itself is the leader
            control_message = str(1) + "," + str(self.simulator.time) + "," + str(self.id)
            self.connection_manager.broadcast(control_message)
            if self.leader_time == 0:
                # Give itself the red light
                if self.vehicle.direction == "east-west":
                    traci.trafficlight.setRedYellowGreenState("0", "GGgrrrGGgrrr")
                elif self.vehicle.direction == "north-south":
                    traci.trafficlight.setRedYellowGreenState("0", "rrrGGgrrrGGg")
            elif self.leader_time == self.simulator.leader_time_threshold - 2:
                # Initialize a new leader selection
                self.log.write(self.id + " initialize selection\n")
                initialization_message = str(2) + "," + str(self.simulator.time) + "," + str(self.id)
                self.connection_manager.broadcast(initialization_message)
        self.leader_time += 1

    def group_leader_action(self):
        self.group_leader_check_control_message()
        self.group_leader_check_initialization_message()
        # If the current leader selection is not aborted
        if self.is_proposer == True:
            self.selection_time += 1
            if self.selection_time == self.simulator.selection_time_threshold:
                new_leader = self.group_leader_choose_new_leader()
                self.leader = new_leader
                if new_leader != "-1":
                    print("new_leader: " + new_leader)
                    self.log.write(self.id + " choose " + new_leader + " as the new leader\n")
                    control_message = str(1) + "," + str(self.simulator.time) + "," + str(new_leader)
                    self.connection_manager.broadcast(control_message)
                self.silent_time = 0
                self.is_proposer = False
        else:
            if len(self.connection_manager.curr_msg_buffer) == 0:
                self.silent_time += 1
                # If the silent time is beyond the threshold, initialize a new leader selection.
                if self.silent_time == self.simulator.silent_time_threshold:
                    self.log.write(self.id + " initialize selection\n")
                    initialization_message = str(2) + "," + str(self.simulator.time) + "," + str(self.id)
                    self.connection_manager.broadcast(initialization_message)
                    self.is_proposer = True
                    self.selection_start_time = self.simulator.time
                    self.selection_time = 0
            else:
                self.silent_time = 0

    def non_leader_action(self):
        self.non_leader_check_control_message()
    
    # The current leader selects the successive leader
    def leader_choose_successive_leader(self):
        response_message_list = self.connection_manager.get_response_message_list()
        for msg in response_message_list:
            message_parsed = re.split(",", msg)
            if message_parsed[3] != self.vehicle.direction:
                return message_parsed[2]
        return "-1"

    # If there is a new control message, broadcast it and abort the leader selection initialized by itself.
    def group_leader_check_control_message(self):
        latest_control_message = self.connection_manager.get_latest_control_message()
        # If there is a new control message, broadcast it.
        if latest_control_message != "":
            message_parsed = re.split(",", latest_control_message)
            if self.leader != message_parsed[2]:
                self.log.write(self.id + " change the leader to " + message_parsed[2] + "\n")
            self.leader = message_parsed[2]
            self.is_proposer = False
            self.connection_manager.broadcast(latest_control_message)
            # If the vehicle becomes the new leader
            if self.leader == self.id:
                self.leader_time = 0

    # If there is an earlier leader selection, abort the leader selection initialized by itself.
    # If there is a concurrent leader selection and the id of the proposer is smaller, abort the leader selection initialized by itself.
    def group_leader_check_initialization_message(self):
        earliest_initialization_message = self.connection_manager.get_earliest_initialization_message()
        if earliest_initialization_message != "":
            message_parsed = re.split(",", earliest_initialization_message)
            time = float(message_parsed[1])
            vid = message_parsed[2]
            if self.is_proposer == True:
                if time < self.selection_start_time or (time == self.selection_start_time and int(vid) < int(self.id)):
                    self.is_proposer = False
            if self.is_proposer == False:
                self.log.write(self.id + " respond to " + vid + "\n")
                response_message = "3," + vid + "," + str(self.id) + "," + self.vehicle.direction + "," + str(self.vehicle.lane_position)
                self.connection_manager.broadcast(response_message)

    def group_leader_choose_new_leader(self):
        dict = {self.vehicle.direction: [self.id, self.vehicle.lane_position]}
        shortest_lane_position = self.vehicle.lane_position
        shortest_direction = self.vehicle.direction
        response_message_list = self.connection_manager.get_response_message_list()
        for msg in response_message_list:
            message_parsed = re.split(",", msg)
            if message_parsed[1] == self.id:
                id = message_parsed[2]
                direction = message_parsed[3]
                lane_position = float(message_parsed[4])
                if lane_position < shortest_lane_position:
                    shortest_lane_position = lane_position
                    shortest_direction = direction
                if direction in dict.keys():
                    if lane_position > dict[direction][1]:
                        dict[direction] = [id, lane_position]
                else:
                    dict[direction] = [id, lane_position]
        if len(dict) <= 1:
            return "-1"
        elif shortest_direction == "east-west":
            return dict["north-south"][0]
        else:
            return dict["east-west"][0]

    def non_leader_check_control_message(self):
        latest_control_message = self.connection_manager.get_latest_control_message()
        if latest_control_message != "":
            message_parsed = re.split(",", latest_control_message)
            if self.leader != message_parsed[2]:
                self.log.write(self.id + " change the leader to " + message_parsed[2] + "\n")
            self.leader = message_parsed[2]