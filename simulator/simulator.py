import traci
import re
class Simulator():
    def __init__(self,
        route_manager, 
        algorithm_module,
        connection_module,
        map_folder,
        visual = True,
        ):
        self.map_folder = map_folder
        self.visual = visual
        self.algorithm_module = algorithm_module
        self.connection_module = connection_module
        if visual:
            self.sumo_binary = "sumo-gui"
        else:
            self.sumo_binary = "sumo"
        self.sumoCmd = [self.sumo_binary, "--step-length", "0.01", "-c", self.map_folder + "/traffic.sumocfg"]
        self.route_manager = route_manager
        self.time = 0
        self.vehicle_list = {}

    def init_params(self):
        self.route_manager.init_routes()
        traci.start(self.sumoCmd)
        self.deltaT = traci.simulation.getDeltaT()

    def start_simulation(self):
        self.init_params()
        count = 0
        while traci.simulation.getMinExpectedNumber() > 0: 
            self.step(count)
            self.time += self.deltaT
            count += 1

    def step(self, count):
        traci.simulationStep()
        self.route_manager.step()
        self.maintain_vehicle_list()
        for vid in self.vehicle_list:
            self.vehicle_list[vid].get_lane_position()
        for vid in self.vehicle_list:
            self.vehicle_list[vid].step()
        for vid in self.vehicle_list:
            self.vehicle_list[vid].action()
        if count % 100 == 0 and len(self.vehicle_list) > 0:
            print(self.time, end=":\n")
            print(traci.trafficlight.getRedYellowGreenState("0"))
            for vid in self.vehicle_list:
                print("vid: " + str(vid) + " lane: " + str(self.vehicle_list[vid].original_lane) + " direction: " + self.vehicle_list[vid].direction
                    + " is_group_leader: " + str(self.vehicle_list[vid].is_group_leader) + " position: " + str(self.vehicle_list[vid].lane_position)
                    + " connected_list:", end="")
                for cvid in self.vehicle_list[vid].connected_list:
                    print(" " + str(cvid), end="")
                print()

    def maintain_vehicle_list(self):
        depart_id_list = traci.simulation.getDepartedIDList()
        for id in depart_id_list:
            if not id in self.vehicle_list.keys():
                self.vehicle_list[id] = Vehicle(id)
                v = self.vehicle_list[id]
                v.bind_simulator(self)
                # v.bind_algorithm(self.algorithm_module(v))
                # v.bind_connection_manager(self.connection_module(v))
        
        arrived_id_list = traci.simulation.getArrivedIDList()
        for id in arrived_id_list:
            if id in self.vehicle_list.keys():
                self.vehicle_list.pop(id)

        for id in list(self.vehicle_list):
            v = self.vehicle_list[id]
            if traci.vehicle.getLaneID(id) != v.original_lane:
                self.vehicle_list.pop(id)

class Vehicle():
    def __init__(self, id):
        self.id = id
        self.original_lane = traci.vehicle.getLaneID(self.id)
        self.direction = self.get_direction()
        self.leader = -1
        self.isProposer = False
        self.curr_msg_buffer = []
        self.next_msg_buffer = []
        self.connected_list = []
        self.latest_control_msg_time = -1.0    # The last time when the vehicle receives a traffic control message
        self.remain_leader_time = 0    # The remaining time for the leader
        self.silent_time = 0    # The time length of no message received
        
    def bind_algorithm(self, algorithm):
        self.algorithm = algorithm    

    def bind_connection_manager(self, connection_manager):
        self.connection_manager = connection_manager

    def bind_simulator(self, simulator):
        self.simulator = simulator

    def get_direction(self):
        list = re.split("_", self.original_lane)
        if list[0] == "east" or list[0] == "west":
            return "east-west"
        elif list[0] == "north" or list[0] == "south":
            return "north-south"
        
    def get_lane_position(self): 
        #return distance toward the next intersection
        from_origin = traci.vehicle.getLanePosition(self.id)  
        lane_length = traci.lane.getLength(self.original_lane)
        self.lane_position = lane_length - from_origin

    def connected(self, vid):
        v = self.simulator.vehicle_list[vid]
        if self.original_lane == v.original_lane:
            return True
        if self.lane_position <= 20 and v.lane_position <= 20:
            return True
        return False

    def step(self):
        # self.algorithm.step()

        # Get connected list
        self.connected_list = []
        self.is_group_leader = True
        for vid in self.simulator.vehicle_list:
            if self.id != vid and self.connected(vid) == True:
                self.connected_list.append(vid)
                v = self.simulator.vehicle_list[vid]
                if v.original_lane == self.original_lane and v.lane_position < self.lane_position:
                    self.is_group_leader = False

        # Refresh message buffer
        self.curr_msg_buffer = self.next_msg_buffer
        self.next_msg_buffer = []

    def broadcast(self, msg):
        for vid in self.connected_list:
            self.simulator.vehicle_list[vid].next_msg_buffer.append(msg)

    # message type: traffic control message (1), selection initialization message (2), selection response message (3)
    # traffic control message: 1,<time of message>,<id of leader>
    # selection initialization message: 2,<time of message>,<id of proposor>
    # selection response message: 3,<id of proposer>,<id of acceptor>,<direction of acceptor>,<position of acceptor>
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

    # The current leader selects the successive leader
    def choose_successive_leader(self):
        for msg in self.curr_msg_buffer:
            message_parsed = re.split(",", msg)
            if message_parsed[0] == "3" and message_parsed[3] != self.direction:
                return int(message_parsed[2])
        return -1

    def leader_action(self):
        self.remain_leader_time -= 1
        # Initialize a new leader selection
        if self.remain_leader_time == 1:
            initialization_message = str(2) + "," + str(self.simulator.time) + "," + str(self.id)
            self.broadcast(initialization_message)
            control_message = str(1) + "," + str(self.simulator.time) + "," + str(self.id)
            self.broadcast(control_message)
        # Select a new leader or abort the selection and change the traffic light
        elif self.remain_leader_time == 0:
            successive_leader = self.choose_successive_leader()
            # No matter whether the new leader exists or not, broadcast the result.
            control_message = str(1) + "," + str(self.simulator.time) + "," + str(successive_leader)
            self.broadcast(control_message)
            # Change the traffic light
            if self.direction == "east-west":
                traci.trafficlight.setRedYellowGreenState("0", "rrrGGgrrrGGg")
            elif self.direction == "north-south":
                traci.trafficlight.setRedYellowGreenState("0", "GGgrrrGGgrrr")
        # Broadcast the traffic control message
        else:
            control_message = str(1) + "," + str(self.simulator.time) + "," + str(self.id)
            self.broadcast(control_message)

    def group_leader_action(self):
        if len(self.curr_msg_buffer) == 0:
            self.silent_time += 1
            if self.silent_time == 20:
                initialization_message = str(2) + "," + str(self.simulator.time) + "," + str(self.id)
                self.broadcast(initialization_message)
        else:
            self.silent_time = 0
            self.broadcast_latest_control_message()
            # If there is a control message
            if self.leader != -1:
                self.isProposer = False

    def non_leader_action(self):
        self.broadcast_latest_control_message()

    def broadcast_latest_control_message(self):
        latest_control_message = ""
        for msg in self.curr_msg_buffer:
            message_parsed = re.split(",", msg)
            if message_parsed[0] == "1":
                time = float(message_parsed[1])
                if time > self.latest_control_msg_time:
                    self.leader = int(message_parsed[2])
                    self.latest_control_msg_time = time
                    latest_control_message = msg
        # If there is a new control message, broadcast it.
        if latest_control_message != "":
            self.broadcast(latest_control_message)
        # If the vehicle becomes the new leader
        if self.leader == self.id:
            self.remain_leader_time = 200