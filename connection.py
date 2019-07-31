import re
class ConnectionManager():
    def __init__(self, veh):
        self.id = veh.id
        self.vehicle = veh
        self.simulator = veh.simulator
        self.connected_list = []
        self.curr_msg_buffer = []
        self.next_msg_buffer = []
        self.latest_control_msg_time = -1.0    # The last time when the vehicle receives a traffic control message
        self.latest_control_message = ""
        self.earliest_initialization_message = ""
        self.response_message_list = []
        self.position_message_list = []

    def bind_simulator(self, sim):
        print("connection bind simulator")
        self.simulator = sim
        #self.vehicle_list = self.simulator.vehicle_list

    def step(self):
        self.get_connected_list()
        self.curr_msg_buffer = self.next_msg_buffer
        self.next_msg_buffer = []
        self.classify_message()

    def connected(self, vid):
        v = self.simulator.vehicle_list[vid]
        if self.vehicle.original_lane == v.original_lane:
            return True
        if self.vehicle.lane_position <= 20 and v.lane_position <= 20:
            return True
        return False

    def broadcast(self, msg):
        for vid in self.connected_list:
            self.simulator.vehicle_list[vid].connection_manager.next_msg_buffer.append(msg)

    def get_connected_list(self):
        self.connected_list = []
        for vid in self.simulator.vehicle_list:
            if self.id != vid and self.connected(vid) == True:
                self.connected_list.append(vid)

    def classify_message(self):
        self.latest_control_message = ""
        self.earliest_initialization_message = ""
        earliest_selection_start_time = float(self.simulator.time)
        self.response_message_list = []
        self.position_message_list = []
        for msg in self.curr_msg_buffer:
            message_parsed = re.split(",", msg)
            if message_parsed[0] == "1":
                time = float(message_parsed[1])
                if time > self.latest_control_msg_time:
                    self.latest_control_msg_time = time
                    self.latest_control_message = msg
            elif message_parsed[0] == "2":
                time = float(message_parsed[1])
                if time < earliest_selection_start_time:
                    earliest_selection_start_time = time
                    self.earliest_initialization_message = msg
            elif message_parsed[0] == "3":
                self.response_message_list.append(msg)
            elif message_parsed[0] == "4":
                self.position_message_list.append(msg)

    def get_latest_control_message(self):   
        return self.latest_control_message

    def get_earliest_initialization_message(self):
        return self.earliest_initialization_message

    def get_response_message_list(self):
        return self.response_message_list

    def get_position_message_list(self):
        return self.position_message_list