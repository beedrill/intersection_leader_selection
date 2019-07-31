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

    def bind_simulator(self, sim):
        print("connection bind simulator")
        self.simulator = sim
        #self.vehicle_list = self.simulator.vehicle_list

    def step(self):
        self.get_connected_list()
        self.curr_msg_buffer = self.next_msg_buffer
        self.next_msg_buffer = []

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

    def get_latest_control_message(self):
        latest_control_message = ""
        for msg in self.curr_msg_buffer:
            message_parsed = re.split(",", msg)
            if message_parsed[0] == "1":
                time = float(message_parsed[1])
                if time > self.latest_control_msg_time:
                    self.latest_control_msg_time = time
                    latest_control_message = msg
        return latest_control_message

    def get_earliest_initialization_message(self):
        earliest_selection_start_time = float(self.simulator.time)
        earliest_initialization_message = ""
        for msg in self.curr_msg_buffer:
            message_parsed = re.split(",", msg)
            if message_parsed[0] == "2":
                time = float(message_parsed[1])
                if time < earliest_selection_start_time:
                    earliest_selection_start_time = time
                    earliest_initialization_message = msg
        return earliest_initialization_message

    def get_response_message_list(self):
        response_message_list = []
        for msg in self.curr_msg_buffer:
            message_parsed = re.split(",", msg)
            if message_parsed[0] == "3":
                response_message_list.append(msg)
        return response_message_list