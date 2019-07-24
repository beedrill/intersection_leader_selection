class ConnectionManager():
    def __init__(self, veh):
        self.id = veh.id
        self.vehicle = veh
        self.msg_buffer = []
        self.next_msg_buffer = []

    def bind_simulator(self, sim):
        print("here")
        self.simulator = sim
        #self.vehicle_list = self.simulator.vehicle_list

    def broadcast(self, msg):
        for veh in self.simulator.vehicle_list:
            if self._connected(self.vehicle.id, veh.id):
                self.simulator.vehicle_list[veh].connection_manager.next_msg_buffer.append(msg)

    def send(self, msg, to_veh_id):
        if self._connected(self.vehicle.id, to_veh_id):
            self.simulator.vehicle_list[to_veh_id].connection_manager.next_msg_buffer.append(msg)

    @staticmethod
    def _connected(veh_id1, veh_id2):
        #determine if vehi1 is connected with vehi2
        # print("inherit and implement more realistic connected method")
        return True

    def step(self):
        # print("connection manager step")
        self.connected_list = []
        for vid in self.simulator.vehicle_list:
            if self.id != vid and _connected(self.id, vid) == True:
                self.connected_list.append(vid)
        self.msg_buffer = self.next_msg_buffer
        self.next_msg_buffer = []
