import traci
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
        self.sumoCmd = [self.sumo_binary, "-c", self.map_folder+"/traffic.sumocfg"]
        self.route_manager = route_manager
        self.time = 0
        self.vehicle_list = {}

    def init_params(self):
        self.route_manager.init_routes()
        self.deltaT = traci.simulation.getDeltaT()

    def start_simulation(self):
        traci.start(self.sumoCmd)
        self.init_params()
        while traci.simulation.getMinExpectedNumber() > 0: 
            self.step() 
            self.time += self.deltaT

    def step(self):
        traci.simulationStep()
        self.route_manager.step()
        self.maintain_vehicle_list()
        for vid in self.vehicle_list:
            self.vehicle_list[vid].step()
            print(self.vehicle_list[vid].lane_position)

    def maintain_vehicle_list(self):
        depart_id_list = traci.simulation.getDepartedIDList()
        for id in depart_id_list:
            if not id in self.vehicle_list.keys():
                self.vehicle_list[id] = Vehicle(id)
                v = self.vehicle_list[id]
                v.bind_algorithm(self.algorithm_module(v))
                v.bind_connection_manager (self.connection_module(v))
        
        arrived_id_list = traci.simulation.getArrivedIDList()
        for id in arrived_id_list:
            if id in self.vehicle_list.keys():
                self.vehicle_list.pop(id)

class Vehicle():
    def __init__(self, id):
        self.id = id
        
    def bind_algorithm(self, algorithm):
        self.algorithm = algorithm    

    def bind_connection_manager(self, connection_manager):
        self.connection_manager = connection_manager
    def step(self):
        self.connection_manager.step() 
        self.lane_id = traci.vehicle.getLaneID(self.id)
        self.lane_position = self.get_lane_position()
        self.algorithm.step()
        
    def get_lane_position(self): 
        #return distance toward the next intersection
        from_origin = traci.vehicle.getLanePosition(self.id)  
        lane_length = traci.lane.getLength(self.lane_id)
        return lane_length - from_origin

    

