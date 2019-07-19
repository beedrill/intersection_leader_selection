class RouteManager():
    def __init__(self, map_folder, 
    route_configs={
        "pattern":"fixed" #or "random"
        }):
        self.route_configs = route_configs
    def bind_simulator(self, sim):
        self.simulator = sim
    def init_routes(self):
        print("please inheritate and reimplement this method")
    def step(self):
        print ("route manager stepped")

