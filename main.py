from simulator.simulator import Simulator 
from simulator.route_manager import RouteManager
from algorithm import AlgorithmManager as algo
from connection import ConnectionManager as conn
rm = RouteManager("simulator/maps/simple")
#algo = AlgorithmManager
sim = Simulator(
    rm,  #route manager
    algo, #algorithm module
    conn, #connection module
    "simulator/maps/simple" #map folder
    )


rm.bind_simulator(sim)
sim.start_simulation()