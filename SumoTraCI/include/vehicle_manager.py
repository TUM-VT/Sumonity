import traci
from typing import Dict, Any
import random

class VehicleManager:
    def __init__(self):
        self.vehicle_dict = {}

    def spawn_vehicle(self, vehicle_type: str, vehicle_id: str, position: tuple) -> None:
        """Spawn a vehicle in SUMO simulation."""
        try:
            traci.vehicle.add(vehicle_id, route_id="route_0", typeID=vehicle_type)
            traci.vehicle.moveToXY(vehicle_id, "", 0, position[0], position[1])
            self.vehicle_dict[vehicle_id] = {"type": vehicle_type, "position": position}
        except traci.exceptions.TraCIException as e:
            print(f"Error spawning vehicle: {e}")

    def remove_vehicle(self, vehicle_id: str) -> None:
        """Remove a vehicle from SUMO simulation."""
        if vehicle_id in self.vehicle_dict:
            try:
                traci.vehicle.remove(vehicle_id)
                del self.vehicle_dict[vehicle_id]
            except traci.exceptions.TraCIException as e:
                print(f"Error removing vehicle: {e}") 