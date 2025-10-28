
class DroneState:
    def __init__(self, drone_id):
        self.id = drone_id
        self.mode = None
        self.lat = None
        self.lon = None
        self.alt = None
        self.battery = None
        self.is_armed = False

    def update_from_dict(self, state_dict):
        self.mode = state_dict.get("Mode")
        self.lat = state_dict.get("GlobalLat")
        self.lon = state_dict.get("GlobalLon")
        self.alt = state_dict.get("RelativeAlt")
        self.battery = state_dict.get("BatteryLevel")
        self.is_armed = state_dict.get("armed")
