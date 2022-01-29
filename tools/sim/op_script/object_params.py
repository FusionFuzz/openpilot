class VehicleBehavior():
    def __init__(self, lane_change, speed):
        # 0: left, 1: stay, 2: right
        self.lane_change = lane_change
        # percentage to current speed limit
        # [-50, 50]
        self.speed = speed
    def __str__(self):
        return str([self.lane_change, self.speed])
    def __repr__(self):
        return self.__str__()

class Vehicle():
    def __init__(self, model, x, y, yaw, speed, behaviors):
        self.model = model
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = speed
        self.behaviors = behaviors
    def __str__(self):
        return str([self.model, self.x, self.y, self.yaw, self.speed, self.behaviors])
    def __repr__(self):
        return self.__str__()
