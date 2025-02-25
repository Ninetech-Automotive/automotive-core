from Navigation.Edge import Edge

class Angle:
    def __init__(self, waypoint, value: float, edge: Edge):
        self.waypoint = waypoint
        self.value = value
        self.edge = edge

    def get_value(self):
        return self.value
    
    def get_edge(self):
        return self.edge

    def get_waypoint(self):
        return self.waypoint
    
    def __str__(self):
        return f"""
        Angle[
            Waypoint:{self.waypoint}
            Value:{self.value}
            Edge:{self.edge}
        ]
        """