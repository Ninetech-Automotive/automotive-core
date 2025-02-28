from Navigation.Edge import Edge

class Angle:
    """
    Represents an outgoing angle from one waypoint to another.
    """
    def __init__(self, outgoing_waypoint, value: float, edge: Edge):
        self.outgoing_waypoint = outgoing_waypoint
        self.value = value
        self.edge = edge

    def get_value(self):
        return self.value
    
    def get_edge(self):
        return self.edge

    def get_waypoint(self):
        return self.outgoing_waypoint
    
    def __str__(self):
        return f"""
        Angle[
            Waypoint:{self.outgoing_waypoint}
            Value:{self.value}
            Edge:{self.edge}
        ]
        """