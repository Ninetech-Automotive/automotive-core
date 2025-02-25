from Navigation.EdgeStatus import EdgeStatus

class Edge:
    def __init__(self):
        self.status = EdgeStatus.UNKNOWN
        self.length = 1

    def get_weight(self) -> int:
        return self.status.value + (self.length * 10)
    
    def get_status(self):
        return self.status
    
    def set_status(self, status):
        self.status = status

    def __str__(self):
        return f"Edge[status:{self.status};Length:{self.length}]"