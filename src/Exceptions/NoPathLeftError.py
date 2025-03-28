class NoPathLeftError(Exception):
    """
    Exception raised when there are no paths left to explore or follow.
    """
    def __init__(self, message="No paths left to proceed."):
        super().__init__(message)