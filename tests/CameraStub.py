from ObjectDetection.Camera import Camera

class CameraStub(Camera):

    def __init__(self, file_path):
        self.file_path = file_path

    def enable(self):
        pass

    def disable(self):
        pass

    def get_width(self):
        return 4032

    def get_height(self):
        return 3024

    def get_image_array(self):
        return self.file_path