from Navigation.WaypointStatus import WaypointStatus
from Navigation.EdgeStatus import EdgeStatus
from ObjectDetection.ObjectDetector import ObjectDetector

class ColorDetector(ObjectDetector):

    def __init__(self, camera):
        self.camera = camera
 
    def is_target_color(self, r, g, b, target_color, tolerance):
        target_r, target_g, target_b = target_color
        # Check if the pixel is within the color tolerance
        return (
            abs(r - target_r) <= tolerance
            and abs(g - target_g) <= tolerance
            and abs(b - target_b) <= tolerance
        )

    def detect(self):
        self.camera.enable()

        width = self.camera.get_width()
        height = self.camera.get_height()
        strip_width = width // 2
        strip_height = height
        cone_color = [228, 162, 55]
        obstacle_color = [255, 0, 0]
        color_tolerance = 40

        image = self.camera.get_image_array()

        # Calculate the horizontal strip bounds (centered in the middle)
        strip_start_x = (width - strip_width) // 2
        strip_end_x = strip_start_x + strip_width

        # Calculate the vertical bounds (centered vertically)
        strip_start_y = (height - strip_height) // 2
        strip_end_y = strip_start_y + strip_height

        matching_cone_pixel_count = 0
        matching_obstacle_pixel_count = 0

        for x in range(strip_start_x, strip_end_x, 2):
            for y in range(strip_start_y, strip_end_y):
                red = image[x][y][0]
                green = image[x][y][1]
                blue = image[x][y][2]

                if self.is_target_color(red, green, blue, cone_color, color_tolerance):
                    matching_cone_pixel_count += 1
                if self.is_target_color(
                    red, green, blue, obstacle_color, color_tolerance
                ):
                    matching_obstacle_pixel_count += 1
                    
        waypoint_status = WaypointStatus.POTENTIALLY_FREE
        edge_status = EdgeStatus.POTENTIALLY_FREE
        if matching_cone_pixel_count > 100:
            waypoint_status = WaypointStatus.POTENTIALLY_BLOCKED
        if matching_obstacle_pixel_count > 50:
            edge_status = EdgeStatus.POTENTIALLY_OBSTRUCTED

        self.camera.disable()
        return waypoint_status, edge_status
