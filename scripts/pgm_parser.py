import yaml
import numpy as np
from PIL import Image
import cv2

SCALING_FACTOR = 2.75

class ROSMapReader:
    def __init__(self, pgm_path, yaml_path):
        self.map_data = np.array(Image.open(pgm_path))
        with open(yaml_path, 'r') as file:
            self.map_info = yaml.safe_load(file)
        
        self.resolution = self.map_info['resolution']  # meters per pixel
        self.origin = self.map_info['origin']           # [x, y, yaw] in meters

        # Detect red clusters (grey equivalent) and store their centroids
        self.centroids = self._find_red_pixels()

    # def _find_red_clusters(self):
    #     # Pure red RGB(255, 0, 0) -> In greyscale: 76 (0.299*255)
    #     red_grey_value = 127
    #     mask = (self.map_data == red_grey_value).astype(np.uint8) * 255

    #     # Find contours
    #     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     centroids = []

    #     for contour in contours:
    #         M = cv2.moments(contour)
    #         if M["m00"] != 0:
    #             cX = int(M["m10"] / M["m00"])
    #             cY = int(M["m01"] / M["m00"])
    #             # Convert to meters
    #             x_m = self.origin[0] + cX * self.resolution
    #             y_m = self.origin[1] + (self.map_data.shape[0] - cY) * self.resolution
    #             centroids.append((x_m, y_m))
    #     return centroids

    def _find_red_pixels(self):
        # Pure red RGB(255, 0, 0) -> In greyscale: 76 (0.299*255)
        red_grey_value = 127
        mask = (self.map_data == red_grey_value)
        
        # Extract coordinates of red pixels
        y_coords, x_coords = np.where(mask)

        # Convert to meters
        pixel_coords = [(self.origin[0] + x * self.resolution,
                         self.origin[1] + (self.map_data.shape[0] - y) * self.resolution)
                        for x, y in zip(x_coords, y_coords)]
        return pixel_coords

    def get_cell_value(self, x, y):
        # Translate coordinates to pixel space
        pixel_x = int((x - self.origin[0]) / self.resolution)
        pixel_y = int((y - self.origin[1]) / self.resolution)

        # Map's origin might be bottom-left, but PGM assumes top-left origin
        pixel_y = self.map_data.shape[0] - pixel_y  # Flip Y-axis for PGM orientation

        # Ensure the coordinates are within map boundaries
        if 0 <= pixel_x < self.map_data.shape[1] and 0 <= pixel_y < self.map_data.shape[0]:
            return self.map_data[pixel_y, pixel_x]
        else:
            raise ValueError("Coordinates are out of map bounds.")

    def get_centroids(self):
        x_safe = []
        y_safe = []
        x_coords, y_coords = zip(*self.centroids) if self.centroids else ([], [])
        for x in x_coords:
            x_safe.append(round(x * SCALING_FACTOR, 2))
        for y in y_coords:
            y_safe.append(round(y * SCALING_FACTOR, 2))
        return x_safe, y_safe
    
    def get_min_max_bounds(self):
        if not self.centroids:
            return None
        x_coords, y_coords = zip(*self.centroids)
        return min(x_coords) * SCALING_FACTOR, max(x_coords) * SCALING_FACTOR, min(y_coords) * SCALING_FACTOR, max(y_coords) * SCALING_FACTOR
    
    def pgm_to_ros_coordinates(self, pixel_x, pixel_y):
        x = self.origin[0] + pixel_x * self.resolution
        y = self.origin[1] + (self.map_data.shape[0] - pixel_y) * self.resolution
        return x, y


if __name__ == "__main__":
    mapper = ROSMapReader("./maps/house/turtlehouse_annotated.pgm", "./maps/house/turtlehouse.yaml")
    colors = []
    for x in range(0, mapper.map_data.shape[0]):
        for y in range(0, mapper.map_data.shape[0]):
            if mapper.map_data[x, y] not in colors:
                colors.append(mapper.map_data[x, y])
    print(mapper.get_centroids()[0])
    print(mapper.get_centroids()[1])
    print(mapper.get_min_max_bounds())