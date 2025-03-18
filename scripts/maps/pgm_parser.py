import yaml
import numpy as np
from PIL import Image
import cv2

class ROSMapReader:
    def __init__(self, pgm_path, yaml_path):
        self.map_data = np.array(Image.open(pgm_path))
        with open(yaml_path, 'r') as file:
            self.map_info = yaml.safe_load(file)
        
        self.resolution = self.map_info['resolution']  # meters per pixel
        self.origin = self.map_info['origin']           # [x, y, yaw] in meters

        # Detect red clusters (grey equivalent) and store their centroids
        self.centroids = self._find_red_clusters()

    def _find_red_clusters(self):
        # Pure red RGB(255, 0, 0) -> In greyscale: 76 (0.299*255)
        red_grey_value = 127
        mask = (self.map_data == red_grey_value).astype(np.uint8) * 255

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []

        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                # Convert to meters
                x_m = self.origin[0] + cX * self.resolution
                y_m = self.origin[1] + (self.map_data.shape[0] - cY) * self.resolution
                centroids.append((x_m, y_m))
        return centroids

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
        x_coords, y_coords = zip(*self.centroids) if self.centroids else ([], [])
        return np.array(x_coords), np.array(y_coords)

# Example usage:
# reader = ROSMapReader("map.pgm", "map.yaml")
# print(reader.get_cell_value(1.0, 2.0))
# print(reader.get_centroids())


if __name__ == "__main__":
    mapper = ROSMapReader("./house/turtlehouse_annotated.pgm", "./house/turtlehouse.yaml")
    colors = []
    for x in range(0, mapper.map_data.shape[0]):
        for y in range(0, mapper.map_data.shape[0]):
            if mapper.map_data[x, y] not in colors:
                colors.append(mapper.map_data[x, y])
    print(colors)
    print(mapper.get_centroids())