import cv2
import numpy as np

SCALING_FACTOR = 2.75

def round_even(n):
    return round(n / 2) * 2

def scale_value(value, min_original, max_original, min_new, max_new):
    return ((value - min_original) / (max_original - min_original)) * (max_new - min_new) + min_new

def crop_black_bounded_shape(input_pgm, output_pgm):
    # Load the PGM file as a grayscale image
    image = cv2.imread(input_pgm, cv2.IMREAD_GRAYSCALE)

    # Threshold the image to isolate the black-bounded shape
    _, thresholded = cv2.threshold(image, 50, 255, cv2.THRESH_BINARY_INV)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create a black image with the same dimensions
    black_image = np.zeros_like(image)

    # Find the largest contour (assumed to be the black-bounded shape)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)

        # Get bounding box coordinates for the largest contour
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Crop the original image using these coordinates
        cropped_image = image[y:y+h, x:x+w]

        # Fill the contour area on the black image
        cv2.drawContours(black_image, [largest_contour], -1, 255, thickness=cv2.FILLED)

        # Mask the cropped image to keep only the shape inside the contour
        masked_cropped_image = cv2.bitwise_and(cropped_image, cropped_image, mask=black_image[y:y+h, x:x+w])

        # Save the cropped image
        cv2.imwrite(output_pgm, masked_cropped_image)
    else:
        print("No black-bounded shape found.")

def find_centroids(pgm_file):
    # Load the PGM file
    image = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError("Could not read the PGM file.")

    # Identify grey points (values between dark and light thresholds)
    grey_mask = cv2.inRange(image, 50, 250)

    # # Show binary mask for inspection
    # cv2.imshow("Grey Mask", grey_mask)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Find contours (connected components)
    contours, _ = cv2.findContours(grey_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    centroids = []
    for contour in contours:
        # Calculate the moments to find the centroid
        M = cv2.moments(contour)
        if M["m00"] != 0:  # Avoid division by zero
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroids.append((cx, cy))

    return centroids

def get_pgm_dimensions(filename):
    with open(filename, 'rb') as f:
        # Read the magic number (P5 or P2)
        magic_number = f.readline().strip()
        if magic_number not in [b'P5', b'P2']:
            raise ValueError("Not a valid PGM file.")
        
        # Read comments (if any)
        while True:
            line = f.readline().strip()
            if line.startswith(b'#'):  # Skip comments
                continue
            else:
                # This line should be the width and height
                width, height = map(int, line.split())
                break
        
        return width, height

def get_ini_parmas():
    # Example usage
    input_pgm = "turtleworld.pgm"
    output_pgm = "cropped_output.pgm"
    crop_black_bounded_shape(input_pgm, output_pgm)
    centroids = find_centroids("cropped_output.pgm")
    pgm_x, pgm_y = get_pgm_dimensions("cropped_output.pgm")
    new_x = int(pgm_x / SCALING_FACTOR)
    new_y = int(pgm_y / SCALING_FACTOR)
    center_x = round_even(new_x / 2)
    center_y = round_even(new_y / 2)

    min_x = -center_x
    max_x = center_x
    min_y = -center_y
    max_y = center_y

    print(min_x, max_x, min_y, max_y)

    x = []
    y = []

    for c in centroids:
        x.append(int(scale_value(c[0], 0, pgm_x, min_x, max_x)))
        y.append(int(scale_value(c[1], 0, pgm_y, min_y, max_y)))

    print(x)
    print(y)

w, h = get_pgm_dimensions('./turtleworld_cropped.pgm')
print((w * .05) / 2)
print((h * .05) / 2)
