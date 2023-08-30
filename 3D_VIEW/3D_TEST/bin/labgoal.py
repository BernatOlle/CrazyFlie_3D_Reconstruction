import cv2
import threading
import numpy as np

def display_image(image):
    cv2.imshow("Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def image_processing_thread():
    # Simulate some image processing
    processed_image = np.random.randint(255, size=(420, 666, 3),dtype=np.uint8)
    
    # Pass the processed image to the display function in the main thread
    display_image(processed_image)

def main():
    original_image = np.random.randint(255, size=(420, 666, 3),dtype=np.uint8)

    # Create a thread for image processing
    processing_thread = threading.Thread(target=image_processing_thread)
    processing_thread.start()

    # Display the original image in the main thread
    display_image(original_image)

if __name__ == "__main__":
    main()

