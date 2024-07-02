import vineyard
import numpy as np
import cv2
import matplotlib.pyplot as plt

def get_image_from_vineyard(object_id, vineyard_ipc_socket, height, width, channels):
    # Initialize Vineyard client
    client = vineyard.connect(vineyard_ipc_socket)

    # Get the blob from Vineyard
    blob = client.get("o81910377956355462")
    print(blob.shape)
    print(blob.nbytes)
    buffer = memoryview(blob)

    # Convert buffer to numpy array
    np_arr = np.frombuffer(buffer, dtype=np.uint8)

    print(np_arr.shape)

    # Reshape the numpy array to the original image shape
    image = np_arr.reshape((height, width, channels))

    return image

def show_image(image):
    # Convert BGR image to RGB
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Display the image using Matplotlib
    plt.imshow(image_rgb)
    plt.axis('off')  # Hide axis
    plt.show()

if __name__ == "__main__":
    vineyard_ipc_socket = "/var/run/vineyard.sock"  # Replace with your Vineyard IPC socket path
    object_id = "81910377956355462"  # Replace with the actual ObjectID

    height = 1390  # Replace with actual image height
    width = 866   # Replace with actual image width
    channels = 3  # For a color image

    image = get_image_from_vineyard(object_id, vineyard_ipc_socket, height, width, channels)
    show_image(image)
