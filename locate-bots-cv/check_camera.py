import cv2

def check_cameras():
    valid_cameras = []
    for index in range(10):  # Check up to 10 camera indices
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            valid_cameras.append(index)
            cap.release()
    return valid_cameras

if __name__ == "__main__":
    available_cameras = check_cameras()
    if available_cameras:
        print("Available cameras:", available_cameras)
    else:
        print("No cameras found.")