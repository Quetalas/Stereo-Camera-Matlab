import cv2
import os
import numpy as np

def show_stereo_image(image, is_success=True):
    if is_success:
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        left_gray = gray_image[:, 0:320]
        right_gray = gray_image[:, 320:640]

        ret_left, corners_left = cv2.findChessboardCorners(left_gray, (9, 6), None)
        ret_right, corners_right = cv2.findChessboardCorners(right_gray, (9, 6), None)

        image_to_show = np.copy(image)
        left_image = cv2.drawChessboardCorners(image_to_show[:, 0:320, :], (9,6), corners_left, ret_left)
        right_image = cv2.drawChessboardCorners(image_to_show[:, 320:640, :], (9, 6), corners_right, ret_right)

        image_to_show = np.concatenate((left_image, right_image), axis=1)
        cv2.imshow('frame', image_to_show)


def find_last_dir(path_where_search):
    last_dir = -1
    for exist_dir in os.listdir(path_where_search):
        if last_dir <= int(exist_dir):
            last_dir = int(exist_dir)
    return str(last_dir)


def is_empty(dir_path):
    return len(os.listdir(dir_path)) == 0

def get_the_path_to_save(save_into_last_dir=False):
    working_dir = os.getcwd()
    samples_dir_name = "images_samples"
    path_to_samples = os.path.join(working_dir, samples_dir_name)

    if not os.path.isdir(path_to_samples):
        os.mkdir(path_to_samples)

    last_dir_number = find_last_dir(path_to_samples)
    if last_dir_number == '-1':
        last_dir_number = "0"
        path_to_save = os.path.join(path_to_samples, last_dir_number)
        os.mkdir(path_to_save)
    else:
        path_to_save = os.path.join(path_to_samples, last_dir_number)

    if save_into_last_dir or is_empty(path_to_save):
       return path_to_save

    new_dir_number = str(int(last_dir_number) + 1)
    path_to_save = os.path.join(path_to_samples, new_dir_number)
    os.mkdir(path_to_save)

    return path_to_save


def save_image(image, path_to_save, image_number):

    left_dir = os.path.join(path_to_save, "left")
    right_dir = os.path.join(path_to_save, "right")

    if not os.path.isdir(left_dir):
        os.mkdir(left_dir)

    if not os.path.isdir(right_dir):
        os.mkdir(right_dir)

    left_image_path = os.path.join(left_dir, "{}.jpg".format(image_number))
    right_image_path = os.path.join(right_dir, "{}.jpg".format(image_number))

    left_image = image[:,0:320,:]
    right_image = image[:,320:640,:]

    if cv2.imwrite(left_image_path, left_image) and \
            cv2.imwrite(right_image_path, right_image):

        print(image_number + 1, ' images are saved')
        return True
    print("image isn't saved")
    return False


if __name__ == '__main__':
    stereo_camera = cv2.VideoCapture(1)
    stereo_camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    stereo_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    path_to_save = get_the_path_to_save(save_into_last_dir=False)
    num_saves = 0
    while True:
        ret, frame = stereo_camera.read()
        show_stereo_image(frame, is_success=ret)
        key = cv2.waitKey(100)
        if key == ord('s'):
            if save_image(frame, path_to_save, num_saves):
                num_saves += 1

        if key == ord('q'):
            break
