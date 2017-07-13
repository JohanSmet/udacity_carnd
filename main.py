import cv2
import numpy as np
import time

from car_classifier import CarClassifier
from car_detector import CarDetector
from camera import Camera

DEBUG_VISUALIZE = True
DEBUG_WINDOW = 'output'

def process_video(filename, detector, camera):
    delay = 0
    frame_idx = 0

    video_src = cv2.VideoCapture(filename)
    video_out = cv2.VideoWriter('output_images/output.avi', cv2.VideoWriter_fourcc(*'DIB '), 25.0, (1280,720))

    while video_src.isOpened():
        ret_ok, frame = video_src.read()
        frame_idx = frame_idx + 1

        if not ret_ok:
            break

        t1 = time.time()

        # apply camera distortion correction
        corrected = camera.undistort(frame)

        # run car detection pipeline
        detector.run(corrected)

        # draw rectangles around cars
        result = corrected
        result = detector.draw_detected_rects(result)
        result = detector.draw_car_rects(result)

        # print duration of frame
        t_delta = int((time.time() - t1) * 1000)
        cv2.putText(result, "{} ms - {:.3f} fps".format(t_delta, 1000/t_delta), (20, 30), cv2.FONT_HERSHEY_PLAIN, 2.0, (0,255,255), 2)
        print('Frame {} : {} ms'.format(frame_idx, t_delta))

        # debug output
        if DEBUG_VISUALIZE:
            cv2.imshow(DEBUG_WINDOW, result)
            #cv2.imshow(DEBUG_WINDOW, detector.heatmap.astype(np.uint8))

        # video output 
        video_out.write(result)

        # input when debug-mode is activated
        if DEBUG_VISUALIZE:
            key = cv2.waitKey(delay)
            if key == 27:           # esc
                break
            elif key == ord(' '):   # spacebar
                delay = 0 if delay == 1 else 1

    video_out.release()

def main():
    # initialize camera (for distortion correction)
    camera = Camera('camera_udacity')
    
    if not camera.load():
        print("ERROR LOADING CAMERA CALIBRATION")
    
    # initialize car classifier and detector
    clf = CarClassifier.restore('classifier_svc.pkl')
    detector = CarDetector(clf, heat_threshold=10, num_heat_frames=10)

    # debug visualization
    if DEBUG_VISUALIZE:
        cv2.namedWindow(DEBUG_WINDOW)

    # process video
    #process_video("test_video.mp4", detector, camera)
    process_video("project_video.mp4", detector, camera)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()