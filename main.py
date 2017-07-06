import cv2
import glob

from camera import Camera
from pipeline import LaneDetectionPipeline

# settings
CALIBRATION_MASK = 'camera_cal/calibration*.jpg'
GRID_X = 9
GRID_Y = 6

DEBUG_VISUALIZE = False
DEBUG_PIP = True

def process_video(filename, pipeline, skip_until_frame=0):
    delay = 0
    frame_idx = 0  

    video_src = cv2.VideoCapture(filename)
    video_out = cv2.VideoWriter('output_images/output.avi',cv2.VideoWriter_fourcc(*'DIB '), 25.0, (1280,720))
    video_pip = cv2.VideoWriter('output_images/output_debug.avi',cv2.VideoWriter_fourcc(*'DIB '), 25.0, (1280,720))

    while video_src.isOpened():
        ret_ok, frame = video_src.read()
        frame_idx = frame_idx + 1

        if not ret_ok:
            break

        if skip_until_frame > 0 and frame_idx < skip_until_frame:
            continue

        print("Processing frame {}".format(frame_idx))

        # process a frame
        result = pipeline.run(frame)

        # debug output
        if DEBUG_VISUALIZE:
            cv2.imshow('output', result)
            cv2.imshow('debug', pipeline.debug_out)

        # video output 
        video_out.write(result)

        # video output (with debug info pip)
        result[10:190,700:1020] = cv2.resize(pipeline.debug_out, (320, 180))
        video_pip.write(result)

        # input when debug-mode is on
        if DEBUG_VISUALIZE:
            key = cv2.waitKey(delay)
            if key == 27:           # esc
                break
            elif key == ord(' '):   # spacebar
                delay = 0 if delay == 1 else 1
            elif key == ord('s'):   
                cv2.imwrite('output_images/captured.jpg', frame)
            elif key == ord('d'):
                cv2.imwrite('output_images/debug.jpg', pipeline.debug_out)
    
    video_out.release()
    video_pip.release()

def main():
    # initialize camera
    camera = Camera('camera_udacity')

    if not camera.calibrate(CALIBRATION_MASK, GRID_X, GRID_Y):
        print("ERROR CALIBRATING CAMERA")
        return

    # initialize lane detection pipeline
    pipeline = LaneDetectionPipeline(camera)

    # create debug visualisations and tools
    if DEBUG_VISUALIZE:
        cv2.namedWindow('output')
        cv2.namedWindow('debug')
        cv2.createTrackbar('min_bn', 'debug', 0, 255, lambda x: pipeline.set_threshold_min('b_n', x))
        cv2.setTrackbarPos('min_bn', 'debug', pipeline.get_threshold_min('b_n'))
        cv2.createTrackbar('min_bl', 'debug', 0, 255, lambda x: pipeline.set_threshold_min('b_l', x))
        cv2.setTrackbarPos('min_bl', 'debug', pipeline.get_threshold_min('b_l'))
        cv2.createTrackbar('min_ln', 'debug', 0, 255, lambda x: pipeline.set_threshold_min('l_n', x))
        cv2.setTrackbarPos('min_ln', 'debug', pipeline.get_threshold_min('l_n'))
        cv2.createTrackbar('min_ll', 'debug', 0, 255, lambda x: pipeline.set_threshold_min('l_l', x))
        cv2.setTrackbarPos('min_ll', 'debug', pipeline.get_threshold_min('l_l'))

    # process the video
    process_video('project_video.mp4', pipeline)
    #process_video('challenge_video.mp4', pipeline)
    #process_video('harder_challenge_video.mp4', pipeline)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
