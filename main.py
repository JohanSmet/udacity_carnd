import cv2
import glob

from camera import Camera
from pipeline import LaneDetectionPipeline

# settings
CALIBRATION_MASK = 'camera_cal/calibration*.jpg'
GRID_X = 9
GRID_Y = 6

def process_video(filename, pipeline, skip_until_frame=0):
    delay = 0
    frame_idx = 0  

    video_src = cv2.VideoCapture(filename)
    video_out = cv2.VideoWriter('output_images/output.avi',cv2.VideoWriter_fourcc(*'MJPG'), 25.0, (1280,720))
    debug_out = cv2.VideoWriter('output_images/debug.avi',cv2.VideoWriter_fourcc(*'MJPG'), 25.0, (1280,720))

    while video_src.isOpened():
        ret, frame = video_src.read()
        frame_idx = frame_idx + 1

        if not ret:
            break

        if skip_until_frame > 0 and frame_idx < skip_until_frame:
            continue

        print("Processing frame {}".format(frame_idx))

        result = pipeline.run(frame)
        cv2.imshow('output', result)
        cv2.imshow('debug', pipeline.debug_out)

        video_out.write(result)
        debug_out.write(pipeline.debug_out)

        
        key = cv2.waitKey(delay)
        if key == 27:           # esc
            break
        elif key == ord(' '):   # spacebar
            delay = 0 if delay == 1 else 1
        elif key == ord('s'):   
            cv2.imwrite('output_images/captured.jpg', frame)
    
    video_out.release()
    debug_out.release()

def main():
    # initialize camera
    camera = Camera('camera_udacity')

    if not camera.calibrate(CALIBRATION_MASK, GRID_X, GRID_Y):
        print("ERROR CALIBRATING CAMERA")
        return

    # initialize lane detection pipeline
    pipeline = LaneDetectionPipeline(camera)

    cv2.namedWindow('output')
    cv2.namedWindow('debug')
    cv2.namedWindow('debug2')
    #cv2.createTrackbar('min_g', 'debug', 0, 255, lambda x: pipeline.set_threshold_min('g', x))
    #cv2.setTrackbarPos('min_g', 'debug', pipeline.get_threshold_min('g'))
    #cv2.createTrackbar('min_r', 'debug', 0, 255, lambda x: pipeline.set_threshold_min('r', x))
    #cv2.setTrackbarPos('min_r', 'debug', pipeline.get_threshold_min('r'))
    cv2.createTrackbar('min_vn', 'debug', 0, 255, lambda x: pipeline.set_threshold_min('v_n', x))
    cv2.setTrackbarPos('min_vn', 'debug', pipeline.get_threshold_min('v_n'))
    cv2.createTrackbar('min_vl', 'debug', 0, 255, lambda x: pipeline.set_threshold_min('v_l', x))
    cv2.setTrackbarPos('min_vl', 'debug', pipeline.get_threshold_min('v_l'))
    cv2.createTrackbar('min_ln', 'debug', 0, 255, lambda x: pipeline.set_threshold_min('l_n', x))
    cv2.setTrackbarPos('min_ln', 'debug', pipeline.get_threshold_min('l_n'))
    cv2.createTrackbar('min_ll', 'debug', 0, 255, lambda x: pipeline.set_threshold_min('l_l', x))
    cv2.setTrackbarPos('min_ll', 'debug', pipeline.get_threshold_min('l_l'))

    process_video('project_video.mp4', pipeline)
    #process_video('challenge_video.mp4', pipeline)
    #process_video('harder_challenge_video.mp4', pipeline)


    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
