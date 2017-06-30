import cv2
import glob

from camera import Camera
from pipeline import LaneDetectionPipeline

# settings
CALIBRATION_MASK = 'camera_cal/calibration*.jpg'
GRID_X = 9
GRID_Y = 6

def process_video(filename, pipeline):
    delay = 0

    video_src = cv2.VideoCapture(filename)
    video_out = cv2.VideoWriter('output.avi',cv2.VideoWriter_fourcc(*'MJPG'), 25.0, (1280,720))
    debug_out = cv2.VideoWriter('debug.avi',cv2.VideoWriter_fourcc(*'MJPG'), 25.0, (1280,720))

    while video_src.isOpened():
        ret, frame = video_src.read()

        if not ret:
            break
        
        result = pipeline.run(frame)
        cv2.imshow('output', result)
        cv2.imshow('debug', pipeline.debug_out)

        video_out.write(result)
        debug_out.write(pipeline.debug_out)
        
        key = cv2.waitKey(delay)
        if key == 27:           # esc
            break
        elif key == ord(' '):    # spacebare
            delay = 0 if delay == 1 else 1
    
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

    process_video('project_video.mp4', pipeline)
    #process_video('challenge_video.mp4', pipeline)


    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
