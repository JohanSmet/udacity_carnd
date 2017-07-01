
import numpy as np
import cv2

from collections import deque

class LaneDetectionPipeline:
    def __init__(self, camera, smooth_factor=10):
        self.compute_perspective_transform()
        self.camera = camera

        self.debug = True
        self.debug_out = None

        self.lanes = [deque(maxlen=smooth_factor), deque(maxlen=smooth_factor)]
        self.cur_lane = [None, None]

    def compute_perspective_transform(self):
        src_points = np.float32([[ 230, 700], [ 580, 460], [ 700, 460], [1090, 700]])
        dst_points = np.float32([[ 300, 720], [ 310, 130], [1030, 130], [1040, 720]])

        self.persp_transform = cv2.getPerspectiveTransform(src_points, dst_points)
        self.persp_transform_inv = cv2.getPerspectiveTransform(dst_points, src_points)

    def undistort(self, img):
        return self.camera.undistort(img)

    def transform_topdown(self, img):
        return cv2.warpPerspective(img, self.persp_transform,
                                   (img.shape[1], img.shape[0]),
                                   flags=cv2.INTER_LINEAR)

    def gradient_treshold(self, img, t_min=20, t_max=100):
        # absolute value of derivatie in x (accentuates more vertical lines)
        sobelx = np.absolute(cv2.Sobel(img, cv2.CV_64F, 1, 0))
        scaled_sobelx = np.uint8(255*sobelx/np.max(sobelx))

        # threshold x gradient
        mask = np.zeros_like(scaled_sobelx)
        mask[(scaled_sobelx >= t_min) & (scaled_sobelx <= t_max)] = 1

        return mask

    def color_treshold(self, img, c_min=175, c_max=255):
        mask = np.zeros_like(img)
        mask[(img >= c_min) & (img < c_max)] = 1
        return mask
    
    def treshold(self, img):
        # convert image to HLS colorspace
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        
        # take the luminance channel and apply a gradient treshold
        g_mask = self.gradient_treshold(hls[:,:,1])
                
        # take the saturation channel and apply a color treshold
        c_mask = self.color_treshold(hls[:,:,2])
        
        # combine the masks
        mask = np.zeros_like(g_mask)
        mask[(g_mask == 1) | (c_mask == 1)] = 1
        
        return mask
    
    def detect_lane_lines(self, img_mask):
        
        # fill some shorthand variables
        img_w, img_h = (img_mask.shape[1], img_mask.shape[0])
        
        window_n = 9
        window_w = 200
        window_d = window_w // 2
        window_h = img_h // window_n
        recenter_min = 50
        
        # save the indices of the active pixels in the mask
        active_y, active_x = np.nonzero(img_mask)
                
        # create a RGB output image to draw on and  visualize the result
        if self.debug:
            self.debug_out = np.dstack((img_mask, img_mask, img_mask)) * 255
               
        # compute the histogram of the lower half of the image (sum each column)
        hist = np.sum(img_mask[img_h//2:,:], axis=0)
        
        # get the position of the peak in the left and right half of the histogram
        center_x = np.int(img_w / 2)
        
        c_left   = np.argmax(hist[:center_x])
        c_right  = np.argmax(hist[center_x:]) + center_x
        y_max    = img_h
        y_min    = y_max - window_h
        
        # indices into the active_x/y arrays of points that fall with the sliding windows
        l_lane_indices = []
        r_lane_indices = []
        
        for window in range(window_n):
            # compute horizontal edges of the window
            l_x_min = c_left - window_d
            l_x_max = c_left + window_d
            r_x_min = c_right - window_d
            r_x_max = c_right + window_d
                     
            # debug-visualisation
            if self.debug:
                cv2.rectangle(self.debug_out, (l_x_min, y_min), (l_x_max, y_max), (0, 255, 0), 2)
                cv2.rectangle(self.debug_out, (r_x_min, y_min), (r_x_max, y_max), (0, 255, 0), 2)
            
            # save coordinates of the detected pixels in the windows            
            l_inds = ((active_y >= y_min) & (active_y < y_max) & (active_x >= l_x_min) & (active_x < l_x_max)).nonzero()[0]
            r_inds = ((active_y >= y_min) & (active_y < y_max) & (active_x >= r_x_min) & (active_x < r_x_max)).nonzero()[0]
                    
            l_lane_indices.append(l_inds)
            r_lane_indices.append(r_inds)
        
            # recenter the windows ?
            if len(l_inds) > recenter_min:
                c_left = np.int(np.mean(active_x[l_inds]))
            if len(r_inds) > recenter_min:
                c_right = np.int(np.mean(active_x[r_inds]))
            
            # move to the next vertical slice
            y_max = y_min
            y_min = y_max - window_h
            
        # lane indices are arrays of arrays: concatenate into 1 array
        l_lane_indices = np.concatenate(l_lane_indices)
        r_lane_indices = np.concatenate(r_lane_indices)
        
        # color the detected pixels
        if self.debug:
            self.debug_out[active_y[l_lane_indices], active_x[l_lane_indices]] = [255, 0, 0]
            self.debug_out[active_y[r_lane_indices], active_x[r_lane_indices]] = [0, 0, 255]
        
        # fit second order polynomal functions to the points
        l_lane = np.polyfit(active_y[l_lane_indices], active_x[l_lane_indices], 2)
        r_lane = np.polyfit(active_y[r_lane_indices], active_x[r_lane_indices], 2)

        self.lanes[0].append(l_lane)
        self.lanes[1].append(r_lane)

    def average_lanes(self):
        self.cur_lane[0] = np.mean(np.array(list(self.lanes[0])), axis=0)
        self.cur_lane[1] = np.mean(np.array(list(self.lanes[1])), axis=0)

    def lane_image(self, img_w, img_h):
        # convert the polynomal functions to a list of points
        plot_y  = np.linspace(0, img_h-1, img_h)
        plot_lx = self.cur_lane[0][0]*plot_y**2 + self.cur_lane[0][1]*plot_y + self.cur_lane[0][2]
        plot_rx = self.cur_lane[1][0]*plot_y**2 + self.cur_lane[1][1]*plot_y + self.cur_lane[1][2]

        # combine the separate x/y arrays in to arrays with (x,y)-pairs
        l_lane = np.transpose(np.vstack([plot_lx, plot_y]))
        r_lane = np.transpose(np.vstack([plot_rx, plot_y]))

        # output to the debug surface
        if self.debug:
            cv2.polylines(self.debug_out, [np.int32(l_lane)], False, [0,255,255], 4)
            cv2.polylines(self.debug_out, [np.int32(r_lane)], False, [0,255,255], 4)

        # create a new image to draw the lane on
        layer_zero  = np.zeros((img_h, img_w), dtype=np.uint8)
        lane_output = np.dstack((layer_zero, layer_zero, layer_zero))
        
        points = np.concatenate((np.int32(l_lane), np.int32(np.flipud(r_lane))))
        cv2.fillPoly(lane_output, [points], (0,255,0))
        
        # reproject to the original space
        lane_unwarp = cv2.warpPerspective(lane_output, self.persp_transform_inv, (img_w, img_h))
        return lane_unwarp

    
    def run(self, img):
        corrected = self.undistort(img)
        top_down = self.transform_topdown(corrected)
        mask = self.treshold(top_down)
        
        self.detect_lane_lines(mask)

        self.average_lanes()

        lane_img = self.lane_image(mask.shape[1], mask.shape[0])
        
        return cv2.addWeighted(corrected, 1, lane_img, 0.3, 0)