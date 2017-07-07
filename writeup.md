## Writeup Template

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/chessboard.png "Undistorted Chessboard"
[image2]: ./output_images/undistorted.png "Road Transformed"
[image3]: ./output_images/colorspaces.png "Colorspace Channels"
[image4]: ./output_images/gradient_threshold.png "Gradient Threshold"
[image5]: ./output_images/threshold.png "Threshold Example"
[image6]: ./output_images/perspective_transform.png "Example of perspective transform"
[image7]: ./output_images/sliding_window.jpg "Example of sliding window search"
[image8]: ./output_images/margin_search.jpg "Example of margin search"
[image9]: ./output_images/output_example.png "Example of resulting image"
[video1]: https://cdn.rawgit.com/JohanSmet/CarND-Advanced-Lane-Lines/0011fb7b/output_images/project_video.mp4 "Processed Project Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the calibrate function of the `Camera` class located in the camera.py file.

The results of the calibration are saved to a file. The first step of the calibration function is to check if this file exists from a previous execution and load it instead of running the entire calibration again. This will save us a lot of time in the next stage of the project.

The first real step of the calibration is preparing the "object points", which are the XYZ-coordinates of the chessboard corners in world space. By assuming the chessboard is fixed to the XY-plane (Z=0) these points will be the same for each calibration image.

Then the function iterates through each image and uses OpenCV to find the internal corners of the chessboard in each image. These coordinates are saved for the next step. This process can fail if the image does not contain the entire chessboard.

The last step uses OpenCV again to calculate the camera matrix and distance coefficients.

The `Camera` class also has a function to undistort images using the calculated information, called `undistort()`.
An example of running this function on a chessboard image is reproduced below.

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

The distortion-correction is done by the `undistort()` function of the `Camera` class (located in camera.py) by using the OpenCV `undistort()` function and the previously calculated camera matrix and distance coefficients. Here you can see the effect of the functions when applied to one of the test images:
![alt text][image2]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I experimented a lot with different combinations for this step. In fact it feels like I spent the majority of my time on this project fiddling with these settings. You can see the final implementation in the `LaneDetectionPipeline` class located in pipeline.py in the function `threshold()` and helper functions `gradient_threshold()` and `color_threshold()`.

I started with a gradient threshold by applying the Sobel operator in the x-direction to emphasize near vertical lines. At first I combined this with a color threshold on a grayscaled image. This worked reasonably well for the provided test images but broke down on the project video. I found that the gradient threshold resulted in to chaotic images with much unwanted artifacts. Here is an example of the gradient threshold:
![alt text][image4]

The next step was to look at the color space conversion provided by OpenCV and seeing which channel had the best qualities to detect certain colors. Here is an overview of a few different color representations:
![alt text][image3]

Several of these look like good candidates: a combination of the green and red channel can be used to detect yellow lines, the 'L'uminance channel of HLS accentuates white lines, the 'V'alue channel of HSV shows the white and yellow lines and finally the 'B' (blue-yellow) channel of LAB seems ideal to detect yellow lines.

I experimented with quite a few combinations but in the end I settled on a combination of the 'L' and 'B' channels. The next image shows an example of this filter:
![alt text][image5]

This works very well on the test images and the project video. I also tried to vary the minimum threshold based on the average brightness of the image in an effort to improve results on the first challenge video. I applied the threshold after the perspective transform to limit the area of interest. This helps a bit but still does not create a perfect result.

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The `LaneDetectionPipeline` class contains a function called `compute_perspective_transform()` that uses OpenCV to compute a perspective transform matrix based on hard-coded points. The `transform_topdown()` takes an image as input and applies this transform to it.

I chose to keep it simple and hard-coded the following source and destination points : 

| Source    | Destination |
|:---------:|:-----------:|
|  565, 475 | 384,200     |
|  720, 475 | 896,200     |
| 1030, 675 | 896,720     |
|  280, 675 | 384,720     |

I verified these points and the transform by drawing the source points on a distortion corrected image and displaying the result of the transform:

![alt text][image6]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

The `LaneDetectionPipeline` class contains two functions that try to identify lane-line pixels:
* `sliding_window_search()` implements a histogram based sliding window search that start from scratch
* `margin_search()` starts from detected lane-lines and searches a small area around them for new pixels

The `lane_from_points()` functions takes the detected pixels and fits a second-order polynomial to it using the NumPy polyfit function.

The sliding window search starts by computing the histogram of the lower half of the image. The biggest peaks on the left and right side respectively are used as the starting points for the lane lines. A window of predetermined width and height is centered on these starting points and all true pixels in the window are considered a part of the line. The windows then moves up and are potentially recentered around the horizontal mean of the detected pixels. This makes the windows follow a curved line. The next image is an example of this process: 

![alt text][image7]

The sliding window search is only used when no lane-lines were previously detected. When lane-lines are available the pipeline switches to searching the vicinity of those lines. This implemented in the `margin_search()` function and not only speeds up the operation but has the added benefit of stabilizing the detected lane-lines. An example of the result of the margin-search function:

![alt text][image8]

Finally the `lane_from_points()` function is used to fit a polynomial function to the points. The pipeline stores the last few detected lines and the average of these is used to further smooth the resulting lines. The newly detected lines are compared to the current average and if the polynomial factors differ too much the new line is rejected as an outlier.

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

These calculations are performed in the `lane_measurements()` function of the `LaneDetectionPipeline` class.
The result of the calculations has to be expressed in meters and not in pixels so the first step is to define conversion factors from image space to world space. I used the US standard regulations to get lane width and length of an line segment and measured the respective size of this on a few perspective projected test images to derive the current factors.

Next the function converts the currently detected lines to world space and performs the calculations to get the radius of curvature of the lane and the offset of the camera with respect to the center of the lane. For this we assume the camera is in the middle of the image.

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in the `lane_image()` function of the `LaneDetectionPipeline` class. This function uses OpenCV to draw a filled poly bounded by the detected lane-lines on a blank image. This image is transformed back from the top-down view to the camera view by applying the reverse perspective transformation.

Later on this image is blended with the distortion corrected input image. Here is an example of my result on a frame of the project video:

![alt text][image9]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).


The processed project video is included in the GitHup repo:
![alt text][video1]

I also uploaded to YouTube once with and once without the debug overly. Click on the images below to view the videos:

[![Project Video](https://img.youtube.com/vi/a_g7BlnXAXw/0.jpg)](https://www.youtube.com/watch?v=a_g7BlnXAXw)

[![Project Video](https://img.youtube.com/vi/bgZ47s5omnI/0.jpg)](https://www.youtube.com/watch?v=bgZ47s5omnI)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

The first issue I faced was getting the perspective transformation to give decent results. It took some tweaking of the source and destinations points but I think I got a decent top-down view with parallel lane lines.

The biggest problem was trying to generalize the thresholding function to work in different lighting conditions. I tried lots of different combination to try the get the pipeline to work on all three videos. The setup now works great on the project video but only on about 80% of the challenge video if I tweak the threshold values. I wanted to get better results but I ran out of time and decided to concentrate on the project video. 

The pipeline now fails in very bright or dark scenarios. I would imagine different weather conditions would also be catastrophic. I tried varying the thresholds based on the brightness but maybe some other heuristic (e.g. number of detected pixels) might provide better results. Maybe a different type of camera (near-infrared or uv) might help in these situations?

Very winding roads also cause some problems for the polyfit function. Only predicting a smaller piece could help or maybe splitting the image in sections and mapping multiple second order polynomial functions. 