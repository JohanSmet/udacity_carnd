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


