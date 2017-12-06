#ifndef CONFIG_H
#define CONFIG_H

//Line Detection and Clustering Constants
#define CONTOUR_RADIUS 7
#define H_LOW 15	//Low Hue for thresholding
#define S_LOW 150	//Low Saturation for thresholding
#define V_LOW 150	//Low Value for thresholding
#define H_HIGH 30	//High Hue for thresholding
#define S_HIGH 255	//High Saturation for thresholding
#define V_HIGH 255	//High Value for thresholding
#define N_SLICE_H 16	//Number of height slices of image for contour detection
#define N_SLICE_W 15	//Number of width slices of image for contour detection 
#define POINT_RATIO 7.0   //Ratio of Right points to Left for L detection
#define POINT_THRESHOLD 100   //Min points required by hough transform to form a line
#define ANGLE_THRESHOLD 30    //Angle difference required for hough transform to draw a line
#define VERTICAL_TOPIC "/lines/vertical"    //Publish Topic for Vertical Line
#define HORIZONTAL_TOPIC "/lines/horizontal"    //Publish Topic for Horizontal Lines
#define CAM_TOPIC "/cgo3_camera/image_raw"    //Publish Topic for Camera Feed

#endif
