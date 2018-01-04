#ifndef CONFIG_H
#define CONFIG_H

//Line Detection and Clustering Constants
#define CONTOUR_RADIUS 3
#define H_LOW 18	//Low Hue for thresholding
#define S_LOW 30	//Low Saturation for thresholding
#define V_LOW 80	//Low Value for thresholding
#define H_HIGH 31	//High Hue for thresholding
#define S_HIGH 255	//High Saturation for thresholding
#define V_HIGH 250	//High Value for thresholding
#define N_SLICE_H 15	//Number of height slices of image for contour detection
#define N_SLICE_W 18	//Number of width slices of image for contour detection 
#define POINT_RATIO 15.0   //Ratio of Right points to Left for L detection
#define POINT_THRESHOLD 100   //Min points required by hough transform to form a line
#define ANGLE_THRESHOLD 30    //Angle difference required for hough transform to draw a line
#define VERTICAL_TOPIC "/lines/vertical"    //Publish Topic for Vertical Line
#define HORIZONTAL_TOPIC "/lines/horizontal"    //Publish Topic for Horizontal Lines
#define CAM_TOPIC "/ardrone/image_raw"    //Publish Topic for Camera Feed



//Barcode Constants

#define BAR_CAM  "/ardrone/image_raw" //"/ardrone/front/image_rect_color"
#define BARCODE_SUB "/usb_cam/image_raw"	//Front Facing Camera Topic
#define BARCODE_IMG_TOPIC "/code/image"		//Topic to Publish Detected QR/Bar Code Rectangle
#define QR_TOPIC "/code/qr"			//Publish QR Code Data
#define BAR_TOPIC "/code/bar"			//Publish Bar Code Data
#define LOW_BINARY_THRESHOLD 180		//Minimum Threshold for Binary Conversion 

#endif
