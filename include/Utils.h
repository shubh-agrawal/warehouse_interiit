#ifndef UTILS_H
#define UTILS_H
#include "Image.h"
#include "opencv2/opencv.hpp"
#include <vector>
#include "Config.h"

using namespace std;
using namespace cv;


vector<ImagePatch> SlicePart(Mat im, vector<ImagePatch> images, int slices_h, int slices_w,std::vector<Point> &v){
	int sl_i = im.rows/slices_h;
	int sl_j = im.cols/slices_w;
	Mat crop;
	v.clear();
	for (int i = 0; i < slices_h; i++)
	{
		int part_i = sl_i*i;

		for (int j = 0; j < slices_w; j++)
		{
			int part_j = sl_j*j;
			Rect region = Rect(part_j,part_i,sl_j,sl_i);
			crop = im(region);
			images[i+slices_h*j].imag = crop.clone();
			images[i+slices_h*j].Process();
			if(images[i+slices_h*j].valid)
			{
				v.push_back(Point(images[i+slices_h*j].contourCenterX + part_j,
					images[i+slices_h*j].contourCenterY + part_i ));
			}
		}
	}

	return images;
}

Mat RepackImages(vector<ImagePatch> images, int slices_h, int slices_w)
{
	Mat img = images[0].imag.clone();
	Mat img_temp = images[0].imag.clone();

	for (int j = 0; j < slices_w; j++){
		for (int i = 0; i < slices_h; i++)
		{
			if(!i)
			{
				img_temp = images[i+(slices_h*j)].imag.clone();
			}
			else
			{
				vconcat(img_temp, images[i+(slices_h*j)].imag, img_temp);
			}
		}
		if(!j)
		{
			img = img_temp.clone();
		}
		else
		{
			hconcat(img, img_temp, img);
		}
	}
	return img;
}

Mat RemoveBackground(Mat image, bool b){
	Mat mask, img_erosion, masked_image;
	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));

    if(b)
    {
		cvtColor(image, image, CV_BGR2HSV);
		inRange(image, Scalar(H_LOW,S_LOW,V_LOW), Scalar(H_HIGH,S_HIGH,V_HIGH), mask);
		bitwise_and(image, image, masked_image, mask);
	    erode(masked_image, img_erosion, kernel, Point(-1, -1), 3);
	    dilate(img_erosion,img_erosion,kernel,Point(-1,-1),7);
	    return img_erosion;
	}
	else
	{
		return image;
	}

}

Mat ShowGrid(Mat im, int slices_h, int slices_w){
	int sl_i = im.rows/slices_h;
	int sl_j = im.cols/slices_w;
	for (int i = 0; i < slices_h; i++)
	{
		int part_i = sl_i*i;
		line(im,Point(0,part_i),Point(im.cols, part_i),Scalar(0,0,255),1);
	}
	for (int j = 0; j < slices_w; j++)
	{
		int part_j = sl_j*j;
		line(im,Point(part_j,0),Point(part_j, im.rows),Scalar(0,0,255),1);
	}
	return im;
}

#endif
