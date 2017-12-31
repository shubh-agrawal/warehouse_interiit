#ifndef UTILS_H
#define UTILS_H
#include "Image.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include "Config.h"

using namespace std;
using namespace cv;


class para : public ParallelLoopBody
{
public :
	para (const Mat img,std::vector<Point> &v,vector<ImagePatch> &images,const int s_i, const int s_j)
		: m_img(img), m_v(v), m_images(images),sl_i(s_i),sl_j(s_j)
	{
	}

	virtual void operator ()(const Range& range) const 
	{
		for (int k = range.start; k < range.end; k++)
		{
			int i = k/N_SLICE_H;
			int j = k % N_SLICE_H;
			int part_i = sl_i*i;
			int part_j = sl_j*j;
			Rect region = Rect(part_j,part_i,sl_j,sl_i);
			m_images[i+N_SLICE_H*j].imag = m_img(region).clone();
			m_images[i+N_SLICE_H*j].Process();
			if(m_images[i+N_SLICE_H*j].valid)
			{
				m_v.push_back(Point(m_images[i+N_SLICE_H*j].contourCenterX + part_j,
					m_images[i+N_SLICE_H*j].contourCenterY + part_i ));
			}
		}
	}
private :
	Mat m_img;
	std::vector<Point> &m_v;
	vector<ImagePatch> &m_images;
	int sl_j;
	int sl_i;
};

vector<ImagePatch> SlicePart(Mat im, vector<ImagePatch> images, int slices_h, int slices_w,std::vector<Point> &v){
	int sl_i = im.rows/slices_h;
	int sl_j = im.cols/slices_w;
	Mat crop;
	v.clear();
	double t1 = (double) getTickCount();
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
	t1 = ((double) getTickCount() - t1) / getTickFrequency();
    // cout << "Single Slice: " << t1 << " s" << endl;
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

Mat SlicePartParallel(Mat im,std::vector<Point> &v){
	int sl_i = im.rows/N_SLICE_H;
	int sl_j = im.cols/N_SLICE_W;
	Mat crop;
	v.clear();
	vector<ImagePatch> images(N_SLICE_W*N_SLICE_H);
	double t1 = (double) getTickCount();

	#ifdef CV_CXX11
	parallel_for_(Range(0, N_SLICE_W*N_SLICE_H), [&](const Range& range){
		for (int k = range.start; k < range.end; k++)
		{
			int i = k/N_SLICE_H;
			int j = k % N_SLICE_H;
			int part_i = sl_i*i;
			int part_j = sl_j*j;
			Rect region = Rect(part_j,part_i,sl_j,sl_i);
			crop = im(region);
			images[i+N_SLICE_H*j].imag = crop.clone();
			images[i+N_SLICE_H*j].Process();
			if(images[i+N_SLICE_H*j].valid)
			{
				v.push_back(Point(images[i+N_SLICE_H*j].contourCenterX + part_j,
					images[i+N_SLICE_H*j].contourCenterY + part_i ));
			}
		}
		      
    });
    #else
    // cout<<"C++11 Support Required for faster performance";
    para parallelSlice(im,v,images,sl_i,sl_j);
    parallel_for_(Range(0,N_SLICE_W*N_SLICE_H),parallelSlice);
    #endif 

    t1 = ((double) getTickCount() - t1) / getTickFrequency();
    // cout << "Parallel Slice: " << t1 << " s" << endl;,
    return RepackImages(images,N_SLICE_H,N_SLICE_W);
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
