#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <math.h>
#define PI 3.14159265
using namespace cv;
using namespace std;
Mat skull2=imread("skulln2.png");
Mat skull;
Mat result;
string sk="skull";
string image_window = "Source Image";
string result_window = "Result window";
int match_method;
int max_Trackbar = 5;
void drawStuff();
void drawAllTriangles(Mat&, const vector< vector<Point> >&);

void MatchingMethod( int, void* );

Mat img_rgb,img_gray,canny_output,drawing, frame,frame2;

int thresh = 100;
int max_thresh = 255;
int iLowH, iHighH;
int main(){
    skull2.copyTo(skull);
    //cvtColor(skull, skull, cv::COLOR_RGB2HSV);
    imshow("ro", skull);
    VideoCapture cap(0); //capture the video from web cam
    namedWindow( image_window, WINDOW_AUTOSIZE );
    namedWindow( result_window, WINDOW_AUTOSIZE );
    string trackbar_label = "Method: \n 0: TM COEFF NORMED \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: SQDIFF";
    createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }
    while (true)
    {
        Mat img_rgb;

         // read a new frame from video

         if (!cap.isOpened()) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }
        cap >> img_rgb;
        cap >> drawing;
        img_rgb.copyTo(frame2);
    GaussianBlur(img_rgb, frame, cv::Size(0, 0), 3);
    addWeighted(img_rgb, 1.5, frame, -0.5, 0, img_rgb);
    //namedWindow("Control", CV_WINDOW_AUTOSIZE);
    cvtColor(3*img_rgb,img_gray,CV_BGR2GRAY);

    drawStuff();
   //    imshow("gray",img_gray);
    
    if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       }
    }
}

void drawStuff(){
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Canny( img_gray, canny_output, 147, 255, 3 );
    //imshow("Canny",canny_output);
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    

    drawAllTriangles(drawing,contours);
    imshow("Triangles",drawing);
}

void drawAllTriangles(Mat& img, const vector<vector<Point> >& contours){
    vector<Point> approxTriangle;
    namedWindow("fraw", CV_WINDOW_AUTOSIZE);
    vector<Rect> boundRect( contours.size() );

    for(size_t i = 0; i < contours.size(); i++){
        if((contourArea(Mat(contours[i]))>50 && arcLength(Mat(contours[i]), true)>15)){
        approxPolyDP(contours[i], approxTriangle, arcLength(Mat(contours[i]), true)*0.015, true);
        if(approxTriangle.size() == 3 ){
           /* boundRect[i] = boundingRect( Mat(contours[i]) );
            resize(skull2, skull, Size((boundRect[i].width*7)/10, (boundRect[i].height*6)/10));

            drawContours(img, contours, i, Scalar(0, 0, 0), CV_FILLED);
            rectangle( img, boundRect[i].tl(), boundRect[i].br(), Scalar(0,0,0), 2, 8, 0 );
            Mat fraw = Mat(boundRect[i].height,boundRect[i].width, frame.type(), Scalar::all(0));
            Rect r1(boundRect[i].tl(),boundRect[i].br());
            frame2(r1).copyTo(fraw);
            
           // cvtColor(fraw, fraw, cv::COLOR_RGB2HSV);
            imshow("fraw", fraw);
            
            imshow(" ", skull);
            
            waitKey(1);
      /// Create the result matrix
            int result_cols =  fraw.cols - skull.cols + 1;
            int result_rows = fraw.rows - skull.rows + 1;

            result.create( result_cols, result_rows, CV_32FC1 );
            matchTemplate( fraw, skull, result, match_method );
            //normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
            double minVal; double maxVal; Point minLoc; Point maxLoc;
            Point matchLoc;
            minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
            if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED ){
                matchLoc = minLoc;
               // cout << minVal<<endl;
            }
            else{
                matchLoc = maxLoc;
               // cout << maxVal<< endl;
            } */
            vector<Point>::iterator vertex;
            vertex = approxTriangle.begin();
            Point pt1 = *vertex;
            ++vertex;
            Point pt2= *vertex;
            ++vertex;
            Point pt3= *vertex;
            double angle1, angle2, a, b, c;
            a= sqrt( (pt1.x-pt2.x)*(pt1.x-pt2.x) + (pt1.y-pt2.y)*(pt1.y-pt2.y));
            b= sqrt( (pt1.x-pt3.x)*(pt1.x-pt3.x) + (pt1.y-pt3.y)*(pt1.y-pt3.y));
            c= sqrt( (pt2.x-pt3.x)*(pt2.x-pt3.x) + (pt2.y-pt3.y)*(pt2.y-pt3.y));
            angle1 = acos((a*a+b*b-c*c)/(2*a*b))* 180/PI;
            angle2= acos((a*a + c*c - b*b)/ (2*a*c)) * 180/PI;
           // cout << angle1 << " "<< angle2<< endl;
            if(angle1<65 && angle1>55 && angle2<65 && angle2>55){
                cout << 1<< endl;
            }
            else{
                cout<< 0<< endl;
            }

            /*cout << endl;
            cout << endl;
            rectangle( fraw, matchLoc, Point( matchLoc.x + skull.cols , matchLoc.y + skull.rows ), Scalar::all(0), 2, 8, 0 );
            rectangle( result, matchLoc, Point( matchLoc.x + skull.cols , matchLoc.y + skull.rows ), Scalar::all(0), 2, 8, 0 );

            imshow( image_window, fraw );
            imshow( result_window, result );
            fraw.release();*/
    
             // fill GREEN
           /* vector<Point>::iterator vertex; 
            for(vertex = approxTriangle.begin(); vertex != approxTriangle.end(); ++vertex){
                circle(img, *vertex, 3, Scalar(0, 0, 255), 1);

               printf("triangle present\n");
                cout << contourArea(Mat(contours[i]))<<" "<< arcLength(Mat(contours[i]), true)<<endl;
            }*/
          //  cout << i << "no of triangles detected" << endl;
        }

    }
}
}

void MatchingMethod( int, void* ){

    return;



}

