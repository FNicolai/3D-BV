#include "video_stream.h"

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

Video_Stream::Video_Stream(filter_mode filter)
{
    _filter = filter;
}

Video_Stream::~Video_Stream()
{

}

void Video_Stream::doSobel(Mat &src, Mat &dest){

    Mat grad,grad_x, grad_y, abs_grad_x, abs_grad_y;

    int ddepth = CV_16S;

    /// Gradient X
    Sobel( src, grad_x, ddepth, 1, 0, 3, sobel_scale, sobel_delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    Sobel( src, grad_y, ddepth, 0, 1, 3, sobel_scale, sobel_delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

    dest = grad;

    return;
}

void Video_Stream::doCornerHarris(Mat &src, Mat &dest)
{

  Mat dst_norm, dst_norm_scaled;
  dest = Mat::zeros( src.size(), CV_32FC1 );

  /// Detecting corners
  cornerHarris( src, dest, corner_harris_blockSize, corner_harris_apertureSize, (double) corner_harris_k / (double) corner_harris_k_max, BORDER_DEFAULT );

  /// Normalizing
  normalize( dest, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );

  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
     { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > corner_harris_thresh )
              {
               circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
              }
          }
     }

  dst_norm_scaled.copyTo(dest);
}


int Video_Stream::start()
{
    VideoCapture cap(0);

    if(!cap.isOpened()){
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    const char ESC_KEY = 27;

    cout << "Frame size:" << dWidth << "x" << dHeight << endl;

    namedWindow("Camera (ESC to exit)",CV_WINDOW_AUTOSIZE);
    moveWindow("Camera (ESC to exit)",500,200);

    if (_filter != NONE)
        namedWindow("Filters", 1);

    switch (_filter) {
    case CANNY:
        canny_lowThreshold = 50;
        createTrackbar( "Canny LowThresh", "Filters", &canny_lowThreshold, 50);
        canny_ratio = 0;
        createTrackbar( "Canny Ratio", "Filters", &canny_ratio, 50 );
        canny_kernel_size = 3;
        createTrackbar( "Canny kernel size", "Filters", &canny_kernel_size, 7 );
        break;
    case SOBEL:
        sobel_scale = 1;
        createTrackbar( "Sobel Scale", "Filters", &sobel_scale, 50);
        sobel_delta = 0;
        createTrackbar( "Sobel Delta", "Filters", &sobel_delta, 50 );
        break;
    case CORNERHARRIS:
        corner_harris_blockSize = 2;
        createTrackbar( "Corner Harris Blocksize", "Filters", &corner_harris_blockSize, 50);
        corner_harris_apertureSize = 3;
        createTrackbar( "Corner Harris apertureSize", "Filters", &corner_harris_apertureSize, 50);
        corner_harris_k = 4;
        createTrackbar( "Corner Harris k", "Filters", &corner_harris_k, 100);
        corner_harris_thresh = 200;
        createTrackbar( "Corner Harris threshold", "Filters", &corner_harris_thresh, 500);
    }


    while(true){

        Mat frame;
        bool bSuccess = cap.read(frame);

        if(!bSuccess){
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        switch (_filter) {
        case CANNY:
        {
            if(frame.channels() > 1){
                cvtColor(frame,frame,CV_BGR2GRAY);
            }

            Mat detected_edges = frame.clone();

            if (canny_kernel_size % 2 && canny_kernel_size > 2) {
                Canny( detected_edges, detected_edges, canny_lowThreshold, canny_lowThreshold*canny_ratio, canny_kernel_size );
            }
            else {
                cout << "Please make sure you set a valid value for the kernel size!" << endl;
            }

            detected_edges.copyTo(frame);
            break;
        }
        case SOBEL:
        {
            if(frame.channels() > 1){
                cvtColor(frame,frame,CV_BGR2GRAY);
            }
            doSobel(frame, frame);
            break;
        }
        case CORNERHARRIS:
        {
            if(frame.channels() > 1){
                cvtColor(frame,frame,CV_BGR2GRAY);
            }
            Mat inputFrame;
            frame.copyTo(inputFrame);
            doCornerHarris(inputFrame, frame);
        }
        }

        imshow("Camera (ESC to exit)",frame);

        if( (char)waitKey(30) == ESC_KEY){
            cout << "Esc key is pressed by user" << endl;
            destroyWindow("Camera (ESC to exit)");

            break;
        }
    }
    return 0;
}
