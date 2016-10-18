#include "video_stream.h"

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

Video_Stream::Video_Stream()
{

}

Video_Stream::~Video_Stream()
{

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

    while(true){

        Mat frame;
        bool bSuccess = cap.read(frame);

        if(!bSuccess){
            cout << "Cannot read a frame from video stream" << endl;
            break;
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
