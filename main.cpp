#include "mainwindow.h"
#include <QApplication>

#include <stdio.h>
#include <iostream>
#include <opencv/cv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    //w.show();

    VideoCapture cap(0);

    if(!cap.isOpened()){
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    cout << "Frame size:" << dWidth << "x" << dHeight << endl;

    namedWindow("LaptopCam",CV_WINDOW_AUTOSIZE);
    moveWindow("LaptopCam",500,200);

    while(true){

        Mat frame;
        bool bSuccess = cap.read(frame);

        if(!bSuccess){
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        imshow("LaptopCam",frame);

        if(waitKey(30)==27){
            cout << "Esc key is pressed by user" << endl;
            break;
        }
    }

    //return 0;
    return a.exec();
}
