#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>


class Camera_Calibration
{
public:
    Camera_Calibration();
    int start();
};

#endif // CAMERA_CALIBRATION_H
