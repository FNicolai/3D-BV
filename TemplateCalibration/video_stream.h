#ifndef VIDEO_STREAM_H
#define VIDEO_STREAM_H

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class Video_Stream
{
public:
    static void on_canny_lowThresh_trackbar( int, void* );
    enum filter_mode {NONE, CANNY, SOBEL, CORNERHARRIS};
    Video_Stream(filter_mode filter);
    ~Video_Stream();
    int start();


private:
   filter_mode _filter;

   int canny_lowThreshold;
   int canny_ratio;
   int canny_kernel_size;

   int sobel_scale;
   int sobel_delta;

   int corner_harris_blockSize;
   int corner_harris_apertureSize;
   int corner_harris_k;
   int corner_harris_k_max = 100;
   int corner_harris_thresh;

   void doSobel(cv::Mat &src, cv::Mat &dest);
   void doCornerHarris(cv::Mat &src, cv::Mat &dest);
};

#endif // VIDEO_STREAM_H
