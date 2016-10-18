#ifndef VIDEO_STREAM_H
#define VIDEO_STREAM_H

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Video_Stream
{
public:
    Video_Stream();
    ~Video_Stream();
    int start();
};

#endif // VIDEO_STREAM_H
