#ifndef VISUALFEATURES_H
#define VISUALFEATURES_H

#include <vector>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

class Visual_Features
{
private:
    int edge_threshold;
    int fast_threshold;
    int first_level;
    int max_features;
    int n_levels;
    int patch_size;
    int scale_factor;
    int score_type;
    int wta_k;


    const float inlier_threshold = 2.5f; // Distance threshold to identify inliers
    const float nn_match_ratio = 0.8f;   // Nearest neighbor matching ratio
public:
    Visual_Features();
    void start();
};

#endif // VISUALFEATURES_H
