#include "visualfeatures.h"

using namespace std;
using namespace cv;


Visual_Features::Visual_Features(matcher_mode matcher_)
{
    _matcher = matcher_;
}

void
Visual_Features::start()
{
    VideoCapture cap(0);

    if(!cap.isOpened()){
        cout << "Cannot open the video cam" << endl;
        return;
    }

    Mat img_frame;
    Mat obj_frame = imread( "../TemplateCalibration/data/object.jpg", IMREAD_GRAYSCALE );

    double scaling = 0.2;
    resize(obj_frame,obj_frame, cv::Size(), scaling, scaling);

    if( !obj_frame.data ){
        std::cout<< " --(!) Error reading image " << std::endl; return;
    }

    vector<KeyPoint> kpts_img, kpts_obj;
    Mat desc_img, desc_obj;

    Ptr<AKAZE> akaze = AKAZE::create();
    akaze->detectAndCompute(obj_frame, noArray(), kpts_obj, desc_obj);

    while(true){
        bool bSuccess = cap.read(img_frame);

        if(!bSuccess){
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        if(img_frame.channels() > 1){
            cvtColor(img_frame,img_frame,CV_BGR2GRAY);
        }

        akaze->detectAndCompute(img_frame, noArray(), kpts_img, desc_img);

        //-- Step 2: Matching descriptor vectors using BF or FLANN matcher
        vector<DMatch> matches;

        switch (_matcher) {
        case BRUTE_FORCE_MATCHER:
        {
            BFMatcher BFmatcher( NORM_L2, false );
            BFmatcher.match( desc_obj, desc_img, matches );
            break;
        }
        case FLANN_BASED_MATCHER:
        {
            if(desc_obj.type()!=CV_32F) {
                desc_obj.convertTo(desc_obj, CV_32F);
            }
            if(desc_img.type()!=CV_32F) {
                desc_img.convertTo(desc_img, CV_32F);
            }
            FlannBasedMatcher FBmatcher;
            FBmatcher.match( desc_obj, desc_img, matches );
            break;
        }
        default:
            exit(-1);
            break;
        }

        double max_dist = 0; double min_dist = 100;

        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < desc_obj.rows; i++ ) {
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );

        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< DMatch > good_matches;
        for( int i = 0; i < desc_obj.rows; i++ ) {
            if( matches[i].distance < 3*min_dist ) {
                good_matches.push_back( matches[i]);
            }
        }

        Mat img_matches;
        drawMatches( obj_frame, kpts_obj, img_frame, kpts_img,
                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                     std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for( size_t i = 0; i < good_matches.size(); i++ ) {
            //-- Get the keypoints from the good matches
            obj.push_back( kpts_obj[ good_matches[i].queryIdx ].pt );
            scene.push_back( kpts_img[ good_matches[i].trainIdx ].pt );
        }


        if(obj.size() != 0 && scene.size() != 0){
            try {
            Mat H = findHomography( obj, scene, RANSAC );
            //-- Get the corners from the image_1 ( the object to be "detected" )
            std::vector<Point2f> obj_corners(4);
            obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( obj_frame.cols, 0 );
            obj_corners[2] = cvPoint( obj_frame.cols, obj_frame.rows ); obj_corners[3] = cvPoint( 0, obj_frame.rows );
            std::vector<Point2f> scene_corners(4);

            perspectiveTransform( obj_corners, scene_corners, H);
            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line( img_matches, scene_corners[0] + Point2f( obj_frame.cols, 0), scene_corners[1] + Point2f( obj_frame.cols, 0), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( obj_frame.cols, 0), scene_corners[2] + Point2f( obj_frame.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( obj_frame.cols, 0), scene_corners[3] + Point2f( obj_frame.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( obj_frame.cols, 0), scene_corners[0] + Point2f( obj_frame.cols, 0), Scalar( 0, 255, 0), 4 );
            //-- Show detected matches
            } catch (cv::Exception) {
            }
        }

        imshow( "Good Matches & Object detection", img_matches );
        waitKey(30);
    }

    return;
}
