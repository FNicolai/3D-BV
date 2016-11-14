#include "visualfeatures.h"

using namespace std;
using namespace cv;


Visual_Features::Visual_Features()
{

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
    Mat obj_frame = imread( "object.png", IMREAD_GRAYSCALE );

    if( !obj_frame.data )
    { std::cout<< " --(!) Error reading image " << std::endl; return; }

    while(true){
        bool bSuccess = cap.read(img_frame);

        if(!bSuccess){
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        vector<KeyPoint> kpts_img, kpts_obj;
        Mat desc_img, desc_obj;

        if(img_frame.channels() > 1){
            cvtColor(img_frame,img_frame,CV_BGR2GRAY);
        }

        if(obj_frame.channels() > 1){
            cvtColor(obj_frame,obj_frame,CV_BGR2GRAY);
        }

        Ptr<AKAZE> akaze = AKAZE::create();
        akaze->detectAndCompute(img_frame, noArray(), kpts_img, desc_img);
        akaze->detectAndCompute(obj_frame, noArray(), kpts_obj, desc_obj);

        BFMatcher matcher( NORM_L2, false );
        vector<DMatch> matches;
        matcher.match( desc_obj, desc_img, matches );

        double max_dist = 0; double min_dist = 100;

        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < desc_obj.rows; i++ )
        { double dist = matches[i].distance;
          if( dist < min_dist ) min_dist = dist;
          if( dist > max_dist ) max_dist = dist;
        }
        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );
        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< DMatch > good_matches;
        for( int i = 0; i < desc_obj.rows; i++ )
        { if( matches[i].distance < 3*min_dist )
           { good_matches.push_back( matches[i]); }
        }

        Mat img_matches;
        drawMatches( obj_frame, kpts_obj, img_frame, kpts_img,
                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                     std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for( size_t i = 0; i < good_matches.size(); i++ )
        {
          //-- Get the keypoints from the good matches
          obj.push_back( kpts_obj[ good_matches[i].queryIdx ].pt );
          scene.push_back( kpts_img[ good_matches[i].trainIdx ].pt );
        }

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
        }
        catch (cv::Exception) {
        }
        imshow( "Good Matches & Object detection", img_matches );
        waitKey(30);
    }

    return;
}
