/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warranty, support or any guarantee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008

   AVAILABLE AT:
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130

   OPENCV WEBSITES:
     Homepage:      http://opencv.org
     Online docs:   http://docs.opencv.org
     Q&A forum:     http://answers.opencv.org
     Issue tracker: http://code.opencv.org
     GitHub:        https://github.com/Itseez/opencv/
   ************************************************** */

#include "stereo_calib.h"

using namespace cv;
using namespace std;

class StereoSettings
{
public:
    StereoSettings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType { INVALID, CAMERAS, VIDEO_FILE, IMAGE_LIST };

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{"
                  << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "Square_Size"         << squareSize
                  << "Calibrate_Pattern" << patternToUse
                  << "Calibrate_NrOfFrameToUse" << nrFrames
                  << "Calibrate_FixAspectRatio" << aspectRatio
                  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                  << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

                  << "Write_DetectedFeaturePoints" << writePoints
                  << "Write_extrinsicParameters"   << writeExtrinsics
                  << "Write_outputFileName"  << outputFileName

                  << "Show_UndistortedImage" << showUndistorsed

                  << "Input_FlipAroundHorizontalAxis" << flipVertical
                  << "Input_Delay" << delay
                  << "LeftInputID" << leftInput
                  << "RightInputID" << rightInput
           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> writePoints;
        node["Write_extrinsicParameters"] >> writeExtrinsics;
        node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Calibrate_UseFisheyeModel"] >> useFisheye;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["InputType"] >> inputTypeString;
        node["LeftInputID"] >> leftInput;
        node["RightInputID"] >> rightInput;
        node["DataPath"] >> dataPath;
        node["ImageListFile"] >> imageListFile;
        node["IntrinsicsFile"] >> intrinsicsFile;
        node["Input_Delay"] >> delay;
        validate();
    }
    void validate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        if (inputTypeString.empty())
            inputType = INVALID;
        else
        {
            if (inputTypeString.compare("IMAGELIST") == 0)
            {
                if( imageList.size() % 2 != 0 )
                {
                    cout << "Error: the image list contains odd (non-even) number of elements\n";
                    return;
                }
                else
                {
                    bool ok = readStringList(dataPath, imageListFile, imageList);
                    if(!ok || imageList.empty())
                    {
                        cout << "can not open " << dataPath << " or the string list is empty" << endl;
                        return;
                    }
                    inputType = IMAGE_LIST;
                }
            }
            else if (inputTypeString.compare("CAMERAS") == 0)
            {
                if (leftInput[0] >= '0' && leftInput[0] <= '9'&&
                    rightInput[0] >= '0' && rightInput[0] <= '9')
                {
                    stringstream ssl(leftInput);
                    ssl >> leftCameraID;
                    leftInputCapture.open(leftCameraID);

                    stringstream ssr(rightInput);
                    ssr >> rightCameraID;
                    rightInputCapture.open(rightCameraID);

                    inputType = CAMERAS;
                }
                else
                    inputType = INVALID;
            }
        }

        flag = CALIB_FIX_K4 | CALIB_FIX_K5;
        if(calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;

        if (useFisheye) {
            // the fisheye model has its own enum, so overwrite the flags
            flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC |
                   // fisheye::CALIB_FIX_K1 |
                   fisheye::CALIB_FIX_K2 | fisheye::CALIB_FIX_K3 | fisheye::CALIB_FIX_K4;
        }

        // NL 161022: hard coded for the beginning
        calibrationPattern = CHESSBOARD;

        atImageList = 0;

    }

    Mat nextRightImage()
    {
        Mat result;
        if( rightInputCapture.isOpened() )
        {
            Mat view0, gray;
            rightInputCapture >> view0;
            cvtColor(view0, gray, CV_BGR2GRAY);
            gray.copyTo(result);
        }
        else if( atImageList < imageList.size() )
        {
            // make sure we're pointing at a right image in the list (even numbers)
            if (atImageList % 2 == 0) {
                atImageList++;
            }
            result = imread(imageList[atImageList++], IMREAD_GRAYSCALE);
        }

        return result;
    }
    Mat nextLeftImage()
    {
        Mat result;
        if( leftInputCapture.isOpened() )
        {
            Mat view0, gray;
            leftInputCapture >> view0;
            cvtColor(view0, gray, CV_BGR2GRAY);
            gray.copyTo(result);
        }
        else if( atImageList < imageList.size() )
        {
            // make sure we're pointing at a left image in the list (odd numbers)
            if (atImageList % 2 != 0) {
                atImageList++;
            }
            result = imread(imageList[atImageList++], IMREAD_GRAYSCALE);
        }

        return result;
    }

    static bool readStringList( const string& prefix, const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(prefix + filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back(prefix + (string)*it);
        return true;
    }
public:
    Size boardSize;              // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;                // The number of frames to use from the input for calibration
    float aspectRatio;           // The aspect ratio
    int delay;                   // In case of a video input
    bool writePoints;            // Write detected feature points
    bool writeExtrinsics;        // Write extrinsic parameters
    bool calibZeroTangentDist;   // Assume zero tangential distortion
    bool calibFixPrincipalPoint; // Fix the principal point at the center
    bool flipVertical;           // Flip the captured images around the horizontal axis
    string outputFileName;       // The name of the file where to write
    bool showUndistorsed;        // Show undistorted images after calibration
    string leftInput;            // Left input ->
    string rightInput;           // Right input ->
    string inputTypeString;
    string dataPath;
    string imageListFile;
    string intrinsicsFile;
    vector<string> imageList;
    InputType inputType;
    bool useFisheye;             // use fisheye camera model for calibration

    int leftCameraID;
    int rightCameraID;
    size_t atImageList;
    VideoCapture leftInputCapture;
    VideoCapture rightInputCapture;
    bool goodInput;
    int flag;

private:
    string patternToUse;

};

static inline void read(const FileNode& node, StereoSettings& x, const StereoSettings& default_value = StereoSettings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

static inline void write(FileStorage& fs, const String&, const StereoSettings& s )
{
    s.write(fs);
}

static void
StereoCalib(StereoSettings &settings, bool displayCorners = false, bool useCalibrated=true, bool showRectified=true)
{
    const int maxScale = 2;
    const float squareSize = 1.f;  // Set this to your actual square size
    // ARRAY AND VECTOR STORAGE:

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    int i, j, k, nimages = settings.inputType == StereoSettings::IMAGE_LIST ? (int)settings.imageList.size()/2 : settings.nrFrames;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<Mat> goodImageList;

    if (settings.inputType == StereoSettings::CAMERAS) {
        cout << "You have selected CAMERAS as an input source." << endl <<
                "Please make sure that the chessboard is visible in both camera pictures while selecting appropriate images." << endl;
    }

    for( i = j = 0; i < nimages; i++ )
    {
        Mat leftImage = settings.nextLeftImage();
        Mat rightImage = settings.nextRightImage();

        if (settings.inputType == StereoSettings::CAMERAS) {

            while (true) {
                Mat capl, viewl, capr, viewr;
                vector<Point2f> pointBufL;
                vector<Point2f> pointBufR;

                settings.leftInputCapture >> capl;
                capl.copyTo(viewl);
                Mat viewlGray;
                cvtColor(viewl, viewlGray, COLOR_BGR2GRAY);
                bool found_left = findChessboardCorners( viewlGray, settings.boardSize, pointBufL,
                                  CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

                settings.rightInputCapture >> capr;
                capr.copyTo(viewr);
                Mat viewrGray;
                cvtColor(viewr, viewrGray, COLOR_BGR2GRAY);
                bool found_right = findChessboardCorners(viewrGray, settings.boardSize, pointBufR,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

//                cout << "found left: " << (int) (found_left ? 1 : 0)
//                     << " found right: " << (int) (found_right ? 1 : 0)  << endl;

                if (found_left)
                {
                    drawChessboardCorners( viewl, settings.boardSize, Mat(pointBufL), found_left );
                }
                if (found_right)
                {
                    drawChessboardCorners( viewr, settings.boardSize, Mat(pointBufR), found_right );
                }


                imshow("viewl", viewl);
                imshow("viewr", viewr);
                if( found_right && found_left )
                {
                    cvtColor(capl, capl, COLOR_BGR2GRAY);
                    leftImage = capl;
                    cvtColor(capr, capr, COLOR_BGR2GRAY);
                    rightImage = capr;
                    cout << nimages - i << " to go" << endl;
                    break;
                }
                waitKey(1);
            }
        }
        else
        {
            leftImage = settings.nextLeftImage();
            rightImage = settings.nextRightImage();
        }


        for( k = 0; k < 2; k++ )
        {
            Mat img = k % 2 ? leftImage : rightImage;

            if(img.empty())
                break;
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                cout << "The image has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            vector<Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
                found = findChessboardCorners(timg, settings.boardSize, corners,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( displayCorners )
            {
//                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, settings.boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf);
                imshow("corners", cimg1);
                char c = (char)waitKey(500);
                if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                    exit(-1);
            }
            else
                putchar('.');
            if( !found )
                break;
            cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                      30, 0.01));
        }
        if( k == 2 )
        {
            goodImageList.push_back(leftImage);
            goodImageList.push_back(rightImage);
            j++;
        }
    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        cout << "Error: too little pairs to run the calibration\n";
        return;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < settings.boardSize.height; j++ )
            for( k = 0; k < settings.boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
    }

    cout << "Running stereo calibration ...\n";

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = initCameraMatrix2D(objectPoints,imagePoints[0],imageSize,0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints,imagePoints[1],imageSize,0);
    Mat R, T, E, F;

    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    CALIB_FIX_ASPECT_RATIO +
                    CALIB_ZERO_TANGENT_DIST +
                    CALIB_USE_INTRINSIC_GUESS +
                    CALIB_SAME_FOCAL_LENGTH +
                    CALIB_RATIONAL_MODEL +
                    CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                    TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );
    cout << "done with RMS error=" << rms << endl;

// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average epipolar err = " <<  err/npoints << endl;

    // save intrinsic parameters
    FileStorage fs(settings.dataPath + settings.intrinsicsFile, FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    fs.open("extrinsics.yml", FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the extrinsic parameters\n";

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

// COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return;

    Mat rmap[2][2];
// IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
// OR ELSE HARTLEY'S METHOD
    else
 // use intrinsic parameters of each camera, but
 // compute the rectification transformation directly
 // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }

    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
//            Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
            Mat img = goodImageList[i*2+k], rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
            cvtColor(rimg, cimg, COLOR_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
            if( useCalibrated )
            {
                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        char c = (char)waitKey();
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
    }
}


//static bool readStringList( const string& filename, vector<string>& l )
//{
//    l.resize(0);
//    FileStorage fs(filename, FileStorage::READ);
//    if( !fs.isOpened() )
//        return false;
//    FileNode n = fs.getFirstTopLevelNode();
//    if( n.type() != FileNode::SEQ )
//        return false;
//    FileNodeIterator it = n.begin(), it_end = n.end();
//    for( ; it != it_end; ++it )
//        l.push_back((string)*it);
//    return true;
//}


Stereo_Calibration::Stereo_Calibration()
{

}

int Stereo_Calibration::start()
{
    bool showRectified;

    StereoSettings s;
    const string inputSettingsFile = "../TemplateCalibration/stereo_calib_config.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();

    //TODO: add config value
    showRectified = true;

    StereoCalib(s, true, true, showRectified);
    return 0;
}
