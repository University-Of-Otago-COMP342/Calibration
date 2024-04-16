#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <iomanip> // For std::setfill and std::setw

int main(int argc, char* argv[]) {
    /*
    cv::namedWindow("Left");
    cv::namedWindow("Right");

    char fname[255];

    
    for (int i = 457; i < 476; ++i) {
        std::ostringstream pathStreamL;
        pathStreamL << "data/CalibrationLeft/DSCF" 
           << std::setfill('0') << std::setw(4) << i 
           << "_L.JPG";
        std::string fnameL = pathStreamL.str(); // Converts the stream to a string

        cv::Mat imageL = cv::imread(fnameL);

        //vector to hold points
        std::vector<cv::Point2f> cornersL;

        //Find the chessboard pattern
        bool patternWasFoundL = cv::findChessboardCorners(imageL, cv::Size(5, 10), cornersL);

        //Draw the found pattern on the image
        cv::drawChessboardCorners(imageL, cv::Size(5, 10), cornersL, patternWasFoundL);

        cv::imshow("Left", imageL);
        cv::waitKey(10);

        std::ostringstream pathStreamR;
        pathStreamR << "data/CalibrationRight/DSCF" 
           << std::setfill('0') << std::setw(4) << i 
           << "_R.JPG";
        std::string fnameR = pathStreamR.str(); // Converts the stream to a string

        cv::Mat imageR = cv::imread(fnameR);

        //vector to hold points
        std::vector<cv::Point2f> cornersR;

        //Find the chessboard pattern
        bool patternWasFoundR = cv::findChessboardCorners(imageR, cv::Size(5, 10), cornersR);

        //Draw the found pattern on the image
        cv::drawChessboardCorners(imageR, cv::Size(5, 10), cornersR, patternWasFoundR);

        cv::imshow("Right", imageR);
        cv::waitKey(100);

    }
    */

    //the objectPoints and imagePoints vectors will be populated in an upcoming for loop
    std::vector<std::vector<cv::Point3f>> objectPoints1;
    std::vector<std::vector<cv::Point3f>> objectPoints2;
    std::vector<std::vector<cv::Point2f>> imagePoints1;
    std::vector<std::vector<cv::Point2f>> imagePoints2;
    cv::Size imageSize(1920, 1080); // in pixels
    cv::Size patternSize(5, 10); // in squares
    //items below are outputs from the camera calibration function
    cv::Mat K1;
    cv::Mat K2;
    std::vector<double> distCoeffs1;
    std::vector<double> distCoeffs2;
    std::vector<cv::Mat> rvecs, tvecs;
    //this checkerboard pattern must be completed.
    std::vector<cv::Point3f> checkerboardPattern;

    // Fill checkerboardPattern with the 3D corner locations for each image
    for (int y = 0; y < patternSize.height; ++y) {
        for (int x = 0; x < patternSize.width; ++x) {
            checkerboardPattern.push_back(cv::Point3f(x, y, 0));
        }
    }

    for (int j = 457; j < 476; ++j) {
        std::ostringstream pathStream;
        pathStream << "data/CalibrationLeft/DSCF"
            << std::setfill('0') << std::setw(4) << j
            << "_L.JPG";

        std::string fname = pathStream.str(); // Converts the stream to a string

        cv::Mat image = cv::imread(fname);

        std::vector<cv::Point2f> corners;
        cv::findChessboardCorners(image, patternSize, corners);

        imagePoints1.push_back(corners);
        objectPoints1.push_back(checkerboardPattern);
    }

    double err = cv::calibrateCamera(objectPoints1, imagePoints1, imageSize, K1, distCoeffs1, rvecs, tvecs);

    std::cout << "The reprojection error for the Left camera:" << err << std::endl;

    std::cout << "The calibration matrix for the Left camera:\n" << K1 << std::endl;

    for (int j = 457; j < 476; ++j) {
        std::ostringstream pathStream;
        pathStream << "data/CalibrationRight/DSCF"
            << std::setfill('0') << std::setw(4) << j
            << "_R.JPG";

        std::string fname = pathStream.str(); // Converts the stream to a string

        cv::Mat image = cv::imread(fname);

        std::vector<cv::Point2f> corners;
        cv::findChessboardCorners(image, patternSize, corners);

        imagePoints2.push_back(corners);
        objectPoints2.push_back(checkerboardPattern);
    }

    err = cv::calibrateCamera(objectPoints2, imagePoints2, imageSize, K2, distCoeffs2, rvecs, tvecs);

    std::cout << "The reprojection error for the Right camera:" << err << std::endl;

    std::cout << "The calibration matrix for the Right camera:\n" << K2 << std::endl;

    //stereo calibration

    cv::Mat R(3, 3, CV_64FC1);
    cv::Mat T(3, 1, CV_64FC1);
    cv::Mat E(3, 3, CV_64FC1);
    cv::Mat F(3, 3, CV_64FC1);

    err = cv::stereoCalibrate(objectPoints1, imagePoints1, imagePoints2,
        K1, distCoeffs1, K2, distCoeffs2, imageSize,
        R, T, E, F);

    std::cout << "The reprojection error for the Stereo set of cameras:" << err << std::endl;
    std::cout << "The rotation between the two cameras is:\n" << R << std::endl;
    std::cout << "The translation between the two cameras is:\n" << T << std::endl;
    std::cout << "The essential matrix for the two cameras is:\n" << E << std::endl;
    std::cout << "The fundamental matrix for the two cameras is:\n" << F << std::endl;

    return 0;
}

