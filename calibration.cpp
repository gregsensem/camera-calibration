#include "opencv2/opencv.hpp"
#include "fstream"

int main()
{
    const char* input = "/home/peter/Documents/SLAM Dataset/IMG_9812.MOV";
    cv::Size board_pattern(9,6);
    float board_cellsize = 0.025f;
    bool select_images = true;

    // Open a video
    cv::VideoCapture video;
    if (!video.open(input)) return -1;
    std::cout << "video opened!" << "\n";

    // Select images
    std::vector<cv::Mat> images;
    while (true)
    {
        // Grab an image from the video
        cv::Mat image;
        video >> image;
        if (image.empty()) break;


        // Show the image and keep it if selected
        cv::imshow("Camera Calibration", image);
        int key = cv::waitKey(1);
        if (key == 27) break;                               // 'ESC' key: Exit
        else if (key == 32)                                 // 'Space' key: Pause
        {
            std::vector<cv::Point2f> pts;
            bool complete = cv::findChessboardCorners(image, board_pattern, pts);
            cv::Mat display = image.clone();
            cv::drawChessboardCorners(display, board_pattern, pts, complete);
            cv::imshow("Camera Calibration", display);
            key = cv::waitKey();
            if (key == 27) break;                           // 'ESC' key: Exit
            else if (key == 121)
            {
                images.push_back(image);    // 'Enter' key: Select
                std::cout << "image added!";
            }
        }
    
        // else images.push_back(image);
    }
    video.release();
    if (images.empty())
    {
        std::cout << "image vector empty!" << "\n";
        return -1;
    } 

    // Find 2D corner points from the given images
    std::vector<std::vector<cv::Point2f>> img_points;
    for (size_t i = 0; i < images.size(); i++)
    {
        std::vector<cv::Point2f> pts;
        if (cv::findChessboardCorners(images[i], board_pattern, pts))
            img_points.push_back(pts);
    }
    if (img_points.empty())
    {
        std::cout << "img_points empty!" << "\n";
        return -1;
    } ;

    // Prepare 3D points of the chess board
    std::cout << "prepare 3D points of the chess board!" << "\n";

    std::vector<std::vector<cv::Point3f>> obj_points(1);
    for (int r = 0; r < board_pattern.height; r++)
        for (int c = 0; c < board_pattern.width; c++)
            obj_points[0].push_back(cv::Point3f(board_cellsize * c, board_cellsize * r, 0));
    obj_points.resize(img_points.size(), obj_points[0]); // Copy

    // Calibrate the camera
    std::cout << "calibrate the camera!" << "\n";

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeff = cv::Mat::zeros(4, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(obj_points, img_points, images[0].size(), K, dist_coeff, rvecs, tvecs);
    std::cout << K.row(0) << K.row(1) << K.row(2) << std::endl;
    // Report calibration results
    std::ofstream report("camera_calibration.txt");
    if (!report.is_open()) return -1;
    report << "## Camera Calibration Results" << std::endl;
    report << "* The number of applied images = " << img_points.size() << std::endl;
    report << "* RMS error = " << rms << std::endl;
    report << "* Camera matrix (K) = " << std::endl << "  " << K.row(0) << K.row(1) << K.row(2) << std::endl;
    report << "* Distortion coefficient (k1, k2, p1, p2, k3, ...) = " << std::endl << "  " << dist_coeff.t() << std::endl;
    report.close();
    return 0;
}
