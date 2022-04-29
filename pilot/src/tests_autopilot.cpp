#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
    using namespace std;
    using namespace cv;

    //video input test
    VideoCapture vid_capture("/home/sdv/catkin_ws/src/pilot/resources/trackRecording.avi");

    double img_center;
    bool left_flag = false;
    bool right_flag = false;
    cv::Point right_b;
    double right_m;
    cv::Point left_b;
    double left_m;
    // Print error message if the stream is invalid
    if (!vid_capture.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
    }

    else
    {
        // Obtain fps and frame count by get() method and print
        // You can replace 5 with CAP_PROP_FPS as well, they are enumerations
        int fps = vid_capture.get(5);
        cout << "Frames per second: " << fps << endl;

        // Obtain frame_count using opencv built in frame count reading method
        // You can replace 7 with CAP_PROP_FRAME_COUNT as well, they are enumerations
        int frame_count = vid_capture.get(7);
        cout << "Frame count: " << frame_count << endl;
    }

    // Read the frames to the last frame
    while (vid_capture.isOpened())
    {
        // Initialise frame matrix
        Mat frame;

        // Initialize a boolean to check if frames are there or not
        bool isSuccess = vid_capture.read(frame);

        // If frames are present, show it
        if (isSuccess == true)
        {
            //display frames
            imshow("Video", frame);
            //cut the image so that only the part we need is visible
            frame = frame(Range(300, 600), Range(0, 800));
            //grayscale image
            Mat img_gray;
            cvtColor(frame, img_gray, COLOR_BGR2GRAY);

            // Blur the image for better edge detection
            Mat img_blur;
            GaussianBlur(img_gray, img_blur, Size(3, 3), 0);

            // Canny edge detection
            Mat edges;
            //adjust threshold 1 and 2 that only the white line is visible
            Canny(img_blur, edges, 50, 570, 3, false);

            vector<Vec4i> lines;
            // HoughLinesP(edges, lines, 1, CV_PI / 180, 80, 30, 10);
            HoughLinesP(edges, lines, 1, CV_PI / 180, 20, 20, 30);
            Mat cdstP = img_gray.clone();
            // Draw the lines
            

            // If frames are not there, close it
            if (isSuccess == false)
            {
                cout << "Video camera is disconnected" << endl;
                break;
            }

            //wait 20 ms between successive frames and break the loop if key q is pressed
            if (waitKey(20) == 'q')
            {
                cout << "Stopping the video" << endl;
                break;
            }
            if (waitKey(20) == 'p')
            {
                cout << "Pauzing the video" << endl;
                while (waitKey(20) != 'p')
                {
                }
            }
        }
        // Release the video capture object
        vid_capture.release();
        destroyAllWindows();
        return 0;
    }
}

//-----------------------------------------------------------------------------------------------------------------
//--------------------------------contour--------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

// Mat image = imread("/home/sdv/catkin_ws/src/pilot/resources/linesCP.jpg");
// //cut the image so that only the part we need is visible
// image = image(Range(300, 600), Range(0, 800));
// imshow("cropped_image", image);
// //grayscale image
// Mat img_gray;
// cvtColor(image, img_gray, COLOR_BGR2GRAY);
// //threshold the image
// //Mat thresh;
// //threshold(img_gray, thresh, 150, 255, THRESH_BINARY);
// imshow("gray", img_gray);

// // Blur the image for better edge detection
// Mat img_blur;
// GaussianBlur(img_gray, img_blur, Size(3, 3), 0);

// // Canny edge detection
// Mat edges;
// //adjust threshold 1 and 2 that we only see the white line
// Canny(img_blur, edges, 50, 570, 3, false);
// // Display canny edge detected image
// imshow("Canny edge detection", edges);
// waitKey(0);
// destroyAllWindows();

// /*
// Contour detection and drawing using different extraction modes to complement
// the understanding of hierarchies
// */
// Mat image2 = imread("/home/sdv/catkin_ws/src/pilot/resources/linesCP.jpg");
// Mat img_gray2;
// cvtColor(image2, img_gray2, COLOR_BGR2GRAY);
// Mat thresh2;
// threshold(img_gray2, thresh2, 150, 255, THRESH_BINARY);

// vector<vector<Point>> contours3;
// vector<Vec4i> hierarchy3;
// findContours(thresh2, contours3, hierarchy3, RETR_LIST, CHAIN_APPROX_NONE);
// Mat image_copy4 = image2.clone();
// drawContours(image_copy4, contours3, -1, Scalar(0, 255, 0), 2);
// imshow("LIST", image_copy4);
// waitKey(0);
// imwrite("contours_retr_list.jpg", image_copy4);
// destroyAllWindows();

// vector<vector<Point>> contours4;
// vector<Vec4i> hierarchy4;
// findContours(thresh2, contours4, hierarchy4, RETR_EXTERNAL, CHAIN_APPROX_NONE);
// Mat image_copy5 = image2.clone();
// drawContours(image_copy5, contours4, -1, Scalar(0, 255, 0), 2);
// imshow("LIST2", image_copy5);
// waitKey(0);
// imwrite("contours_retr_external.jpg", image_copy4);
// destroyAllWindows();

// vector<vector<Point>> contours5;
// vector<Vec4i> hierarchy5;
// findContours(thresh2, contours5, hierarchy5, RETR_CCOMP, CHAIN_APPROX_NONE);
// Mat image_copy6 = image2.clone();
// drawContours(image_copy6, contours5, -1, Scalar(0, 255, 0), 2);
// imshow("LIST3", image_copy6);
// waitKey(0);
// imwrite("contours_retr_ccomp.jpg", image_copy6);
// destroyAllWindows();

// vector<vector<Point>> contours6;
// vector<Vec4i> hierarchy6;
// findContours(thresh2, contours6, hierarchy6, RETR_TREE, CHAIN_APPROX_NONE);
// Mat image_copy7 = image2.clone();
// drawContours(image_copy7, contours6, -1, Scalar(0, 255, 0), 2);
// imshow("LIST4", image_copy7);
// waitKey(0);
// imwrite("contours_retr_tree.jpg", image_copy7);
// destroyAllWindows();

//-----------------------------------------------------------------------------------------------------------------
//--------------------------------edge--------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

// #include <opencv2/opencv.hpp>
// #include <iostream>
// // using namespaces to nullify use of cv::function(); syntax and std::function();
// using namespace std;
// using namespace cv;

// int main()
// {
//     // Reading image
//     Mat img = imread("/home/sdv/catkin_ws/src/pilot/linesFarCP.jpg");
//     // Display original image
//     imshow("original Image", img);
//     waitKey(0);

//     // Convert to graycsale
//     Mat img_gray;
//     cvtColor(img, img_gray, COLOR_BGR2GRAY);
//     // Blur the image for better edge detection
//     Mat img_blur;
//     GaussianBlur(img_gray, img_blur, Size(3,3), 0);

//     // Sobel edge detection
//     Mat sobelx, sobely, sobelxy;
//     Sobel(img_blur, sobelx, CV_64F, 1, 0, 5);
//     Sobel(img_blur, sobely, CV_64F, 0, 1, 5);
//     Sobel(img_blur, sobelxy, CV_64F, 1, 1, 5);
//     // Display Sobel edge detection images
//     imshow("Sobel X", sobelx);
//     waitKey(0);
//     imshow("Sobel Y", sobely);
//     waitKey(0);
//     imshow("Sobel XY using Sobel() function", sobelxy);
//     waitKey(0);

//     // Canny edge detection
//     Mat edges;
//     Canny(img_blur, edges, 100, 200, 3, false);
//     // Display canny edge detected image
//     imshow("Canny edge detection", edges);
//     waitKey(0);

//     destroyAllWindows();
//     return 0;
// }

//-----------------------------------------------------------------------------------------------------------------
//--------------------------------lines--------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
// #include <opencv2/opencv.hpp>
// #include <iostream>
// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/highgui/highgui.hpp"

// using namespace std;
// using namespace cv;

// int main() {
//    // read the image
//    Mat image = imread("/home/sdv/catkin_ws/src/pilot/linesCP.jpg");

//    // B, G, R channel splitting
//    Mat channels[3];
//    split(image, channels);

//    // detect contours using blue channel and without thresholding
//    vector<vector<Point>> contours1;
//    vector<Vec4i> hierarchy1;
//    findContours(channels[0], contours1, hierarchy1, RETR_TREE, CHAIN_APPROX_NONE);
//    // draw contours on the original image
//    Mat image_contour_blue = image.clone();
//    drawContours(image_contour_blue, contours1, -1, Scalar(0, 255, 0), 2);
//    //imshow("Contour detection using blue channels only", image_contour_blue);
//    waitKey(0);
//    imwrite("blue_channel.jpg", image_contour_blue);
//    destroyAllWindows();

//    // detect contours using green channel and without thresholding
//    vector<vector<Point>> contours2;
//    vector<Vec4i> hierarchy2;
//    findContours(channels[1], contours2, hierarchy2, RETR_TREE, CHAIN_APPROX_NONE);
//    // draw contours on the original image
//    Mat image_contour_green = image.clone();
//    drawContours(image_contour_green, contours2, -1, Scalar(0, 255, 0), 2);
//    imshow("Contour detection using green channels only", image_contour_green);
//    waitKey(0);
//    imwrite("green_channel.jpg", image_contour_green);
//    destroyAllWindows();

//    // detect contours using red channel and without thresholding
//    vector<vector<Point>> contours3;
//    vector<Vec4i> hierarchy3;
//    findContours(channels[2], contours3, hierarchy3, RETR_TREE, CHAIN_APPROX_NONE);
//    // draw contours on the original image
//    Mat image_contour_red = image.clone();
//    drawContours(image_contour_red, contours3, -1, Scalar(0, 255, 0), 2);
//    imshow("Contour detection using red channels only", image_contour_red);
//    waitKey(0);
//    imwrite("red_channel.jpg", image_contour_red);
//    destroyAllWindows();
// }