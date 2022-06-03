#include <opencv2/opencv.hpp>
#include <iostream>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

vector<vector<Point>> contours;
vector<Vec4i> hierarchy;
//RNG rng(12345);

Point leftBottomMin = Point(0, 200);
Point leftBottomMax = Point(160, 400);
Point leftMiddleMin = Point(600, 100);
Point leftMiddleMax = Point(700, 300);
Point leftTopMin = Point(250, 200);
Point leftTopMax = Point(300, 300);

Point rightMiddleMin = Point(450, 70);
Point rightMiddleMax = Point(550, 240);
Point rightBottomMin = Point(950, 130);
Point rightBottomMax = Point(1300, 400);

int leftBottomCounter;
int leftMiddleCounter;
int leftTopCounter;

int rightBottomCounter;
int rightMiddleCounter;

bool comparePoints(int contourPointX, int contourPointY, int minX, int minY, int maxX, int maxY)
{
    if (contourPointX >= minX && contourPointY >= minY && contourPointX <= maxX && contourPointY <= maxY)
        return true;
    else
        return false;
}

int main()
{
    //video input test
    VideoCapture vid_capture("/home/sdv/catkin_ws/src/pilot/resources/demo.avi");

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

    // Initialise frame matrix
    Mat frame;

    // Read the frames to the last frame
    while (vid_capture.isOpened())
    {
        // Initialize a boolean to check if frames are there or not
        bool isSuccess = vid_capture.read(frame);

        if (isSuccess == false)
        {
            cout << "Video camera is disconnected" << endl;
            break;
        }

        //imshow("original", frame);

        //rescale so only the floor is visible
        frame = frame(Range(200, 600), Range(0, 1280));
        //imshow("cropped image", frame);

        Mat img_gray;
        cvtColor(frame, img_gray, COLOR_BGR2GRAY);

        //imshow("grayscale", img_gray);

        // Blur the image for better edge detection
        Mat img_blur;
        // GaussianBlur(frame, img_blur, Size(3, 3), 0);
        GaussianBlur(img_gray, img_blur, Size(7, 7), 0);
        //imshow("blured image", img_blur);

        // Canny edge detection
        Mat canny;
        //adjust threshold 1 and 2 that we only see the white line
        Canny(img_blur, canny, 30, 340, 3, false);
        //Canny(img_blur, canny, 50, 570, 3, false);

        // Display canny edge detected image
        //imshow("canny edge detection", canny);

        // Find contours
        findContours(canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        
        //imshow("contours", contours);

        vector<vector<Point>> contours_poly(contours.size());
        vector<Rect> boundRect(contours.size());
        vector<Point2f> centers(contours.size());
        vector<float> radius(contours.size());

        for (int i = 0; i < contours.size(); i++)
        {
            for (auto &j : contours[i])
            {
                //cout << "X: " << j.x << " Y: " << j.y << endl;

                //left bottom green
                if (comparePoints(j.x, j.y, leftBottomMin.x, leftBottomMin.y, leftBottomMax.x, leftBottomMax.y))
                {
                    //putText(frame, "leftBottom", Point(30, 100), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 255), 3, 2);
                    leftBottomCounter++;
                }

                //left top rectangle
                if (comparePoints(j.x, j.y, leftTopMin.x, leftTopMin.y, leftTopMax.x, leftTopMax.y))
                {
                    //putText(frame, "leftTop", Point(50, 200), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 255), 3, 2);
                    leftTopCounter++;
                }

                //left middle green
                if (comparePoints(j.x, j.y, leftMiddleMin.x, leftMiddleMin.y, leftMiddleMax.x, leftMiddleMax.y))
                {
                    //putText(frame, "leftMiddle", Point(700, 100), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 255), 3, 2);
                    leftMiddleCounter++;
                }

                //right bottom blue
                if (comparePoints(j.x, j.y, rightBottomMin.x, rightBottomMin.y, rightBottomMax.x, rightBottomMax.y))
                {
                    //putText(frame, "rightBottom", Point(900, 200), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 0, 255), 3, 2);
                    rightBottomCounter++;
                }

                //right middle blue
                if (comparePoints(j.x, j.y, rightMiddleMin.x, rightMiddleMin.y, rightMiddleMax.x, rightMiddleMax.y))
                {
                    //putText(frame, "rightMiddle", Point(200, 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 0, 255), 3, 2);
                    rightMiddleCounter++;
                }
            }
            // cout << "leftBottomCounter " << leftBottomCounter << endl;
            // cout << "leftMiddleCounter " << leftMiddleCounter << endl;
            // cout << "rightBottomCounter " << rightBottomCounter << endl;
            // cout << "rightMiddleCounter " << rightMiddleCounter << endl;

            drawContours(frame, contours, i, Scalar(0, 0, 255), 2, 8, hierarchy, 0, Point());
            //approxPolyDP(contours[i], contours_poly[i], 3, true);
            //boundRect[i] = boundingRect(contours_poly[i]);
        }

        int leftAmount = leftBottomCounter + leftMiddleCounter + leftTopCounter;
        int rightAmount = rightBottomCounter + rightMiddleCounter;

        //left bottom green
        rectangle(frame, leftBottomMin, leftBottomMax, Scalar(0, 255, 0), 2);
        //left top green
        rectangle(frame, leftTopMin, leftTopMax, Scalar(0, 255, 0), 2);
        //left middle green
        rectangle(frame, leftMiddleMin, leftMiddleMax, Scalar(0, 255, 0), 2);
        //left blue
        rectangle(frame, rightMiddleMin, rightMiddleMax, Scalar(255, 0, 0), 2);
        //right blue
        rectangle(frame, rightBottomMin, rightBottomMax, Scalar(255, 0, 0), 2);

        //if more than 20 points have been seen in both left areas, assume that the line goes to the right
        //if (leftBottomCounter >= 15 && leftMiddleCounter >= 15 && leftAmount > rightAmount)
        if (leftBottomCounter >= 15 && leftMiddleCounter >= 15 || leftTopCounter >= 15 && leftAmount > rightAmount)
        {
            //TODO steer car right
            putText(frame, "RIGHT", Point(550, 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 100, 255), 3, 2);
        }
        //if more than 20 points have been seen in both right areas, assume that the line goes to the left
        else if (rightBottomCounter >= 15 && rightMiddleCounter >= 15 && rightAmount > leftAmount)
        {
            //TODO steer car left
            putText(frame, "LEFT", Point(550, 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 100, 255), 3, 2);
        }
        else
        {
            //TODO steer car straight
            putText(frame, "STRAIGHT", Point(550, 50), FONT_HERSHEY_SIMPLEX, 2, Scalar(100, 100, 255), 3, 2);
        }

        leftBottomCounter = 0;
        leftTopCounter = 0;
        leftMiddleCounter = 0;
        rightBottomCounter = 0;
        rightMiddleCounter = 0;

        //imshow("Rectangles", drawing);
        imshow("Final result", frame);

        // If frames are not there, close it
        if (isSuccess == false)
        {
            cout << "Video is disconnected" << endl;
            break;
        }

        //wait 20 ms between successive frames and break the loop if key q is pressed
        if (waitKey(20) == 'q')
        {
            cout << "Stopping the video" << endl;
            vid_capture.release();
            destroyAllWindows();
            break;
        }
        if (waitKey(20) == 'p')
        {
            cout << "Pausing the video" << endl;
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

//-----------------------------------------------------------------------------------------------------------------
//------------------------------------------boundingRect-----------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

// //video input test
//     VideoCapture vid_capture("/home/sdv/catkin_ws/src/pilot/resources/trackRecording.avi");

//     double img_center;
//     bool left_flag = false;
//     bool right_flag = false;
//     cv::Point right_b;
//     double right_m;
//     cv::Point left_b;
//     double left_m;

//     vector<vector<Point>> contours;
//     vector<Vec4i> hierarchy;
//     RNG rng(12345);

//     // Print error message if the stream is invalid
//     if (!vid_capture.isOpened())
//     {
//         cout << "Error opening video stream or file" << endl;
//     }

//     else
//     {
//         // Obtain fps and frame count by get() method and print
//         // You can replace 5 with CAP_PROP_FPS as well, they are enumerations
//         int fps = vid_capture.get(5);
//         cout << "Frames per second: " << fps << endl;

//         // Obtain frame_count using opencv built in frame count reading method
//         // You can replace 7 with CAP_PROP_FRAME_COUNT as well, they are enumerations
//         int frame_count = vid_capture.get(7);
//         cout << "Frame count: " << frame_count << endl;
//     }

//     // Initialise frame matrix
//     Mat frame;

//     // Read the frames to the last frame
//     while (vid_capture.isOpened())
//     {
//         // Initialize a boolean to check if frames are there or not
//         bool isSuccess = vid_capture.read(frame);

//         //rescale so only the floor is visible
//         frame = frame(Range(290, 600), Range(0, 1280));

//         Mat img_gray;
//         cvtColor(frame, img_gray, COLOR_BGR2GRAY);
//         //threshold the image
//         Mat thresh;
//         threshold(img_gray, thresh, 150, 255, THRESH_BINARY);

//         // Blur the image for better edge detection
//         Mat img_blur;
//         GaussianBlur(img_gray, img_blur, Size(3, 3), 0);

//         // Canny edge detection
//         Mat canny;
//         //adjust threshold 1 and 2 that we only see the white line
//         Canny(img_blur, canny, 50, 570, 3, false);
//         // Display canny edge detected image

//         // Find contours
//         findContours(canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
//         vector<vector<Point>> contours_poly(contours.size());
//         vector<Rect> boundRect(contours.size());
//         vector<Point2f> centers(contours.size());
//         vector<float> radius(contours.size());

//         for (int i = 0; i < contours.size(); i++)
//         {
//             //cout << "contours: " << contours[i] << endl;
//             drawContours(frame, contours, i, Scalar(0, 0, 255), 2, 8, hierarchy, 0, Point());

//             approxPolyDP(contours[i], contours_poly[i], 3, true);
//             boundRect[i] = boundingRect(contours_poly[i]);

//             //cout << "rect: " << boundRect[i] << endl;
//             if (boundRect[i].x == 0 && boundRect[i].height + boundRect[i].y > 200)
//             {
//                 cout << "right" << endl;
//             }
//             else if (boundRect[i].x > 500){
//                 cout << "left" << endl;
//             }
//         }

//         Mat drawing = Mat::zeros(canny.size(), CV_8UC3);
//         for (size_t i = 0; i < contours.size(); i++)
//         {
//             Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
//             drawContours(drawing, contours_poly, (int)i, color);
//             rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
//         }

//         imshow("This is it", drawing);

//         imshow("Final result", frame);

//         // If frames are present, show it
//         if (isSuccess == true)
//         {
//             // If frames are not there, close it
//             if (isSuccess == false)
//             {
//                 cout << "Video camera is disconnected" << endl;
//                 break;
//             }

//             //wait 20 ms between successive frames and break the loop if key q is pressed
//             if (waitKey(20) == 'q')
//             {
//                 cout << "Stopping the video" << endl;
//                 break;
//             }
//             if (waitKey(20) == 'p')
//             {
//                 cout << "Pauzing the video" << endl;
//                 while (waitKey(20) != 'p')
//                 {
//                 }
//             }
//         }
//     }

//     // Release the video capture object
//     vid_capture.release();
//     destroyAllWindows();
//     return 0;

//-----------------------------------------------------------------------------------------------------------------
//--------------------------------------------inRange & contours---------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

// Mat originalFrame;
// Mat img_gray;
// Mat hsv_image;
// Mat edges;
// imshow("original", originalFrame);

//             cvtColor(frame, hsv_image, COLOR_BGR2HSV);

//             Mat imgThreshold1,
//                 imgThreshold2, imgThreshold;
//             inRange(hsv_image,
//                     Scalar(100, 10, 0),
//                     Scalar(115, 20, 20),
//                     imgThreshold1);

//             inRange(hsv_image,
//                     Scalar(225, 5, 50),
//                     Scalar(255, 10, 90),
//                     imgThreshold2);

//             imgThreshold = max(imgThreshold1, imgThreshold2); // combining the two thresholds

//             Mat element_erode = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
//             Mat element_dilate = getStructuringElement(MORPH_ELLIPSE, Size(10, 10));
//             /// Apply the erosion and dilation operations
//             erode(imgThreshold, imgThreshold, element_erode);
//             dilate(imgThreshold, imgThreshold, element_dilate);

//             GaussianBlur(imgThreshold, imgThreshold, Size(9, 9), 2, 2);

//             vector<vector<Point>> contours;
//             vector<Vec4i> hierarchy;
//             /// Find contours
//             findContours(edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

//             for (int i = 0; i < contours.size(); i++)
//             {
//                 drawContours(frame, contours, i, Scalar(0, 0, 255), 2, 8, hierarchy, 0, Point());
//             }

//             namedWindow("Display Image", WINDOW_AUTOSIZE);
//             imshow("Display Image", imgThreshold);
//             imshow("Final result", frame);

//-----------------------------------------------------------------------------------------------------------------
//--------------------------------------------HoughLinesP with dilation and erode----------------------------------
//-----------------------------------------------------------------------------------------------------------------

// //dilate(frame, dilation_dst, element);
//             dilate(frame, dilation_dst, Mat(), Point(-1, -1), 4, 1, 1);
//             erode(dilation_dst, erode_dst, Mat(), Point(-1, -1), 3, 1, 1);
//             //imshow("erode", erode_dst);

//             //imshow("Dilation Demo", dilation_dst);

//             //grayscale image
//             Mat img_gray;
//             //before dilate it was just frame
//             cvtColor(erode_dst, img_gray, COLOR_BGR2GRAY);

//             // Blur the image for better edge detection
//             Mat img_blur;
//             GaussianBlur(img_gray, img_blur, Size(3, 3), 0);

//             // Canny edge detection
//             Mat edges;
//             //adjust threshold 1 and 2 that only the white line is visible
//             Canny(img_blur, edges, 50, 570, 3, false);

//             vector<Vec4i> lines;
//             // HoughLinesP(edges, lines, 1, CV_PI / 180, 80, 30, 10);
//             HoughLinesP(edges, lines, 1, CV_PI / 180, 20, 20, 30);
//             cout << "size: " << lines.size() << endl;

//             for (auto &i : lines)
//             //for (size_t i = 0; i < lines.size(); i++)
//             {
//                 cout << "NEW SET" << endl;
//                 cout << "lines seen: " << lines.size() << endl;
//                 cout << i << endl;

//                 cout << "0" << Point(i[0], i[1]) << endl;
//                 cout << "1" << Point(i[2], i[3]) << endl;
//                 cout << "1" << Point(i[4], i[5]) << endl;
//                 cout << "1" << Point(i[6], i[7]) << endl;

//                 // line(img_gray,
//                 //      Point(i[0], i[1]),
//                 //      Point(i[2], i[3]),
//                 //      Scalar(0, 0, 0), 3, 8);

//                 // cout << "0" << Point(i[0], i[1]) << endl;
//                 // cout << "1" << Point(i[2], i[3]) << endl;
//                 // cout << "1" << Point(i[4], i[5]) << endl;
//                 // cout << "1" << Point(i[6], i[7]) << endl;

//                 cout << endl;
//                 cout << endl;

//                 line(erode_dst, Point(i[0], i[1]), Point(i[2], i[3]), Scalar(0, 255, 255), 5, CV_AA);
//                 // line(frame, Point(lines[i][4], lines[i][5]), Point(lines[i][6], lines[i][7]), Scalar(0, 0, 255), 5, CV_AA);

//                 imshow("Lane", erode_dst);
//             }

// Display canny edge detected image
//imshow("Canny edge detection", edges);
//imshow("Lines", erode_dst);
//imshow("Lane", frame);

//-----------------------------------------------------------------------------------------------------------------
//--------------------------------------------HoughLinesP----------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

// //display frames
//             imshow("Video", frame);
//             //cut the image so that only the part we need is visible
//             frame = frame(Range(300, 600), Range(0, 800));
//             //grayscale image
//             Mat img_gray;
//             cvtColor(frame, img_gray, COLOR_BGR2GRAY);

//             // Blur the image for better edge detection
//             Mat img_blur;
//             GaussianBlur(img_gray, img_blur, Size(3, 3), 0);

//             // Canny edge detection
//             Mat edges;
//             //adjust threshold 1 and 2 that only the white line is visible
//             Canny(img_blur, edges, 50, 570, 3, false);

//             vector<Vec4i> lines;
//             // HoughLinesP(edges, lines, 1, CV_PI / 180, 80, 30, 10);
//             HoughLinesP(edges, lines, 1, CV_PI / 180, 20, 20, 30);
//             Mat cdstP = img_gray.clone();
//             // Draw the lines
//-----------------------------------------------------------------------------------------------------------------
//---------------------------------------------contour-------------------------------------------------------------
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