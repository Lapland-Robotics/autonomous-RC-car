#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
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
            frame = frame(Range(290, 600), Range(0, 1280));
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

            //for (auto &i : lines)
            //cout << "size: " << lines.size() << endl;

            for (size_t i = 0; i < lines.size(); i++)
            {
                cout << "NEW SET" << endl;
                cout << "lines seen: " << lines.size() << endl;
                cout << i << endl;

                cout << "0" << Point(lines[i][0], lines[i][1]) << endl;
                cout << "1" << Point(lines[i][2], lines[i][3]) << endl;
                cout << "1" << Point(lines[i][4], lines[i][5]) << endl;
                cout << "1" << Point(lines[i][6], lines[i][7]) << endl;

                // line(img_gray,
                //      Point(i[0], i[1]),
                //      Point(i[2], i[3]),
                //      Scalar(0, 0, 0), 3, 8);

                // cout << "0" << Point(i[0], i[1]) << endl;
                // cout << "1" << Point(i[2], i[3]) << endl;
                // cout << "1" << Point(i[4], i[5]) << endl;
                // cout << "1" << Point(i[6], i[7]) << endl;

                cout << endl;
                cout << endl;

                line(frame, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 255, 255), 5, CV_AA);
                // line(frame, Point(lines[i][4], lines[i][5]), Point(lines[i][6], lines[i][7]), Scalar(0, 0, 255), 5, CV_AA);

                imshow("Lane", frame);
            }

            // Display canny edge detected image
            //imshow("Canny edge detection", edges);
            //imshow("Lines", lines);
            //imshow("Lane", frame);
        }

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