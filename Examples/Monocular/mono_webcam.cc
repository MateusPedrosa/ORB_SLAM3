#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <System.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_webcam path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Open webcam
    cv::VideoCapture cap(0); // 0 is default webcam
    
    if(!cap.isOpened())
    {
        cerr << "ERROR: Could not open webcam" << endl;
        return -1;
    }

    // Set resolution (optional, adjust as needed)
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    cout << endl << "-------" << endl;
    cout << "Start processing webcam ..." << endl;

    cv::Mat im;
    
    while(true)
    {
        // Capture frame
        cap >> im;
        
        if(im.empty())
        {
            cerr << "Failed to capture frame" << endl;
            break;
        }

        // Get timestamp
        auto tframe = std::chrono::steady_clock::now();
        double timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tframe.time_since_epoch()).count();

        // Pass the image to SLAM system
        SLAM.TrackMonocular(im, timestamp);

        // Check for exit key (ESC)
        if(cv::waitKey(1) == 27)
            break;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
