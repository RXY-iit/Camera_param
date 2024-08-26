#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>
#include <opencv2/opencv.hpp> // OpenCV library for video saving and image display

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace std;

void SetExposureAndGain(CameraPtr cam, double exposureTime, double gain)
{
    try
    {
        // Turn off automatic exposure and gain
        cam->ExposureAuto.SetValue(ExposureAuto_Off);
        cam->GainAuto.SetValue(GainAuto_Off);

        // Ensure the exposure and gain values are within allowed ranges
        double minExposure = cam->ExposureTime.GetMin();
        double maxExposure = cam->ExposureTime.GetMax();
        double minGain = cam->Gain.GetMin();
        double maxGain = cam->Gain.GetMax();

        exposureTime = min(max(exposureTime, minExposure), maxExposure);
        gain = min(max(gain, minGain), maxGain);

        // Set exposure time and gain
        cam->ExposureTime.SetValue(exposureTime);
        cam->Gain.SetValue(gain);

        cout << "Exposure set to " << exposureTime << " and gain set to " << gain << "." << endl;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
    }
}

void EnableAutoExposureAndGain(CameraPtr cam)
{
    try
    {
        cam->ExposureAuto.SetValue(ExposureAuto_Continuous);
        cam->GainAuto.SetValue(GainAuto_Continuous);
        cout << "Auto exposure and gain enabled." << endl;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
    }
}

void MinimizeOverexposure(CameraPtr cam)
{
    try
    {
        cam->BeginAcquisition();

        ImagePtr image = cam->GetNextImage();

        if (image->IsIncomplete())
        {
            cout << "Image incomplete with image status " << image->GetImageStatus() << endl;
        }
        else
        {
            unsigned int width = image->GetWidth();
            unsigned int height = image->GetHeight();

            unsigned char* pData = (unsigned char*)image->GetData();

            vector<int> histogram(256, 0);

            for (unsigned int i = 0; i < width * height; i++)
            {
                histogram[pData[i]]++;
            }

            int peakBrightness = max_element(histogram.begin(), histogram.end()) - histogram.begin();

            if (peakBrightness > 200)
            {
                double currentExposure = cam->ExposureTime.GetValue();
                double currentGain = cam->Gain.GetValue();

                double newExposure = max(1000.0, currentExposure * 0.9);
                double newGain = max(0.0, currentGain * 0.9);

                SetExposureAndGain(cam, newExposure, newGain);
            }
        }

        image->Release();
        cam->EndAcquisition();
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
    }
}
int main()
{
    // Path for saving video and images
    string savePath = "/home/ruan-x/rsworkSpace/src/acquisition/auto_adj/";

    SystemPtr system = System::GetInstance();
    CameraList camList = system->GetCameras();

    if (camList.GetSize() == 0)
    {
        cout << "No camera detected." << endl;
        camList.Clear();
        system->ReleaseInstance();
        return -1;
    }

    CameraPtr cam = camList.GetByIndex(0);
    cam->Init();

    try
    {
        char input;
        cv::VideoWriter videoWriter;
        bool recording = false;
        double fps = 30.0;
        unsigned int width = cam->Width.GetValue();
        unsigned int height = cam->Height.GetValue();
        string videoFileName = savePath + "output.avi";
        double currentExposure = cam->ExposureTime.GetValue();
        double currentGain = cam->Gain.GetValue();
        string currentmode;

        // Initialize video writer
        videoWriter.open(videoFileName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(width, height));

        if (!videoWriter.isOpened())
        {
            cout << "Could not open the video file for writing." << endl;
            cam->DeInit();
            cam = nullptr;
            camList.Clear();
            system->ReleaseInstance();
            return -1;
        }

        cam->BeginAcquisition();
        recording = true;
        cout << "Recording started..." << endl;

        ImagePtr image; // Declare ImagePtr outside the switch statement

        while (true)
        {
            if (cin.rdbuf()->in_avail()) // check if there's input available
            {
                cin >> input;

                switch (input)
                {
                case 'a':
                    EnableAutoExposureAndGain(cam);
                    currentmode="Build-in auto";
                    break;
                case 'n':
                    SetExposureAndGain(cam, 5000.0, 25.0);
                    currentmode="mode 1";
                    break;
                case 'm':
                    SetExposureAndGain(cam, 6000.0, 30.0);
                    currentmode="mode 2";
                    break;
                case 'h':
                    MinimizeOverexposure(cam);
                    currentmode="mode histogram";
                    break;
                case 's':
                    // Save the current image
                    image = cam->GetNextImage();
                    if (!image->IsIncomplete())
                    {
                        stringstream imgFileName;
                        imgFileName << savePath << "image_" << time(0) << ".jpg";
                        image->Save(imgFileName.str().c_str());
                        cout << "Image saved: " << imgFileName.str() << endl;
                    }
                    image->Release();
                    break;
                case 'q':
                    cout << "Quitting..." << endl;
                    recording = false;
                    break;
                default:
                    cout << "Invalid input. Please try again." << endl;
                    break;
                }

                if (!recording)
                {
                    break;
                }
            }

            // Continuously capture and save video frames with current settings
            image = cam->GetNextImage();
            if (!image->IsIncomplete())
            {
                unsigned char* pData = static_cast<unsigned char*>(image->GetData());
                cv::Mat frame(cv::Size(width, height), CV_8UC1, pData);
                cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

                // Overlay exposure time and gain on the frame
                currentExposure = cam->ExposureTime.GetValue();
                currentGain = cam->Gain.GetValue();
                stringstream ss;
                ss << "Current Mode: " << currentmode << " Exposure: " << currentExposure << "us, Gain: " << currentGain << "dB";
                cv::putText(frame, ss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

                // Write the frame to the video file
                videoWriter.write(frame);
            }
            image->Release();
        }

        if (recording)
        {
            videoWriter.release();
            cam->EndAcquisition();
            cout << "Recording stopped." << endl;
        }
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
    }

    cam->DeInit();
    cam = nullptr;

    camList.Clear();
    system->ReleaseInstance();

    return 0;
}
