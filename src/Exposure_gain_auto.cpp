#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace cv;

// Function to calculate the brightness of an image
double CalculateBrightness(const ImagePtr& image)
{
    unsigned char* pData = (unsigned char*)image->GetData();
    int width = image->GetWidth();
    int height = image->GetHeight();
    int numPixels = width * height;
    
    double brightness = 0.0;
    for (int i = 0; i < numPixels; ++i)
    {
        brightness += pData[i];
    }
    return brightness / numPixels;
}

// Function to calculate the histogram of an image
void CalcHistogram(const ImagePtr& image, Mat& hist)
{
    unsigned char* pData = (unsigned char*)image->GetData();
    int width = image->GetWidth();
    int height = image->GetHeight();
    
    Mat img(height, width, CV_8UC1, pData);
    int histSize = 256;
    float range[] = { 0, 256 };
    const float* histRange = { range };

    calcHist(&img, 1, 0, Mat(), hist, 1, &histSize, &histRange);
}

// Function to check for over-exposure
double CalculateOverExposureRatio(const Mat& hist, int threshold = 255)
{
    float totalPixels = sum(hist)[0];
    float overExposedPixels = hist.at<float>(threshold);

    return overExposedPixels / totalPixels;
}

// Function to adjust exposure and gain based on the current brightness
int AdjustExposureGain(CameraPtr pCam, double targetBrightness, double currentBrightness, double overExposureRatio)
{
    INodeMap& nodeMap = pCam->GetNodeMap();

    CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
    CFloatPtr ptrGain = nodeMap.GetNode("Gain");

    if (!IsReadable(ptrExposureTime) || !IsWritable(ptrExposureTime) || 
        !IsReadable(ptrGain) || !IsWritable(ptrGain))
    {
        cout << "Unable to access exposure time or gain. Aborting..." << endl;
        return 2;
    }

    double exposureTime = ptrExposureTime->GetValue();
    double gain = ptrGain->GetValue();
    double brightnessRatio = targetBrightness / currentBrightness;
    double adjustmentFactor = 0.1; // Adjust factor

    if ((currentBrightness < targetBrightness ) &&
        overExposureRatio < 0.03)
    {
        return 1;
    }
    else
    {
        if (currentBrightness < targetBrightness)
        {
            if (gain < ptrGain->GetMax())
            {
                gain = min(gain * (1.0 + adjustmentFactor * (brightnessRatio - 1.0)), ptrGain->GetMax());
            }
            else
            {
                exposureTime = min(exposureTime * (1.0 + adjustmentFactor * (brightnessRatio - 1.0)), ptrExposureTime->GetMax());
            }
        }
        else
        {
            if (gain > ptrGain->GetMin())
            {
                gain = max(gain * (1.0 + adjustmentFactor * (brightnessRatio - 1.0)), ptrGain->GetMin());
            }
            else
            {
                exposureTime = max(exposureTime * (1.0 + adjustmentFactor * (brightnessRatio - 1.0)), ptrExposureTime->GetMin());
            }
        }
    }

    ptrExposureTime->SetValue(exposureTime);
    ptrGain->SetValue(gain);

    return 0;
}

// Function to acquire images and adjust exposure and gain based on brightness
int AcquireAndAdjustImages(CameraPtr pCam, double targetBrightness, chrono::duration<double>& adjustDuration)
{
    int result = 0;

    cout << endl << "*** IMAGE ACQUISITION AND ADJUSTMENT ***" << endl << endl;

    try
    {
        pCam->BeginAcquisition();

        INodeMap& nodeMap = pCam->GetNodeMap();
        CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
        CEnumerationPtr gainAuto = nodeMap.GetNode("GainAuto");

        int imageCnt = 0;
        ImageProcessor processor;
        processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);

        auto start = chrono::steady_clock::now();
        auto adjustStart = start;
        bool adjusting = true;

        while (true)
        {   
            ImagePtr pResultImage = pCam->GetNextImage();

            if (pResultImage->IsIncomplete())
            {
                cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << endl;
            }
            else
            {   
                if (adjusting)
                {
                    adjustStart = chrono::steady_clock::now();
                }

                // Calculate histogram and over-exposure ratio
                Mat hist;
                CalcHistogram(pResultImage, hist);
                double overExposureRatio = CalculateOverExposureRatio(hist);

                double currentBrightness = CalculateBrightness(pResultImage);
                int adjust_re = AdjustExposureGain(pCam, targetBrightness, currentBrightness, overExposureRatio);
                
                if (adjust_re == 1)
                {
                    cout << "Finished adjustment!" << endl;
                    adjusting = false;

                    // Enable automatic exposure and gain
                    if (IsReadable(ptrExposureAuto) && IsWritable(ptrExposureAuto))
                    {
                        ptrExposureAuto->SetIntValue(ptrExposureAuto->GetEntryByName("Continuous")->GetValue());
                        cout << "Automatic exposure enabled..." << endl;
                    }
                    if (IsReadable(gainAuto) && IsWritable(gainAuto))
                    {
                        gainAuto->SetIntValue(gainAuto->GetEntryByName("Continuous")->GetValue());
                        cout << "Automatic gain enabled..." << endl;
                    }
                }
                else if (adjust_re == 2)
                {
                    cout << "Gain and Exposure not accessible. End project" << endl;
                    break;
                }

                if (!adjusting)
                {
                    auto now = chrono::steady_clock::now();
                    chrono::duration<double> elapsed = now - start;
                    if (elapsed.count() >= 0.5)
                    {
                        // Save image every 0.5 seconds
                        ImagePtr convertedImage = processor.Convert(pResultImage, PixelFormat_Mono8);
                        ostringstream filename;
                        string newPath = "/home/ruan-x/rsworkSpace/src/acquisition/auto_adj/";

                        filename << newPath << "Exposure_Gain_auto-";
                        filename << imageCnt << ".jpg";
                        imageCnt++;

                        convertedImage->Save(filename.str().c_str());
                        cout << "Image saved at " << filename.str() << endl;

                        start = chrono::steady_clock::now(); // Reset timer
                    }
                }
            }
            pResultImage->Release();

            if (!adjusting)
            {
                break;
            }
        }

        adjustDuration = chrono::steady_clock::now() - adjustStart;

        pCam->EndAcquisition();
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// Main function to run the camera
int RunSingleCamera(CameraPtr pCam)
{
    int result = 0;
    double targetBrightness = 125.0;  // Desired brightness level
    chrono::duration<double> adjustDuration(0);

    try
    {
        pCam->Init();
        pCam->BeginAcquisition();

        INodeMap& nodeMap = pCam->GetNodeMap();
        CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
        CEnumerationPtr gainAuto = nodeMap.GetNode("GainAuto");

        if (IsReadable(ptrExposureAuto) && IsWritable(ptrExposureAuto))
        {
            ptrExposureAuto->SetIntValue(ptrExposureAuto->GetEntryByName("Continuous")->GetValue());
            cout << "Automatic exposure enabled..." << endl;
        }

        if (IsReadable(gainAuto) && IsWritable(gainAuto))
        {
            gainAuto->SetIntValue(gainAuto->GetEntryByName("Continuous")->GetValue());
            cout << "Automatic gain enabled..." << endl;
        }

        int imageCnt = 0;
        ImageProcessor processor;
        processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);

        auto start = chrono::steady_clock::now();
        while (true)
        {
            ImagePtr pResultImage = pCam->GetNextImage();

            if (pResultImage->IsIncomplete())
            {
                cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << endl;
            }
            else
            {
                // Calculate histogram and over-exposure ratio
                Mat hist;
                CalcHistogram(pResultImage, hist);
                double overExposureRatio = CalculateOverExposureRatio(hist);

                double currentBrightness = CalculateBrightness(pResultImage);

                // Check if the current conditions are satisfied
                if ((currentBrightness >= targetBrightness) && (overExposureRatio < 0.03))
                {
                    auto now = chrono::steady_clock::now();
                    chrono::duration<double> elapsed = now - start;
                    if (elapsed.count() >= 0.5)
                    {
                        // Save image every 0.5 seconds
                        ImagePtr convertedImage = processor.Convert(pResultImage, PixelFormat_Mono8);
                        ostringstream filename;
                        string newPath = "/home/ruan-x/rsworkSpace/src/acquisition/auto_adj/";

                        filename << newPath << "Exposure_Gain_auto-";
                        filename << imageCnt << ".jpg";
                        imageCnt++;

                        convertedImage->Save(filename.str().c_str());
                        cout << "Image saved at " << filename.str() << endl;

                        start = chrono::steady_clock::now(); // Reset timer
                    }
                }
                else
                {
                    // Switch to manual adjustment mode
                    if (IsReadable(ptrExposureAuto) && IsWritable(ptrExposureAuto))
                    {
                        ptrExposureAuto->SetIntValue(ptrExposureAuto->GetEntryByName("Off")->GetValue());
                        cout << "Automatic exposure disabled..." << endl;
                    }
                    if (IsReadable(gainAuto) && IsWritable(gainAuto))
                    {
                        gainAuto->SetIntValue(gainAuto->GetEntryByName("Off")->GetValue());
                        cout << "Automatic gain disabled..." << endl;
                    }

                    pCam->EndAcquisition();
                    result = AcquireAndAdjustImages(pCam, targetBrightness, adjustDuration);
                    break;
                }
            }
            pResultImage->Release();
        }

        pCam->DeInit();
        pCam = nullptr;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    cout << "Time taken to adjust parameters: " << adjustDuration.count() << " seconds" << endl;

    return result;
}

// Main entry point
int main(int /*argc*/, char** /*argv*/)
{
    SystemPtr system = System::GetInstance();
    CameraList camList = system->GetCameras();
    CameraPtr pCam = nullptr;

    if (camList.GetSize() == 0)
    {
        cout << "No cameras detected." << endl;
        system->ReleaseInstance();
        return -1;
    }

    pCam = camList.GetByIndex(0);
    int result = RunSingleCamera(pCam);

    camList.Clear();
    system->ReleaseInstance();

    return result;
}
