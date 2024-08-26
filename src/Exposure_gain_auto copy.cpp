#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

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

// Function to adjust exposure and gain based on the current brightness
int AdjustExposureGain(CameraPtr pCam, double targetBrightness, double currentBrightness)
{
    INodeMap& nodeMap = pCam->GetNodeMap();

    // change exposuretime and Gain
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
    if ((currentBrightness < targetBrightness+25) && (currentBrightness > targetBrightness-25)){
        return 1;
    }else{
        //////// Method 1 ///////////////
        // if (currentBrightness < targetBrightness)
        // {
        //     exposureTime *= 1.1;
        //     gain *= 1.1;
        // }
        // else
        // {
        //     exposureTime *= 0.9;
        //     gain *= 0.9;
        // }

        //////// Method 2 ///////////////
        double brightnessRatio = targetBrightness / currentBrightness;
        double exposureAdjustmentFactor = 0.1; // Adjust exposure by this factor of the brightness ratio
        double gainAdjustmentFactor = 0.05;     // Adjust gain by this factor of the brightness ratio

        // Adjust exposure time
        exposureTime *= (1.0 + exposureAdjustmentFactor * (brightnessRatio - 1.0));
        // Adjust gain
        gain *= (1.0 + gainAdjustmentFactor * (brightnessRatio - 1.0));
    }
    
    // insure not expand the min max
    exposureTime = min(max(exposureTime, ptrExposureTime->GetMin()), ptrExposureTime->GetMax());
    gain = min(max(gain, ptrGain->GetMin()), ptrGain->GetMax());

    ptrExposureTime->SetValue(exposureTime);
    ptrGain->SetValue(gain);
    return 0;
}

// Function to acquire images and adjust exposure and gain based on brightness
int AcquireAndAdjustImages(CameraPtr pCam, double targetBrightness)
{
    int result = 0;

    cout << endl << "*** IMAGE ACQUISITION AND ADJUSTMENT ***" << endl << endl;

    try
    {
        pCam->BeginAcquisition();
        //
        // Turn off automatic exposure mode
        INodeMap& nodeMap = pCam->GetNodeMap();
        CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
        if (IsReadable(ptrExposureAuto) &&
            IsWritable(ptrExposureAuto))
        {
            CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
            if (IsReadable(ptrExposureAutoOff))
            {
                ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
                cout << "Automatic exposure disabled..." << endl;
            }
        }
        else 
        {
            CEnumerationPtr ptrAutoBright = nodeMap.GetNode("autoBrightnessMode");
            if (!IsReadable(ptrAutoBright) ||
                !IsWritable(ptrAutoBright))
            {
                cout << "Unable to get or set exposure time. Aborting..." << endl << endl;
                // return -1;
            }
            cout << "Unable to disable automatic exposure. Expected for some models... " << endl;
            cout << "Proceeding..." << endl;
            // result = 1;
        }

        // Turn off automatic gain mode
        CEnumerationPtr gainAuto = nodeMap.GetNode("GainAuto");
        if (IsReadable(gainAuto) &&
            IsWritable(gainAuto))
        {
            gainAuto->SetIntValue(gainAuto->GetEntryByName("Off")->GetValue());
            cout << "Automatic gain disabled..." << endl;
        }
        else 
        {
            cout << "Unable to disable automatic gain. Expected for some models... " << endl;
        }

        int imageCnt=0;
        // Create ImageProcessor instance for post processing images
        //
        ImageProcessor processor;

        //
        // Set default image processor color processing method
        //
        // *** NOTES ***
        // By default, if no specific color processing algorithm is set, the image
        // processor will default to NEAREST_NEIGHBOR method.
        //
        processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
        while (true)
        {   
            ImagePtr pResultImage = pCam->GetNextImage();

            if (pResultImage->IsIncomplete())
            {
                cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << endl;
            }
            else
            {   
                // save current image
                // Convert image to mono 8
                ImagePtr convertedImage = processor.Convert(pResultImage, PixelFormat_Mono8);

                // Create a unique filename
                ostringstream filename;
                // Define the new save path
                string newPath = "/home/ruan-x/rsworkSpace/src/acquisition/auto_adj/";

                filename << newPath << "Exposure_Gain_auto-";
                filename << imageCnt << ".jpg";
                imageCnt++;
                // Save image
                convertedImage->Save(filename.str().c_str());

                cout << "Image saved at " << filename.str() << endl;

                //change exposure and gain
                double currentBrightness = CalculateBrightness(pResultImage);
                int adjust_re = AdjustExposureGain(pCam, targetBrightness, currentBrightness);
                if ((adjust_re == 0))
                {
                    cout << "Current Brightness: " << currentBrightness << endl;
                }
                else if((adjust_re == 1))
                {
                    cout << "finished adjustment!" << endl;
                    break;
                }else{
                    cout << "Gain and Exposure not accessible. End project" << endl;
                    break;
                }
            }
            pResultImage->Release();
        }

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

    try
    {
        // Initialize camera
        pCam->Init();

        // Acquire images and adjust exposure and gain
        result = AcquireAndAdjustImages(pCam, targetBrightness);

        // Deinitialize camera
        pCam->DeInit();
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

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
