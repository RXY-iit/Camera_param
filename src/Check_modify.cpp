#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Function to calculate the histogram
void calcHistogram(const Mat& image, Mat& hist) {
    int histSize = 256;    // Number of bins
    float range[] = { 0, 256 }; // Range of intensity values
    const float* histRange = { range };

    // Calculate the histogram
    calcHist(&image, 1, 0, Mat(), hist, 1, &histSize, &histRange);
}

// Function to draw and display the histogram with coordinate annotations
Mat drawHistogram(const Mat& hist) {
    // int histHeight = 400;
    // int histWidth = 512;
    int histHeight = 300;
    int histWidth = 312;
    Mat histImage(histHeight + 50, histWidth + 50, CV_8UC1, Scalar(255)); // Increased size for annotations
    normalize(hist, hist, 0, histHeight, NORM_MINMAX);

    int histSize = hist.rows;
    int binWidth = cvRound((double)histWidth / histSize);
    for (int i = 1; i < histSize; i++) {
        line(histImage,
             Point(binWidth * (i - 1) + 25, histHeight - cvRound(hist.at<float>(i - 1)) + 25),
             Point(binWidth * i + 25, histHeight - cvRound(hist.at<float>(i)) + 25),
             Scalar(0), 2);
    }

    // Draw x-axis and y-axis
    line(histImage, Point(25, 25), Point(25, histHeight + 25), Scalar(0), 2);
    line(histImage, Point(25, histHeight + 25), Point(histWidth + 25, histHeight + 25), Scalar(0), 2);

    // Annotate x-axis
    for (int i = 0; i <= 255; i += 51) { // 5 ticks
        line(histImage, Point(binWidth * i + 25, histHeight + 25), Point(binWidth * i + 25, histHeight + 30), Scalar(0), 2);
        putText(histImage, to_string(i), Point(binWidth * i + 20, histHeight + 45), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0), 1);
    }

    // Annotate y-axis
    for (int i = 0; i <= histHeight; i += histHeight / 4) { // 5 ticks
        line(histImage, Point(20, histHeight - i + 25), Point(25, histHeight - i + 25), Scalar(0), 2);
        putText(histImage, to_string(i), Point(0, histHeight - i + 30), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0), 1);
    }

    return histImage;
}

// Function to check for over-exposure and modify the image if needed
tuple<bool, float> isOverExposed(const Mat& hist, int threshold = 255, double fraction = 0.01) {
    float totalPixels = sum(hist)[0];
    float overExposedPixels = hist.at<float>(threshold);

    // Calculate the ratio of over-exposed pixels to total pixels
    float ratio = overExposedPixels / totalPixels;

    // Check if the fraction of over-exposed pixels is greater than the given fraction
    return make_tuple(ratio > fraction, ratio);
}

// Function to calculate average brightness
double calcAverageBrightness(const Mat& image) {
    return mean(image)[0];
}

int main(int argc, char** argv) {
    // Directory containing images
    // string folderPath = "/home/ruan-x/rsworkSpace/src/acquisition/output";
    string folderPath = "/home/ruan-x/Videos";
    vector<String> imagePaths;
    glob(folderPath + "/*.jpg", imagePaths, false);
    // glob(folderPath + "/*.png", imagePaths, false);
    // glob(folderPath + "/*.pgm", imagePaths, false); 

    size_t currentIndex = 0;
    while (currentIndex < imagePaths.size()) {
        // Load the current image
        Mat image = imread(imagePaths[currentIndex], IMREAD_GRAYSCALE);

        if (image.empty()) {
            cout << "Could not open or find the image: " << imagePaths[currentIndex] << endl;
            currentIndex++;
            continue;
        }

        // Calculate the histogram
        Mat hist;
        calcHistogram(image, hist);

        // Check for over-exposure
        bool overExposed;
        float overExposedRatio;
        tie(overExposed, overExposedRatio) = isOverExposed(hist);
        double avgBrightness = calcAverageBrightness(image);

        // Draw the histogram
        Mat histImage = drawHistogram(hist);

        // Create the result image
        int resultWidth = image.cols + histImage.cols;
        int resultHeight = max(image.rows, histImage.rows) + 100;
        Mat resultImage(resultHeight, resultWidth, CV_8UC1, Scalar(255));
        image.copyTo(resultImage(Rect(0, 0, image.cols, image.rows)));
        histImage.copyTo(resultImage(Rect(image.cols, 0, histImage.cols, histImage.rows)));

        // Add brightness and over-exposure information
        string brightnessText = "Avg Brightness: " + to_string(avgBrightness);
        putText(resultImage, brightnessText, Point(image.cols + 10, histImage.rows + 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0), 2);
        string exposureText = "Overexposed: " + string(overExposed ? "Yes" : "No");
        putText(resultImage, exposureText, Point(image.cols + 10, histImage.rows + 70), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0), 2);
        string ratioText = "Ratio: " + to_string(overExposedRatio);
        putText(resultImage, ratioText, Point(image.cols + 10, histImage.rows + 110), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0), 2);

        // Display the result
        namedWindow("Analysis Result", WINDOW_NORMAL);
        imshow("Analysis Result", resultImage);

        char key = waitKey(0);
        if (key == ' ') {
            currentIndex++;
        } else if (key == 's') {
            string outputPath = "/home/ruan-x/rsworkSpace/src/acquisition/analysis/test_" + to_string(currentIndex) + ".png";
            // string outputPath = "/home/ruan-x/Pictures/Factor_analysis/result_" + to_string(currentIndex) + ".png";
            imwrite(outputPath, resultImage);
            cout << "Result saved to " << outputPath << endl;
        }
    }

    return 0;
}
