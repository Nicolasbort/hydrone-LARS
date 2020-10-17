#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#define SIZE 7
#define RATIO_LIMIT 0.65f
#define THICK 4
#define OFFSET 1
#define OFFSET_LEFT 4
#define OFFSET_TEST 3
#define IMG_ROWS 28
#define IMG_COLS 28

using namespace std;
using namespace cv;


class Detect
{
    Mat imgMain;

    int detectedNumber;

    int positions[7][4] = {
        //     x             y                               dx                  dy
        {OFFSET_LEFT*2,        OFFSET,                 IMG_COLS-(OFFSET_LEFT*2),   THICK},                 // TOP
        {OFFSET_TEST,          OFFSET,                 OFFSET_TEST+THICK,          (IMG_ROWS+THICK)/2},    // TOP-LEFT
        {OFFSET_TEST,          (IMG_ROWS+THICK)/2,     OFFSET_TEST+THICK,          IMG_ROWS-OFFSET},       // BOT-LEFT
        {OFFSET_LEFT*2,        IMG_ROWS,               IMG_COLS-(OFFSET_TEST*2),   IMG_ROWS-OFFSET-THICK}, // BOT
        {IMG_ROWS-OFFSET_TEST, IMG_ROWS-OFFSET,        IMG_COLS-OFFSET_TEST-THICK, (IMG_ROWS+THICK)/2},    // BOT-RIGHT
        {OFFSET_LEFT*2,        IMG_ROWS/2,             IMG_COLS-(OFFSET_LEFT*2),   (IMG_ROWS/2)+THICK},    // CENTER
        {IMG_ROWS-OFFSET_TEST, OFFSET,                 IMG_COLS-OFFSET_TEST-THICK, (IMG_ROWS+THICK)/2}     // TOP-RIGHT
    };

    int numbers[10][7] = {
        {1, 1, 1, 1, 1, 0, 1}, // 0
        {0, 0, 0, 0, 1, 0, 1}, // 1
        {1, 0, 1, 1, 0, 1, 1}, // 2
        {1, 0, 0, 1, 1, 1, 1}, // 3
        {0, 1, 0, 0, 1, 1, 1}, // 4
        {1, 1, 0, 1, 1, 1, 0}, // 5
        {1, 1, 1, 1, 1, 1, 0}, // 6
        {1, 0, 0, 0, 1, 0, 1}, // 7
        {1, 1, 1, 1, 1, 1, 1}, // 8
        {1, 1, 0, 1, 1, 1, 1}  // 9
    };

    int isON[SIZE];

public:
    bool isNegative = false;

    void setImage(Mat);
    void processImage(bool);
    void resizeMain(int, int);
    void showMain();

    int getPredictedNumber();

    bool checkForOne();
    bool checkForNegative();
    bool predict(bool);
};