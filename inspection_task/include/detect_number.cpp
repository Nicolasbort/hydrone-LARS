#include "detect_number.hpp"


// VERIFICA SE DOIS VETORES SAO IGUAIS
bool areEqual(int v1[SIZE], int v2[SIZE])
{
    for (int i=0; i<SIZE; i++){
		if (v1[i] != v2[i])
			return false;
    }

    return true;
}


void Detect::processImage(bool canBeNegative)
{
    if (canBeNegative)
    {
        // Expande os segmentos do negativo
        morphologyEx(this->imgMain, this->imgMain, MORPH_ELLIPSE, Mat::ones(Size(11,11), CV_8U));
    }
    else
    {
        // Expande os segmentos do numero
        Mat structure = getStructuringElement(CV_SHAPE_CROSS, Size(9, 6));
        morphologyEx(this->imgMain, this->imgMain, MORPH_OPEN, structure); 
    }
    
}

void Detect::setImage(Mat img)
{
    this->imgMain = img.clone();
}

void Detect::resizeMain(int width, int height)
{
    cv::resize(this->imgMain, this->imgMain, Size(width, height));
}

bool Detect::checkForNegative()
{
    int LOCAL_THICK = 3;

    Point pt1(0,            (IMG_ROWS/2)-LOCAL_THICK);
    Point pt2(IMG_COLS/2,   (IMG_ROWS/2)+LOCAL_THICK);

    Mat tempimg = this->imgMain(Rect(pt1, pt2));

    threshold(tempimg, tempimg, 150, 255, THRESH_BINARY);

    int nonZero = countNonZero( tempimg );
    int totalPixels = tempimg.rows * tempimg.cols;
    float ratio = (nonZero / (float)totalPixels);

    this->isNegative = (ratio <= 0.65f);

    // DEBUG
    // cout << tempimg << "\n\n";
    // cout << "EH NEGATIVO: " << this->isNegative << "  RATIO NON ZEROS: " << ratio*100 << "%\n\n"; 

    return this->isNegative;
}

bool Detect::checkForOne()
{
    Point pt1_top(IMG_ROWS-OFFSET_TEST,         OFFSET);
    Point pt2_top(IMG_COLS-OFFSET_TEST-THICK,   (IMG_ROWS+THICK)/2);

    Point pt1_bot(IMG_ROWS-OFFSET_TEST,         IMG_ROWS-OFFSET);
    Point pt2_bot(IMG_COLS-OFFSET_TEST-THICK,   (IMG_ROWS+THICK)/2);


    Mat tempimg_top = this->imgMain(Rect(pt1_top, pt2_top));
    Mat tempimg_bot = this->imgMain(Rect(pt1_bot, pt2_bot));

    threshold(tempimg_top, tempimg_top, 150, 255, THRESH_BINARY);
    threshold(tempimg_bot, tempimg_bot, 150, 255, THRESH_BINARY);

    int nonZero_top = countNonZero( tempimg_top );
    int nonZero_bot = countNonZero( tempimg_bot );

    int totalPixels_top = tempimg_top.rows * tempimg_top.cols;
    int totalPixels_bot = tempimg_bot.rows * tempimg_bot.cols;

    float ratio_top = (nonZero_top / (float)totalPixels_top);
    float ratio_bot = (nonZero_bot / (float)totalPixels_bot);

    // DEBUG
    // cout << "TOP:\n" << tempimg_top << "\n\n";
    // printf("TOP:  NONZEROS: %i    TOTAL: %i    PERCENT: %.2f\n\n", nonZero_top, totalPixels_top, ratio_top);
    // cout << "BOT:\n" << tempimg_bot << "\n\n";
    // printf("BOT:  NONZEROS: %i    TOTAL: %i    PERCENT: %.2f\n\n", nonZero_bot, totalPixels_bot, ratio_bot);

    return (ratio_top <= RATIO_LIMIT && ratio_bot <= RATIO_LIMIT);
}

bool Detect::predict(bool canBeNegative = false)
{
    bool success = false;

    if (canBeNegative)
    {
        this->processImage(true);

        this->checkForNegative();

        if (this->checkForOne())
        {
            this->detectedNumber = 1;
            success = true;
        }
        else
        {
            success = false;
        }
    }
    else
    {
        this->processImage(false);

        for (int i=0; i<SIZE; i++)
        {
            Point pt1(this->positions[i][0], this->positions[i][1]);
            Point pt2(this->positions[i][2], this->positions[i][3]);

            Mat tempimg = this->imgMain(Rect(pt1, pt2));

            // cout << tempimg << "\n\n";

            threshold(tempimg, tempimg, 150, 255, THRESH_BINARY);

            int nonZero = countNonZero( tempimg );
            int totalPixels = tempimg.rows * tempimg.cols;
            float ratio = (nonZero / (float)totalPixels);

            if (ratio <= RATIO_LIMIT){
                this->isON[i] = 1;
            }else{
                this->isON[i] = 0;
            }

            // USADO PARA DEBUG
            cout << tempimg << "\n\n";
            printf("%i NONZEROS: %i    TOTAL: %i    PERCENT: %.2f\n\n", i, nonZero, totalPixels, ratio*100);

            rectangle(this->imgMain, Rect (pt1, pt2), int(255 * ratio * ratio), -1);
        }

        for (int i=0; i<10; i++)
        {
            if (areEqual(this->isON, this->numbers[i])){
                success = true;
                this->detectedNumber = i;
                printf("VALOR ENCONTRADO: %i\n", i);
                break;
            }
        }
    }

    return success;
}

int Detect::getPredictedNumber()
{
    return this->detectedNumber;
}

void Detect::showMain()
{
    imshow("imgMain", this->imgMain);
}