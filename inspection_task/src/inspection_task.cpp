#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include "detect_number.hpp"

using namespace cv;
using namespace std;

#define cvCOLOR_RED     Scalar(0, 0, 255)
#define cvCOLOR_GREEN   Scalar(0, 255, 0)
#define cvCOLOR_BLUE    Scalar(255, 0, 0)

// Parametros de filtros
#define GAUSSIANFILTER 3
#define KERNELSIZE 7

// Limiares da cor azul ( Imagem HSV )
#define MINBLUE         105
#define MAXBLUE         135

#define MINSATBLUE      50
#define MAXSATBLUE      220

#define MINVALBLUE      50
#define MAXVALBLUE      255


// Limiares da cor amarela ( Imagem HSV )
#define MINYELLOW       15
#define MAXYELLOW       35

#define MINSATYELLOW    50
#define MAXSATYELLOW    220

#define MINVALYELLOW    50
#define MAXVALYELLOW    255


#define MAX_IMGS_TO_PRINT 2

// DEFINES DA COMPETICAO
#define MIN_GAS_VALUE 45
#define MAX_GAS_VALUE 55

#define MIN_AJUSTE_ZERO -5
#define MAX_AJUSTE_ZERO 5

// ROS variable (1 Green 0 Red)
ros::Publisher led_green_red;


int ARR_MAXBLUE[3]   = {MAXBLUE, MAXSATBLUE, MAXVALBLUE};
int ARR_MINBLUE[3]   = {MINBLUE, MINSATBLUE, MINVALBLUE};

int ARR_MAXYELLOW[3] = {MAXYELLOW, MAXSATYELLOW, MAXVALYELLOW};
int ARR_MINYELLOW[3] = {MINYELLOW, MINSATYELLOW, MINVALYELLOW};

int ARR_MAX_C2[3] = {75, 70, 70};
int ARR_MIN_C2[3] = {0, 0, 0};

int ARR_MAXMARCADOR_HSV[3] = {45, 25, 80};
int ARR_MINMARCADOR_HSV[3] = {0, 0, 0};

// Vetores usados para guardar os numeros detectados
vector<long> vectorTopNumber(99, 0);
vector<long> vectorBotNumber(99, 0);

long vecTopNumber[99] = {0};
long vecBotNumber[99] = {0};

long countNegative = 0;

// KNN
Ptr<cv::ml::KNearest>  kNearest(cv::ml::KNearest::create());
const int MIN_CONTOUR_AREA = 100;
const int RESIZED_IMAGE_WIDTH = 20;
const int RESIZED_IMAGE_HEIGHT = 30;


// FUNCOES

// Razao da largura pela altura de um RotatedRect
float razao(RotatedRect rect)
{
    Point2f pts[4];

    rect.points(pts);

    return float( hypot( pts[1].x - pts[0].x, pts[1].y - pts[0].y ) / hypot( pts[2].x - pts[1].x, pts[2].y - pts[1].y ));
}


// Area de um RotatedRect
float rotated_area(RotatedRect rect)
{
  Point2f pts[4];
  float area;

  rect.points( pts );

  area = hypot( pts[1].x - pts[0].x, pts[1].y - pts[0].y ) * hypot( pts[2].x - pts[1].x, pts[2].y - pts[1].y );

  return area;
}


// Transforma RotatedRect para imagem
Mat rotateToImage(Mat img, RotatedRect rect)
{

	Mat rotated, cropped;
	float angle = rect.angle;
    Size rect_size = rect.size;

	Mat M = getRotationMatrix2D(rect.center, angle, 1.0);

    // Rotaciona a imagem
	warpAffine(img, rotated, M, img.size());

	// Isola o rotatedrect para dentro do cropped
	getRectSubPix(rotated, rect_size, rect.center, cropped);

    return cropped;
}


// Treino dos XMLs para o KNN
// bool train()
// {
//     Mat matClassificationInts;      // we will read the classification numbers into this variable as though it is a vector

//     FileStorage fsClassifications("classifications_gazebo.xml", FileStorage::READ);        // open the classifications file

//     if (fsClassifications.isOpened() == false) {                                                    // if the file was not opened successfully
//         ROS_INFO("ERRO, falha ao abrir o xml classificador";    // show error message
//         return false;                                                                                  // and exit program
//     }

//     fsClassifications["classifications"] >> matClassificationInts;      // read classifications section into Mat classifications variable
//     fsClassifications.release();                                        // close the classifications file

//                                                                         // read in training images ////////////////////////////////////////////////////////////

//     Mat matTrainingImagesAsFlattenedFloats;         // we will read multiple images into this single image variable as though it is a vector

//     FileStorage fsTrainingImages("images_gazebo.xml", FileStorage::READ);          // open the training images file

//     if (fsTrainingImages.isOpened() == false) {                                                 // if the file was not opened successfully
//         ROS_INFO("ERRO, falha ao abrir o xml imagens\n");         // show error message
//         return false;                                                                              // and exit program
//     }

//     fsTrainingImages["images"] >> matTrainingImagesAsFlattenedFloats;           // read images section into Mat training images variable
//     fsTrainingImages.release();                                                 // close the traning images file

//                                                                                 // finally we get to the call to train, note that both parameters have to be of type Mat (a single Mat)
//                                                                                 // even though in reality they are multiple images / numbers
//     kNearest->train(matTrainingImagesAsFlattenedFloats, cv::ml::ROW_SAMPLE, matClassificationInts); 

//     ROS_INFO("TREINO FINALIZADO");
// }


// Retorna a string encontrada na imagem (usada para deteccao da porcentagem)
string getPercent(Mat img)
{
    if (img.empty()) {
        ROS_INFO("ERRO: a imagem está vazia\n");
        return ""; 
    }

    Mat matThresh;   

    // // filter image from grayscale to black and white
    adaptiveThreshold(img, matThresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 11, 2);    

    resize(matThresh, matThresh, Size(RESIZED_IMAGE_WIDTH, RESIZED_IMAGE_HEIGHT));


    Mat matROIFloat;
    matThresh.convertTo(matROIFloat, CV_32FC1);  
    Mat matROIFlattenedFloat = matROIFloat.reshape(1, 1);
    Mat matCurrentChar(0, 0, CV_32F);

    kNearest->findNearest(matROIFlattenedFloat, 1, matCurrentChar);

    string strFinalString = "";
    float fltCurrentChar = (float)matCurrentChar.at<float>(0, 0);   
    strFinalString += char(int(fltCurrentChar));

    return strFinalString;
}


// Inverte o preto com branco
void invert_color(Mat img)
{
    bitwise_not(img, img);
}


// Retorna a string encontrada na imagem
string getKNNChar(Mat img, const char* name)
{
    Mat matTestingNumbers = img.clone();

    Rect rect_left( Point(10, 0), Point(img.cols * 0.52, img.rows) );
    Rect rect_right( Point(img.cols * 0.52, 0), Point(img.cols, img.rows) );

    if (matTestingNumbers.empty()) {
        cout << "ERRO: a imagem está vazia\n";
        return ""; 
    }

    Mat matThresh;   

    // filter image from grayscale to black and white
    adaptiveThreshold(matTestingNumbers, matThresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 11, 2);                                   // constant subtracted from the mean or weighted mean


    Mat LEFT_ROI = matThresh(rect_left);
    Mat RIGHT_ROI = matThresh(rect_right);

    resize(LEFT_ROI, LEFT_ROI, Size(RESIZED_IMAGE_WIDTH, RESIZED_IMAGE_HEIGHT));  
    resize(RIGHT_ROI, RIGHT_ROI, Size(RESIZED_IMAGE_WIDTH, RESIZED_IMAGE_HEIGHT));

    Mat LEFT_FLOAT, RIGHT_FLOAT;

    LEFT_ROI.convertTo(LEFT_FLOAT, CV_32FC1); 
    RIGHT_ROI.convertTo(RIGHT_FLOAT, CV_32FC1);      

    Mat LEFT_FLATTEN = LEFT_FLOAT.reshape(1, 1);
    Mat RIGHT_FLATTEN = RIGHT_FLOAT.reshape(1, 1);

    Mat LEFT_CHAR(0, 0, CV_32F);
    Mat RIGHT_CHAR(0, 0, CV_32F);

    kNearest->findNearest(LEFT_FLATTEN, 1, LEFT_CHAR); 
    kNearest->findNearest(RIGHT_FLATTEN, 1, RIGHT_CHAR);

    float FLOAT_LEFT_CHAR = (float)LEFT_CHAR.at<float>(0, 0);
    float FLOAT_RIGHT_CHAR = (float)RIGHT_CHAR.at<float>(0, 0);

    string strFinalString;
    strFinalString = char(int(FLOAT_LEFT_CHAR));
    strFinalString += char(int(FLOAT_RIGHT_CHAR));
 
    return strFinalString;
}


Mat leftTop, rightTop;
Mat leftBot, rightBot;
Mat imgPercentResized;

void generateDataset(Mat imgTop, Mat imgBot, Mat imgPercent)
{
    resize(imgTop, imgTop, Size(56, 28));
    resize(imgBot, imgBot, Size(56, 28));
    resize(imgPercent, imgPercentResized, Size(28, 28));

    int middle_top = floor(imgTop.cols / 2);
    int middle_bot = floor(imgBot.cols / 2);

    leftTop = imgTop( Rect( Point(0, 0), Point(middle_top, imgTop.rows) ) );
    rightTop = imgTop( Rect( Point(middle_top, 0), Point(imgTop.cols, imgTop.rows) ) );

    leftBot = imgBot( Rect( Point(0, 0), Point(middle_bot, imgTop.rows) ) );
    rightBot = imgBot( Rect( Point(middle_bot, 0), Point(imgTop.cols, imgTop.rows) ) );
}


void generate28x28(Mat imgTop, Mat imgBot)
{
    resize(imgTop, imgTop, Size(56, 28));
    resize(imgBot, imgBot, Size(56, 28));

    int middle_top = floor(imgTop.cols / 2);
    int middle_bot = floor(imgBot.cols / 2);

    leftTop = imgTop( Rect( Point(0, 0), Point(middle_top, imgTop.rows) ) );
    rightTop = imgTop( Rect( Point(middle_top, 0), Point(imgTop.cols, imgTop.rows) ) );

    leftBot = imgBot( Rect( Point(0, 0), Point(middle_bot, imgTop.rows) ) );
    rightBot = imgBot( Rect( Point(middle_bot, 0), Point(imgTop.cols, imgTop.rows) ) );
}


void generatePercent28x28(Mat img)
{
    resize(img, imgPercentResized, Size(28, 28));
}


Mat rotateImg(Mat img, float angle)
{
    Mat M = getRotationMatrix2D(Point2f( img.cols/2, img.rows/2 ), angle, 1.0);
    Mat imgRotated;
    warpAffine(img, imgRotated, M, img.size());

    return imgRotated;
}

// Retorna o index do maior valor do vector
int getIndexOfMaxElement(long vec[99])
{
    auto it_max = max_element(&vec[0], &vec[99]);

    return distance(&vec[0], it_max);
}

void cleanArray(long vec[99])
{
    fill(&vec[0], &vec[99], 0);
}

bool gasIsInRange(int value)
{
    return (value >= MIN_GAS_VALUE && value <= MAX_GAS_VALUE);
}

bool ajusteZeroIsInRange(int value)
{
    return (value >= MIN_AJUSTE_ZERO && value <= MAX_AJUSTE_ZERO);
}

// FIM FUNCOES



// CLASSES

// Classe para deteccao da porcentagem (IMAGEM 28X28)
class Percent
{
    Mat imgMain;

    vector<Point> points;

    float imgArea;

    
public:

    float ratioCenter, ratioTopBall, ratioBotBall, ratioNonZeroImg;

    // 4 POIS EXISTEM QUATRO ETAPAS QUE DEVEM SER VERIFICADAS
    int isON[4];

    Percent()
    {
        points.push_back(Point(2,25));  //point1
        points.push_back(Point(3,28));  //point2
        points.push_back(Point(28,3));  //point3
        points.push_back(Point(25,2));  //point4

        this->imgArea = float(IMG_COLS * IMG_ROWS);

        this->isON[0] = 0;  // CENTER
        this->isON[1] = 0;  // TOP BALL
        this->isON[2] = 0;  // BOT BALL
        this->isON[3] = 0;  // NON ZEROS IMG
    }


    void setImage(Mat img)
    {
        this->imgMain = img.clone();
    }


    void predictCenter()
    {
        Mat mask_zeros = Mat::zeros(this->imgMain.size(), CV_8U);
        Mat bitwiseFinal;

        fillConvexPoly(mask_zeros, this->points, 255);

        bitwise_and(this->imgMain, mask_zeros, bitwiseFinal);

        int nonZeros = countNonZero(bitwiseFinal);

        // SE O ratioCenter FOR <= 0.135 -> OK
        this->ratioCenter = nonZeros / this->imgArea;

        int nonZerosImgMain = countNonZero(this->imgMain);
        this->ratioNonZeroImg = nonZerosImgMain / this->imgArea;


        this->isON[3] = ( ((0.55 - this->ratioNonZeroImg) <= 0) && ((this->ratioNonZeroImg - 0.95) <= 0) );

        this->isON[0] = (( this->ratioCenter - 0.14 ) <= 0);
    }


    void predictTopBall()
    {
        Rect rectTopBall(Point(2, 3), Point(14, 11));

        Mat imgTopBall = this->imgMain(rectTopBall);

        threshold(imgTopBall, imgTopBall, 110, 255, THRESH_BINARY);


        int imgTopBallArea = imgTopBall.rows * imgTopBall.cols;

        int nonZeroTopBall = countNonZero(imgTopBall);

        // SE O ratioTopBall FOR <= 0.34 -> OK
        this->ratioTopBall = nonZeroTopBall / float(imgTopBallArea);


        this->isON[1] = (( this->ratioTopBall - 0.45 ) <= 0);
    }


    void predictBotBall()
    {
        Rect rectBotBall(Point(16, 20), Point(26, 28));

        Mat imgBotBall = this->imgMain(rectBotBall);

        threshold(imgBotBall, imgBotBall, 110, 255, THRESH_BINARY);

        int imgBotBallArea = imgBotBall.rows * imgBotBall.cols;

        int nonZeroBotBall = countNonZero(imgBotBall);

        // SE O ratioTopBall FOR <= 0.2 -> OK
        this->ratioBotBall = nonZeroBotBall / float(imgBotBallArea);

        // 1 se for menor que 0.2 ou 0 se nao
        this->isON[2] = (( this->ratioBotBall - 0.45 ) <= 0);
    }


    void predict()
    {
        this->predictCenter();
        this->predictTopBall();
        this->predictBotBall();
    }


    bool isPercent()
    {
        return this->isON[0] * this->isON[1] * this->isON[2] * this->isON[3];
    }

    void debug()
    {
        printf("ratioCenter: %.2f\nratioTopBall: %.2f\nratioBotBall: %.2f\nratioNonZeroImg: %.2f\n", this->ratioCenter, this->ratioTopBall, this->ratioBotBall, this->ratioNonZeroImg);
    }


    void resizeMain(int width, int height)
    {
        cv::resize(this->imgMain, this->imgMain, Size(width, height));
    }


    void showMain()
    {
        imshow("imgMain", this->imgMain);
    }

};

// Classe de detecção das bases
class LandingMark
{
public:

    //// Variaveis ////
    Mat image, main_imgHSV_C3, img_blue_C1, img_yellow_C1, img_final_C1;
    Mat img_lab_can1_C1, img_hsv_can3_C1;
    Mat morph_kernel;

    int rows, cols;
    int centerX, centerY;

    bool success, fstTime;

	Rect mark;
    RotatedRect markRotatedRect;

    vector< vector<Point>> contours;


    LandingMark()
    {
        morph_kernel = Mat::ones(KERNELSIZE, KERNELSIZE, CV_8U);

        success = false;
        fstTime = true;
    }


    void camParam(Mat img)
    {
        rows = img.rows;
        cols = img.cols;

        centerX = img.size().width/2;
        centerY = img.size().height/2;
    }


    void setImage(Mat img)
    {
        if (fstTime)
        {
            LandingMark::camParam(img);
            fstTime = false;
        }

        image = img;
    }


    Mat imfill(Mat img)
    {
        morphologyEx(img, img, MORPH_CLOSE, morph_kernel, Point(-1, -1), 3);

        findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        vector<vector<Point>> hull( contours.size() );

        for( size_t i = 0; i < contours.size(); i++ )
        {
            convexHull( contours[i], hull[i], true );
        }

        if (hull.size() == 1)
        {
            drawContours(img, hull, 0, 255, -1);
        }
        else if (hull.size() > 1)
        {
            float biggestArea = 0;
            vector<Point> biggestContour;

            for ( size_t i = 0; i < hull.size(); i++ )
            {
                float area = contourArea(hull[i]);

                if (area > biggestArea)
                {
                    biggestArea = area;
                    biggestContour = hull[i];
                }
            }
            vector<vector<Point>> bigContours;
            bigContours.push_back(biggestContour);
            drawContours(img, bigContours, 0, 255, -1);
        }

        return img;
    }


    Mat imlimiares(Mat hsv, int hsvMin[3], int hsvMax[3])
    {
        Mat hsvtresh;

        inRange(hsv, Scalar(hsvMin[0], hsvMin[1], hsvMin[2]), Scalar(hsvMax[0], hsvMax[1], hsvMax[2]), hsvtresh);

        hsvtresh = imfill(hsvtresh);

        return hsvtresh;
    }


    void processImage()
    {
        Mat img_aux;

        cvtColor(image, main_imgHSV_C3, COLOR_BGR2HSV);;

        Mat hsv, output;

        // Pega a area azul
        img_blue_C1 = imlimiares(main_imgHSV_C3, ARR_MINBLUE, ARR_MAXBLUE);
        bitwise_and(main_imgHSV_C3, main_imgHSV_C3, hsv, img_blue_C1);

        // Pega a area amarela
        img_yellow_C1 = imlimiares(main_imgHSV_C3, ARR_MINYELLOW, ARR_MAXYELLOW);
        bitwise_and(hsv, hsv, output, img_yellow_C1);

        // Pega apenas a area do mark
        bitwise_and(img_blue_C1, img_yellow_C1, img_final_C1);
    }


    bool foundMarkRect()
    {
        Rect currentRect, biggestRect;
        bool succ = false;
        int biggestArea = 0;

        this->processImage();

        vector<vector<Point>> cont;

        findContours(this->img_final_C1, cont, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); 

        for (int i=0; i<cont.size(); i++)
        {
            currentRect = boundingRect(cont[i]);

            int area = currentRect.area();

            if (area > biggestArea)
            {
                biggestRect = currentRect;
                biggestArea = area;

                succ = true;
            }
        }

        if (succ)
            this->mark = biggestRect;

        return succ;
    }


    bool foundMark()
    {
        this->processImage();

        findContours(this->img_final_C1, this->contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        bool succ = false;

        RotatedRect rotRect;


        for (int i=0; i<this->contours.size(); i++)
        {
            rotRect = minAreaRect( this->contours[i] );

            if (rotRect.size.width > 0 && rotRect.size.height > 0)
            {
                if (int(rotRect.angle) % 90 != 0)
                {
                    rotRect.angle += abs(rotRect.angle - 90);
                }

                this->markRotatedRect = rotRect;
                succ = true;
            }else{
                succ = false;
            }
        }

        return succ;
    }


    void drawRotated()
    {
        Point2f vertices2f[4];
        markRotatedRect.points(vertices2f);

        Point vertices[4];    
        for(int i = 0; i < 4; ++i){
            vertices[i] = vertices2f[i];
        }

        fillConvexPoly(image, vertices, 4, cvCOLOR_RED);
    }


    Mat rotatedToImage()
    {
        Mat rotated;
        float angle = markRotatedRect.angle;

        if (markRotatedRect.angle < -45.) {
            angle += 90.0;
        }

        Mat M = getRotationMatrix2D(markRotatedRect.center, angle, 1.0);

        warpAffine(image, rotated, M, image.size());

        return rotated;
    }

    void debug()
    {
        this->processImage();

        imshow("YELLOW", this->img_yellow_C1);
        imshow("BLUE", this->img_blue_C1);
        imshow("BITWISE", this->img_final_C1);
    }


    void show()
    {
        imshow("main", image);
    }
};

// Classe para melhor controle das areas de interesse
class ROI
{
public: 

    Mat image;
    Mat editable_image, clean_img;
    Mat sensor;
    vector<Rect> numbers;
    RotatedRect biggest_rect;
    Rect rectSensor;

    ROI()
    {
        kernel = Mat::ones(KERNELSIZE, KERNELSIZE, CV_8U);
    }


    ROI(Mat img)
    {
        image = img.clone();

        editable_image = image.clone();

        kernel = Mat::ones(KERNELSIZE, KERNELSIZE, CV_8U);
    }


    ROI(Mat img, Rect rect)
    {
        kernel = Mat::ones(KERNELSIZE, KERNELSIZE, CV_8U);

        image = img(rect);

        editable_image = image.clone();
    }


    ROI(Mat img, RotatedRect rectROI)
    {
        this->kernel = Mat::ones(KERNELSIZE, KERNELSIZE, CV_8U);

        Mat M, rotated, cropped;
        float angle = rectROI.angle;
        Size size = rectROI.size;

        if (rectROI.angle < -45.) {
            angle += 90.0;
            swap(size.width, size.height);
        }

        M = getRotationMatrix2D(rectROI.center, angle, 1.0);

        warpAffine(img, rotated, M, img.size(), INTER_CUBIC);

        getRectSubPix(rotated, size, rectROI.center, cropped);

        this->image = cropped.clone();

        this->editable_image = image.clone();
    }


    void set(Mat img, Rect rect)
    {
        this->image = img(rect);
    }


    void set(Mat img, RotatedRect rectROI)
    {
        this->kernel = Mat::ones(KERNELSIZE, KERNELSIZE, CV_8U);

        Mat M, rotated, cropped;
        float angle = rectROI.angle;
        Size size = rectROI.size;

        if (rectROI.angle < -45.) {
            angle += 90.0;
            swap(size.width, size.height);
        }

        M = getRotationMatrix2D(rectROI.center, angle, 1.0);

        warpAffine(img, rotated, M, img.size(), INTER_CUBIC);

        getRectSubPix(rotated, size, rectROI.center, cropped);

        this->image = cropped.clone();

        this->editable_image = image.clone();
    }


    bool foundRect()
    {
        Rect current, biggestRect;
        int biggestArea = 0;
        bool found = false;

        Mat imageGrayscale;

        inRange(this->image, Scalar(ARR_MIN_C2[0], ARR_MIN_C2[1], ARR_MIN_C2[2]), Scalar(ARR_MAX_C2[0], ARR_MAX_C2[1], ARR_MAX_C2[2]), imageGrayscale);

        vector<vector<Point>> contours;
        findContours(imageGrayscale, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (contours.size() >= 1)
        {
            for (int i=0; i<contours.size(); i++)
            {
                current = boundingRect(contours[i]);

                int area = current.area();

                if (area > biggestArea)
                {
                    biggestArea = area;
                    biggestRect = current;

                    found = true;
                }
            }
        }

        if (found){
            this->sensor = this->image(biggestRect);
            cv::resize(this->sensor, this->sensor, Size(400, 400));
        }

        return found;
    }


    bool found(Mat img)
    {
        RotatedRect current;
        float biggest_area = 0, current_area;
        bool found = false;

        vector<vector<Point>> contours;
        findContours(img, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

        if (contours.size() >= 1)
        {
            for (int i=0; i<contours.size(); i++)
            {
                current = minAreaRect( contours[i] );
            
                current_area = rotated_area(current);

                if ( current_area < 25 )
                    continue;

                if (current_area > biggest_area)
                {
                    this->biggest_rect = current;
                    biggest_area = current_area;
                } 
            }
        }

        if (biggest_area == 0)
        {
            return false;
        }
        else if (razao(biggest_rect) <= 1.5)
        {
            return true;
        }
        else
        {
            return false;
        }
    }


    void rotatedToImage(Mat img)
    {

        Mat rotated, cropped;
        float angle = this->biggest_rect.angle;
        Size rect_size = this->biggest_rect.size;

        Mat M = getRotationMatrix2D(biggest_rect.center, angle, 1.0);

        // Rotaciona a imagem
        warpAffine(img, rotated, M, img.size());

        // Isola o rotatedrect para dentro do cropped
        getRectSubPix(rotated, rect_size, biggest_rect.center, cropped);

        this->image = cropped.clone();

        this->editable_image = this->image.clone();
    }


    void getRectNumbersStatic(Mat img)
    {
        int TOP_LEFT_CORNER_X = img.cols * 0.1;
        int TOP_LEFT_CORNER_Y_1 = img.rows * 0.03;
        int TOP_LEFT_CORNER_Y_2 = img.rows * 0.53;

        int BOT_RIGHT_CORNER_X = img.cols * 0.65;
        int BOT_RIGHT_CORNER_Y_1 = img.rows * 0.46;
        int BOT_RIGHT_CORNER_Y_2 = img.rows * 0.95;

        // Rect da porcentagem
        int P_TOP_LEFT_CORNER_X = img.cols * 0.64;
        int P_TOP_LEFT_CORNER_Y = img.rows * 0.015;

        int P_BOT_RIGHT_CORNER_X = img.cols * 0.95;
        int P_BOT_RIGHT_CORNER_Y = img.rows * 0.45;

        Rect PERCENTE( Point(P_TOP_LEFT_CORNER_X, P_TOP_LEFT_CORNER_Y), Point(P_BOT_RIGHT_CORNER_X, P_BOT_RIGHT_CORNER_Y) );
        Rect TOP( Point(TOP_LEFT_CORNER_X, TOP_LEFT_CORNER_Y_1), Point(BOT_RIGHT_CORNER_X, BOT_RIGHT_CORNER_Y_1) );
        Rect BOT( Point(TOP_LEFT_CORNER_X, TOP_LEFT_CORNER_Y_2), Point(BOT_RIGHT_CORNER_X, BOT_RIGHT_CORNER_Y_2) );

        numbers.push_back(TOP);
        numbers.push_back(BOT);
        numbers.push_back(PERCENTE);

        // rectangle(img, TOP, 140, 2);
        // rectangle(img, BOT, cvCOLOR_RED, 2); 
        // rectangle(img, PERCENTE, Scalar(0, 255, 0), 2);
    }


    void improve_image()
    {
        detailEnhance(this->image, this->image);
    }


    void drawRotated(Mat img)
    {
        Point2f pts[4];

        biggest_rect.points(pts);

        vector<Point> ptss(4);

        for (int i=0;i<4; i++)
        {
            ptss[i] = pts[i];
        }

        fillConvexPoly(img, ptss, Scalar(255, 0, 0));
    }


    void resize(int width, int height)
    {
        cv::resize(this->image, this->image, Size(width, height));

        this->editable_image = this->image.clone();
    }

    void show(const char* title)
    {
        imshow(title, this->image);
    }


private:

    vector<vector<Point>> contours;
    Mat kernel;
};

// Classe para o KNN
class ContourWithData 
{
public:
    vector<Point> ptContour;
    Rect boundingRect;
    float fltArea;


    bool checkIfContourIsValid() {
        if (fltArea < MIN_CONTOUR_AREA) return false;
        return true;
    }

    static bool sortByBoundingRectXPosition(const ContourWithData& cwdLeft, const ContourWithData& cwdRight) { 
        return(cwdLeft.boundingRect.x < cwdRight.boundingRect.x);
    }

};


///////////////////////////////
/////  VARIAVEIS GLOBAIS  /////
///////////////////////////////

LandingMark main_image;
ROI base, marcador;
Mat frame;
Detect detect;
Percent detectPercent;

bool flag = false, rotateImage = true;
bool fstTimeFlag = true;

int SUM_ANGLE = 0;
int contImWrite = 0;
int count_non_percent = 0, count_percent = 0;


string percent;


// Recebe informacao do nodo inpection_task e verifica se inica a deteccao ou nao
void detection_activate(const std_msgs::Bool& data)
{
    if (data.data){
        flag = true;
    }else{
        flag = false;
    }
}


// VARIAVEIS PARA PRINTAR OS NUMEROS NAO ENCONTRADOS
int contImWriteTopLeft = 0, contImWriteTopRight = 0;
int contImWriteBotLeft = 0, contImWriteBotRight = 0;

bool startedFlag = true;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
	{
        if (flag)
        {
            startedFlag = true;

            Mat base_grayscale;
            Mat marcador_grayscale, marcador_inRange;
            Mat imgPercent;
            Mat imgNumberTop, imgNumberBot;

            Mat sensorGrayscale;
            Mat sensorHSV;

            main_image.setImage(cv_bridge::toCvShare(msg, "bgr8")->image);


            //////////  BUSCA O MARCADOR APENAS QUANDO ENCONTRA UMA BASE /////////
            //////////    MARCADOR ESTÁ SEMPRE EM CIMA DE UMA BASE       /////////

            if (main_image.foundMarkRect())
            {
                base.set(main_image.image, main_image.mark);

                if (base.foundRect())
                {

                    cvtColor(base.sensor, sensorHSV, COLOR_BGR2HSV);
                    inRange(sensorHSV, Scalar(ARR_MINMARCADOR_HSV[0], ARR_MINMARCADOR_HSV[1], ARR_MINMARCADOR_HSV[2]), Scalar(ARR_MAXMARCADOR_HSV[0], ARR_MAXMARCADOR_HSV[1], ARR_MAXMARCADOR_HSV[2]), sensorGrayscale);
                    
                    sensorGrayscale = rotateImg(sensorGrayscale, SUM_ANGLE);

                    // Faz um crop dos numeros e armazena da variavel base.numbers (vector)
                    base.getRectNumbersStatic(sensorGrayscale);



                    ////////   PORCENTAGEM DO MARCADOR EH A BASE DO CODIGO   /////////
                    //  DETECCAO SO OCORRE SE A IMAGEM ESTIVER NA ORIENTACAO CERTA  //


                    imgPercent = sensorGrayscale( base.numbers[2] );

                    // Armazena o 28x28 na variavel imgPercentResized
                    generatePercent28x28(imgPercent);

                    detectPercent.setImage(imgPercentResized);
                    detectPercent.predict();


                    // Se for %
                    if (detectPercent.isPercent())
                    {
                        count_percent++;

                        // Subtrai 1 se foi maior que zero
                        count_non_percent -= (count_non_percent > 0);

                        imgNumberTop = sensorGrayscale( base.numbers[0] );
                        imgNumberBot = sensorGrayscale( base.numbers[1] );

                        // Armazena as 4 imagens nas variaveis (leftTop, rightTop, leftBot, rightBot)
                        generate28x28(imgNumberTop, imgNumberBot);

                        // TOP NUMBER
                        string topNumber = "";
                        detect.setImage(leftTop);
                        if (detect.predict(false)){
                            topNumber += to_string(detect.getPredictedNumber());
                        }

                        detect.setImage(rightTop);
                        if (detect.predict(false)){
                            topNumber += to_string(detect.getPredictedNumber());
                        }

                        if (!topNumber.empty())
                            vecTopNumber[ stoi(topNumber) ]++;
                        
                        putText(sensorGrayscale, "TOP: " + topNumber, Point(20, 50), FONT_HERSHEY_DUPLEX, 1.5, 140, 2);



                        // BOT NUMBER
                        string botNumber = "";
                        detect.setImage(leftBot);
                        if (detect.predict(true)){
                            botNumber += to_string(detect.getPredictedNumber());
                        }


                        if (detect.isNegative){
                            countNegative++;
                        }else{
                            countNegative--;
                        }
                        

                        detect.setImage(rightBot);
                        if (detect.predict(false)){
                            botNumber += to_string(detect.getPredictedNumber());
                        }

                        if (!botNumber.empty())
                            vecBotNumber[ stoi(botNumber) ]++;

                        if (detect.isNegative){
                            putText(sensorGrayscale, "BOT: -" + botNumber, Point(20, 100), FONT_HERSHEY_DUPLEX, 1.5, 160, 2);
                        }else{
                            putText(sensorGrayscale, "BOT: " + botNumber, Point(20, 100), FONT_HERSHEY_DUPLEX, 1.5, 160, 2);
                        }

                    }
                    else
                    {
                        count_non_percent++;

                        // Subtrai 1 se foi maior que zero
                        count_percent -= (count_percent > 0);

                        destroyWindow("imgNumberTop");
                        destroyWindow("imgNumberBot");
                    }


                    // Rotaciona a imagem se não encontrar o % quatro vezes seguidas
                    if (count_non_percent >= 4){
                        SUM_ANGLE += 90.0;
                        count_non_percent--;
                        if (SUM_ANGLE == 360)
                            SUM_ANGLE = 0;
                    }
                    

                    //////// GERAR DATASET 28X28 /////////

                        // imgNumberTop = sensorGrayscale( base.numbers[0] );
                        // imgNumberBot = sensorGrayscale( base.numbers[1] );
                        // imgPercent   = sensorGrayscale( base.numbers[2] );

                        // generateDataset(imgNumberTop, imgNumberBot, imgPercent);

                        // imshow("TOP", imgNumberTop);
                        // imshow("BOT", imgNumberBot);
                        // imshow("PERCENT", imgPercent);
                    
                    ///////  FIM GERAR DATASET  /////////


                    imshow("MARCADOR", sensorGrayscale);
                }
            }

            // Imagem vinda do UAV
            // main_image.show();

            int key = waitKey(20);

            if (key == 32){
                imwrite("/home/nicolas/workspace_images/leftTop_"+to_string(contImWrite)+".jpeg", leftTop);
                imwrite("/home/nicolas/workspace_images/rightTop_"+to_string(contImWrite)+".jpeg", rightTop);
                imwrite("/home/nicolas/workspace_images/leftBot_"+to_string(contImWrite)+".jpeg", leftBot);
                imwrite("/home/nicolas/workspace_images/rightBot_"+to_string(contImWrite)+".jpeg", rightBot);
                imwrite("/home/nicolas/workspace_images/percent_"+to_string(contImWrite)+".jpeg", imgPercentResized);
                imwrite("/home/nicolas/workspace_images/sensor_"+to_string(contImWrite)+".jpeg", sensorGrayscale);
                contImWrite++;
            }else if (key == 27)
            {
                SUM_ANGLE += 90.0;
            }
        }
        else
        {
            if (startedFlag)
            {
                int TopValue = getIndexOfMaxElement(vecTopNumber);
                int BotValue = getIndexOfMaxElement(vecBotNumber);
                
                if (countNegative >= 0){
                    BotValue = stoi("-" + to_string(BotValue));
                }

                ROS_INFO("TOP VALUE: %i   BOT VALUE: %i", TopValue, BotValue);

                std_msgs::Bool isRight;

                if (gasIsInRange(TopValue)){
                    ROS_INFO("Valor de gás no range correto!");
                    isRight.data = true;            // LIGA O LED GREEN ( A FAZER )
                    led_green_red.publish(isRight);
                }else{
                    ROS_INFO("Valor de gás no range incorreto!");
                    isRight.data = false;           // LIGA O LED RED ( A FAZER )
                    led_green_red.publish(isRight);
                }

                // RESETA OS VALORES PARA O PROXIMO MARCADOR
                cleanArray(vecTopNumber);
                cleanArray(vecBotNumber);
                countNegative = 0;

                // FALTA RESETAR OS VECTORS E ZERAR A VARIAVEL DE NEGATIVO

                destroyAllWindows();

                startedFlag = false;
                fstTimeFlag = true;
            }
        }
        


    }
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}


int main(int argc, char **argv)
{
 	ros::init(argc, argv, "image_listener");

	ros::NodeHandle n;
    ros::Subscriber activate = n.subscribe("/gas_detector/activate", 10, detection_activate);

	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/hydrone/camera_camera/image_raw", 1, imageCallback);
	
    led_green_red = n.advertise<std_msgs::Bool>("/gas_detector/led_green_red", 1);


    ros::spin();
		
	destroyWindow("view");
}