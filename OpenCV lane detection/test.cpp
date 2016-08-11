/*
 // face detection
 #include<stdio.h>
 #include<math.h>
 #include<opencv\cv.h>
 #include<opencv\highgui.h>
 #include<opencv2\objdetect\objdetect.hpp>
 #include<opencv2\highgui\highgui.hpp>
 #include<opencv2\imgproc\imgproc.hpp>
 #include<vector>

 using namespace cv;
 using namespace std;

 int main()
 {
 CascadeClassifier face_cascade, eye_cascade;
 string face = "C:\\Users\\YuYu\\Documents\\opencv\\install\\etc\\haarcascades\\haarcascade_frontalface_alt2.xml";
 string eye = "C:\\Users\\YuYu\\Documents\\opencv\\install\\etc\\haarcascades\\haarcascade_eye.xml";
 if(!face_cascade.load(face)) {
 printf("Error loading cascade file for face");
 system("pause");
 return 1;
 }
 if(!eye_cascade.load(eye)) {
 printf("Error loading cascade file for eye");
 system("pause");
 return 1;
 }
 VideoCapture capture(0); //-1, 0, 1 device id
 if(!capture.isOpened())
 {
 printf("error to initialize camera");
 system("pause");
 return 1;
 }
 Mat cap_img,gray_img;
 vector<Rect> faces, eyes;
 while(1)
 {
 capture >> cap_img;
 waitKey(10);
 cvtColor(cap_img, gray_img, CV_BGR2GRAY);
 cv::equalizeHist(gray_img,gray_img);
 face_cascade.detectMultiScale(gray_img, faces, 1.1, 10, CV_HAAR_SCALE_IMAGE | CV_HAAR_DO_CANNY_PRUNING, cvSize(0,0), cvSize(300,300));
 for(int i=0; i < faces.size();i++)
 {
 Point pt1(faces[i].x+faces[i].width, faces[i].y+faces[i].height);
 Point pt2(faces[i].x,faces[i].y);
 Mat faceROI = gray_img(faces[i]);
 eye_cascade.detectMultiScale(faceROI, eyes, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30,30));
 for(size_t j=0; j< eyes.size(); j++)
 {
 //Point center(faces[i].x+eyes[j].x+eyes[j].width*0.5, faces[i].y+eyes[j].y+eyes[j].height*0.5);
 Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
 int radius = cvRound((eyes[j].width+eyes[j].height)*0.25);
 circle(cap_img, center, radius, Scalar(255,0,0), 2, 8, 0);
 }
 rectangle(cap_img, pt1, pt2, cvScalar(0,255,0), 2, 8, 0);
 }
 imshow("Result", cap_img);
 waitKey(3);
 char c = waitKey(3);
 if(c == 27)
 break;
 }
 return 0;
 }
 */
/*
 // open image
 #include <opencv2/core/core.hpp>
 #include <opencv2/imgcodecs.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <iostream>
 #include <string>
 using namespace cv;
 using namespace std;
 int main( int argc, char** argv )
 {
 string imageName("C:\\Users\\YuYu\\workspace\\OpenCV\\MyPic.jpg"); // by default
 if( argc > 1)
 {
 imageName = argv[1];
 }
 Mat image;
 image = imread(imageName.c_str(), IMREAD_COLOR); // Read the file
 if( image.empty() )                      // Check for invalid input
 {
 cout <<  "Could not open or find the image" << std::endl ;
 return -1;
 }
 namedWindow( "Display window", WINDOW_AUTOSIZE ); // Create a window for display.
 imshow( "Display window", image );                // Show our image inside it.
 waitKey(0); // Wait for a keystroke in the window
 return 0;
 }
 */
/*
//Open image in gray
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;
int main(int argc, char** argv) {
	string imageName("C:\\Users\\YuYu\\workspace\\OpenCV\\MyPic.jpg"); // by default
	if (argc > 1) {
		imageName = argv[1];
	}
	Mat image;
	image = imread(imageName.c_str(), IMREAD_COLOR); // Read the file
	if (image.empty())                      // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}
	Mat gray_image;
	cvtColor(image, gray_image, COLOR_BGR2GRAY);
	imwrite("C:\\Users\\YuYu\\workspace\\OpenCV\\Gray_Image.jpg", gray_image);
	namedWindow(imageName, WINDOW_AUTOSIZE);
	namedWindow("Gray image", WINDOW_AUTOSIZE);
	imshow(imageName, image);
	imshow("Gray image", gray_image);
	waitKey(0);
	return 0;
}
*/#include "opencv2/highgui/highgui.hpp"
/*
//Opening a video
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    VideoCapture cap("C:\\Users\\YuYu\\Desktop\\dcim\\100CASIO\\CIMG0001.mov"); // open the video file for reading

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the video file" << endl;
         return -1;
    }

    //cap.set(CV_CAP_PROP_POS_MSEC, 300); //start the video at 300ms

    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video

     cout << "Frame per seconds : " << fps << endl;

    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

    while(1)
    {
        Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
                        cout << "Cannot read the frame from video file" << endl;
                       break;
        }

        imshow("MyVideo", frame); //show the frame in "MyVideo" window

        if(waitKey(30) == 27) //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
       {
                cout << "esc key is pressed by user" << endl;
                break;
       }
    }

    return 0;

}
*/

// Red light detection
/*
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    /* Opens Camera feed
	VideoCapture cap("C:\\Users\\YuYu\\Desktop\\RPI2 stuff\\vid2.mov"); //capture the video from webcam
    if(!cap.isOpened()){
        cout << "Camera failed to open" << endl;
        return -1;
    }

    /* Create a window called "Control"
     * Will be used to change HSV values

    namedWindow("Control", CV_WINDOW_AUTOSIZE);

    /* HSV Values on what is considered Red */
    /*
    int iLowH = 170;
    int iHighH = 179;

    int iLowS = 200;
    int iHighS = 255;

    int iLowV = 60;
    int iHighV = 255;

    //my values
	int iLowH = 0;
	int iHighH = 88;

	int iLowS = 0;
	int iHighS = 206;

	int iLowV = 221;
	int iHighV = 255;

    //Create trackbars in "Control" window
    //Move trackbar to make necessary adjustments to limit false positives

    createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &iHighH, 179);

    createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &iHighS, 255);

    createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
    createTrackbar("HighV", "Control", &iHighV, 255);


    while(true) {
        Mat frame;
        Mat HSV;
        Mat Threshold;
        Mat Threshold_output;

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        cap.read(frame);

        /* Convert frame from BGR to HSV
        cvtColor(frame, HSV, COLOR_BGR2HSV);

        /* Checks if object is red
        inRange(HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), Threshold);
        inRange(HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), Threshold_output);

        /* Filtering images
        erode(Threshold, Threshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        dilate(Threshold, Threshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

        dilate(Threshold, Threshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
        erode(Threshold, Threshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

        /* Separates red objects, in this case, traffic lights
        threshold(Threshold, Threshold, 100, 255, THRESH_BINARY_INV);

        /* Finds contours of red objects
        findContours(Threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

        /* Declaring contour variables
        vector<vector<Point> > contours_poly(contours.size());
        vector<Rect> boundRect(contours.size());
        vector<Point2f> center(contours.size());
        vector<float> radius(contours.size());

        /* Calculates upright bounding rectangle of a point set
        for(int i = 0; i < contours.size(); i++){
            approxPolyDP(contours[i], contours_poly[i], 3, true);
            boundRect[i] = boundingRect(Mat(contours_poly[i]));
        }

        Mat drawing = Mat::zeros(Threshold_output.size(), CV_8UC3);

        /* Draws a rectangle around the traffic light object
        for(int i = 0; i < contours.size(); i++){
            rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 100, 255));
        }

        /* Shows Threshold Feed
        imshow("Threshold Image", Threshold_output);

        frame = frame + drawing; // This makes the video feed include the contour drawings
        namedWindow("Camera_feed", CV_WINDOW_AUTOSIZE);
        imshow("Camera_feed", frame);

        if (waitKey(1) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }
    }

    return 0;
}

*/

#include "lane_detect.hpp"

int main(int argc, char** argv)
{
	lane_detect LD;
		LD.detect();
    return 0;
}

