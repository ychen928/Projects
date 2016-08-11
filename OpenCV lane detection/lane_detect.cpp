#include "lane_detect.hpp"

// Choose between HoughP or Hough.
#define houghp 0
#define stdhough 1
#define hsv 1
#define binarize 1

lane_detect::lane_detect() {
	xSigma = 3;
	ySigma = 3;

	upperThresh = 100;
	lowerThresh = 33;

	minLineLength = 2;
	maxLineGap = 2;
}

/*
* Use this if you want to input your own values;
*/
lane_detect::lane_detect(int xSigmai, int ySigmai, int upperThreshi, int lowerThreshi, int minLineLengthi, int maxLineGapi) {
	xSigma = xSigmai;
	ySigma = ySigmai;

	upperThresh = upperThreshi;
	lowerThresh = lowerThreshi;

	minLineLength = minLineLengthi;
	maxLineGap = maxLineGapi;
}

/*
*	Apply gaussian blur to smooth out edges, gets rid of unwanted noise.
*	cv::GaussianBlur(image, image, kSize, sigmaY, sigmaX);
*	kSize must be positive and odd
*	Typical kSize is 5x5 matrix, and sigmaY and X values of 1
*/
cv::Mat lane_detect::gaussian_blur(cv::Mat image) {
	//cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

	cv::Size kSize = cv::Size(5, 5);
	cv::GaussianBlur(image, image, kSize, xSigma, ySigma);

	//180 200
	/*
	// using these with hsv makes the line faint
	cv::threshold(image, image, 175, 185, 0);
	int erodesize = 3;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erodesize, erodesize), cv::Point(-1, -1));
	cv::erode(image, image, element);
	*/
	return image;
}

/*
*	Sobel Edge Detection
*/
cv::Mat lane_detect::sobel_edge(cv::Mat gaussian_image) {
	int scale = 1;
	int delta = 0;
	int ddepth = CV_8U;

	cv::Mat sobel;
	gaussian_image.copyTo(sobel);

	cv::Sobel(gaussian_image, sobel, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
	//cv::cvtColor(sobel, sobel, cv::COLOR_BGR2GRAY);

	return sobel;
}


/*
*	Use canny edge algorithm to get edges
*	cv::Canny(image, image, upperThresh, lowerThresh);
*	upperThresh > lowerThresh
*	lowerThresh may be around 1/3 upperThresh. Depends on use case
*/
cv::Mat lane_detect::canny_edge(cv::Mat gaussian_image) {

	cv::Mat canny;
	//cv::cvtColor(gaussian_image, canny, cv::COLOR_BGR2GRAY);
	gaussian_image.copyTo(canny);
	cv::Canny(gaussian_image, canny, upperThresh, lowerThresh);
	return canny;
}

/*
*	Probabilistic hough transform
*	Use this instead of standard hough transform because we can choose to filter out min/max lengths
*	cv::HoughLinesP(canny_image, lines, rho, theta, threshold, minLineLength, minLineLength);
*/
cv::Mat lane_detect::hough_transform(cv::Mat canny_image, cv::Mat original) {
	cv::Mat hough = original.clone();


	double rho = 1.5;
	double theta = CV_PI/180;
	int threshold = 60;
	// inverse transformation matrix value that I got from using ipm.getHinv() from the points orig & dst points
	//cv::Mat trans orm = (cv::Mat_<float>(3, 3) << -0.02255122438780657, 0.0136674087198828, -16.0628019323675,
	//	0.004747626186906658, -0.00187027698272079, -16.0628019323675,
	//	2.373813093453329e-005, 4.244090076174131e-005, -0.08031400966183749);

#if houghp
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(canny_image, lines, rho, theta, threshold, minLineLength, maxLineGap);
#endif

#if stdhough
	std::vector<cv::Vec2f> lines;
	cv::HoughLines(canny_image, lines, rho, theta, threshold);

#endif

	/*	line(hough, cv::Point(x, y), cv::Point(x, y), cv::Scalar(a, b, c), 3, lineType)
	*	x, y = 2d array. ex: x = [int][int]
	*	a, b, c = RGB values 0-255
	*	lineType = type of line that is to be used. See http://docs.opencv.org/3.1.0/d0/de1/group__core.html#gaf076ef45de481ac96e0ab3dc2c29a777
	*/
	cv::Point pt1, pt2, pt3, pt4;
	cv::Scalar line_color = cv::Scalar(0, 0, 255);
	int line_thickness = 2;

	float dist = 0;
	float distL = 0;
	float distR = 0;
	float distOrigL = 0;
	float distOrigR = 0;
	float center = 0;

	static float midX = 0;
	static float midY = 0;

	int pt1x = 0, pt1y = 0, pt2x = 0, pt2y = 0, pt3x = 0, pt3y = 0, pt4x = 0, pt4y = 0;
	int pt12Counter = 0, pt34Counter = 0;

	static double timeElapse = 0; // keep value every time it access this function

	//std::cout << "time elapse original = " << timeElapse << std::endl;


	double t = (double)cv::getCPUTickCount();
	static bool switchingR = false;
	static bool switchingL = false;

	for (size_t i = 0; i < lines.size(); i++)
	{
#if houghp
		pt1 = cv::Point(lines[i][0],
			lines[i][1]);
		pt2 = cv::Point(lines[i][2],
			lines[i][3]);
		line(hough, pt1, pt2, line_color, line_thickness, cv::LINE_AA);
#endif


		// The code below is for standard Hough Line Transform
#if stdhough
		// http://stackoverflow.com/questions/31147438/how-to-undo-a-perspective-transform-for-a-single-point-in-opencv/31201448#31201448
		// transform single points,  sadly, does not work with houghp :( , apply transformation to vector points
		//perspectiveTransform(lines, lines, transform);
		float rho2 = lines[i][0];
		float theta2 = lines[i][1];
		double a = cos(theta2), b = sin(theta2);

		// Angles between 15 and 60 degrees in radians
		if (theta2 > 0.26 && theta2 < 1) { // taking average
			double x0 = a*rho2, y0 = b*rho2;
			pt1x += cvRound(x0 + 100 * (-b));
			pt1y += cvRound(y0 + 100 * (a));
			pt2x += cvRound(x0 - 100 * (-b));
			pt2y += cvRound(y0 - 100 * (a));
			pt12Counter++;
		}

		// Angles between 115 and 145 degrees in radians
		if (theta2 < 2.5 && theta2 > 2) { // taking average
			double x0 = a*rho2, y0 = b*rho2;
			pt3x += cvRound(x0 - 400 * (-b));
			pt3y += cvRound(y0 - 400 * (a));
			pt4x += cvRound(x0 - 650 * (-b));
			pt4y += cvRound(y0 - 650 * (a));
			pt34Counter++;
		}
#endif
	}
	if (pt12Counter != 0) {
		pt1.x = pt1x / pt12Counter;
		pt1.y = pt1y / pt12Counter;
		pt2.x = pt2x / pt12Counter;
		pt2.y = pt2y / pt12Counter;
		line(hough, pt1, pt2, line_color, 3, cv::LINE_AA); // show only 1 line
	}
	if (pt34Counter != 0) {
		pt3.x = pt3x / pt34Counter;
		pt3.y = pt3y / pt34Counter;
		pt4.x = pt4x / pt34Counter;
		pt4.y = pt4y / pt34Counter;
		line(hough, pt3, pt4, line_color, 3, cv::LINE_AA); // show only 1 line
	}
	if (pt12Counter != 0 && pt34Counter != 0) {
		dist = sqrt(pow(pt1.x - pt4.x, 2) + pow(pt1.y - pt4.y, 2));
		midX = (pt1.x + pt4.x) / 2.0;
		midY = (pt1.y + pt4.y) / 2.0;
		line(hough, cv::Point(midX, midY), cv::Point(midX, 250), cv::Scalar(255, 255, 0), 3, cv::LINE_AA);  // the lane's center
		line(hough, pt1, pt4, line_color, 3, cv::LINE_AA);
		//std::cout << hough.rows/2 <<"    "<< hough.cols/2 << std::endl;
		line(hough, cv::Point(hough.rows, hough.cols), cv::Point(hough.rows, 320), cv::Scalar(255, 0, 255), 3, cv::LINE_AA); //center line (reference)

		distL = sqrt(pow(pt1.x - midX, 2) + pow(pt1.y - midY, 2));
		distR = sqrt(pow(pt4.x - midX, 2) + pow(pt4.y - midY, 2));
		//std::cout << "test:  " <<distL <<"    "<< distR << std::endl;

		distOrigL = sqrt(pow(pt1.x - hough.rows, 2) + pow(pt1.y - midY, 2));
		distOrigR = sqrt(pow(pt4.x - hough.rows, 2) + pow(pt4.y - midY, 2));

		center = sqrt(pow(hough.rows - midX,2) + pow(midY - midY,2));
		line(hough, cv::Point(hough.rows, midY), cv::Point(midX, midY), cv::Scalar(255, 0, 255), 3, cv::LINE_AA);

		//std::cout << "center:  " << center << std::endl;
		//std::cout << "orig:  " << distOrigL <<"    "<< distOrigR << std::endl;
		//std::cout << "% diff:  " << (abs(distL-distOrigL)/distL)*100 << "%    " << (abs(distR-distOrigR)/distR)*100 << "%    "  <<(center/distR)*100<< "%" <<std::endl;
		std::ostringstream diffL, diffR, diffC; // convert float to string, so that we're able to use it in putText function
		float tempL, tempR;
		tempL = ((((distL-distOrigL) / distL) * 100) < 0) ? ((distL-distOrigL) / distL) * -100 : ((distL-distOrigL) / distL) * 100 ;
		tempR = ((((distR-distOrigR) / distR) * 100) < 0) ? ((distR-distOrigR) / distR) * -100 : ((distR-distOrigR) / distR) * 100 ;
		diffL << tempL;  // stupid abs function makes calculation a little off......
		diffR << tempR;
		diffC << (center/distR) *100;
		std::string diffLeft = diffL.str();
		std::string diffRight = diffR.str();
		std::string diffCenter = diffC.str();
		cv::putText(hough, diffLeft + "%", cv::Point(midX - 100, midY - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250)); // show on video
		cv::putText(hough, diffRight + "%", cv::Point(midX + 50, midY - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250)); // show on video
		cv::putText(hough, diffCenter + "%", cv::Point(midX-20 , midY - 70), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250)); // show on video

		timeElapse = 0; // when it detects 2 lanes again, reset the timer to 0
		switchingR = false; // when it sees 2 lanes again, don't display "switching lane text"
		switchingL = false;
						   //std::cout << "reset" << std::endl;

	}
	else if ((pt12Counter != 0 && pt34Counter == 0) || (pt12Counter == 0 && pt34Counter != 0)) { // if only 1 lane is detected
		t = ((double)cv::getCPUTickCount() - t) / cv::getTickFrequency();
		timeElapse += t;
		//std::cout << "time elapse = " << timeElapse << std::endl;
		//std::cout << "Times passed in seconds: " << timeElapse << std::endl;
		if (timeElapse > 0.3 && timeElapse < 3) { // typical time to switch lanes, with some deviation
			//std::cout << "Points:  " << pt1 << "     " << pt2 << "    " << pt3 << "    " << pt4 << std::endl;
			line(hough, cv::Point(hough.rows / 1.25, hough.cols), cv::Point(hough.rows / 1.25, 320), cv::Scalar(100, 200, 250), 3, cv::LINE_AA);
			// these 2 lines show boundary of where the detected lane shouldn't cross, else it will be switching lanes
			line(hough, cv::Point(hough.rows*1.25, hough.cols), cv::Point(hough.rows*1.25, 320), cv::Scalar(100, 200, 250), 3, cv::LINE_AA);

			//std::cout << "counters:  " << pt12Counter << "     " << pt34Counter << std::endl;
			// boundary if statement to determine if its switching lanes
			if ((pt1.x > hough.rows / 1.25 && pt1.x < hough.rows*1.25) || (pt4.x > hough.rows / 1.25 && pt4.x < hough.rows*1.25)) {
				cv::putText(hough, "CHANGING LANES", cv::Point(hough.rows / 2, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0, 0, 255), 3);
			}
			int distThresh = 200; // minimum distance from center line to determine its switching lanes
			if ((pt12Counter != 0 && pt34Counter == 0)) { // switching lane detection based on distance to center line
				distOrigL = sqrt(pow(pt1.x - hough.rows, 2) + pow(pt1.y - midY, 2));
				if (distOrigL < distThresh) {
					//cv::putText(hough, "CHANGING LANES", cv::Point(hough.rows/2, 50),cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,0,255), 3);
					switchingL = true;
				}
				//std::cout << "LEFT:  " << distOrigL <<std::endl;
			}
			else if ((pt12Counter == 0 && pt34Counter != 0)) {
				distOrigR = sqrt(pow(pt4.x - hough.rows, 2) + pow(pt4.y - midY, 2));
				if (distOrigR < distThresh) {
					//cv::putText(hough, "CHANGING LANES", cv::Point(hough.rows/2, 50),cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0,0,255), 3);
					switchingR = true;
				}
				//std::cout << "RIGHT:  " << distOrigR  <<std::endl;
			}

			if (switchingL) {
				cv::putText(hough, "LEFT", cv::Point(hough.rows / 6, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0, 0, 255), 3);
				//switchingL = false;
			}
			else if (switchingR) {
				cv::putText(hough, "RIGHT", cv::Point(hough.rows * 1.5, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0, 0, 255), 3);
				//switchingR = false;
			}
		}
		else if (timeElapse > 3.5) { // one lane for more than 3.5 seconds
			switchingR = false;
			switchingL = false;
		}
	}
	else { // no lanes detected
		if (switchingL && switchingR) {
			cv::putText(hough, "CHANGING LANES", cv::Point(hough.rows / 2, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0, 0, 255), 3);
		}
		if (switchingL) {
			cv::putText(hough, "LEFT", cv::Point(hough.rows / 6, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0, 0, 255), 3);
		}
		else if (switchingR) { // special condition when it doesn't detect any lane when changing lanes, but detect 1 lane before
			cv::putText(hough, "RIGHT", cv::Point(hough.rows * 1.5, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0, 0, 255), 3);
		}

	}


	// The code below is for standard Hough Line Transform
#if stdhough
	// makes video run slower :(
	/*
	cv::cvtColor(hough, hough, cv::COLOR_BGR2GRAY);
	cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_ADV);
	std::vector<cv::Vec4f> lines_std;
	ls->detect(hough, lines_std);
	cv::Mat drawnLines(hough);
	ls->drawSegments(drawnLines, lines_std);
	*/
#endif


	return hough;
}

/*
*	Show images
*/
void lane_detect::show_windows(cv::Mat hough_image, cv::Mat canny_image, cv::Mat gaussian_image, cv::Mat original) {
	//const std::string original_image = "Original Image";
	const std::string gaussian = "Gaussian Blur";
	const std::string canny = "Canny";
	const std::string hough_lines = "Hough Lines";

	//cv::namedWindow(original_image, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(gaussian, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(canny, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(hough_lines, cv::WINDOW_AUTOSIZE);

	//cv::imshow(original_image, original);
	cv::imshow(gaussian, gaussian_image);
	cv::imshow(canny, canny_image);
	cv::imshow(hough_lines, hough_image);
}


void lane_detect::detect() {

	//int width = 640, height = 360;	// Dimensions for the videos and birds-eye view
	int width = 700, height = 350;
	cv::VideoCapture cam = cv::VideoCapture("C:\\Users\\YuYu\\Desktop\\RPI2 stuff\\vid7.mp4");
	//cv::VideoCapture cam(1);
	cv::Rect rect(0, height / 2, width, height / 2);	// ROI
	cv::Mat gray;

	std::vector<cv::Point2f> origPoints;
	std::vector<cv::Point2f> dstPoints;
	IPM ipm = init_ipm(width, height, origPoints, dstPoints);	// Initialize the birds-eye view
	cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"


#if hsv

	int iLowH = 0;
	int iHighH = 88;

	int iLowS = 0;
	int iHighS = 206;

	//int iLowV = 221;
	//int iHighV = 255;

	// for vid7
	int iLowV = 161;
	int iHighV = 191;

	//Create trackbars in "Control" window
	cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cv::createTrackbar("HighH", "Control", &iHighH, 179);

	cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cv::createTrackbar("HighS", "Control", &iHighS, 255);

	cv::createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cv::createTrackbar("HighV", "Control", &iHighV, 255);

#endif

#if binarize
		int threshL = 193;
		int threshH = 255;

		cv::createTrackbar("threshL", "Control", &threshL, 255); //Value (0 - 255)
		cv::createTrackbar("threshH", "Control", &threshH, 255);
#endif

	while (1) {
		cam >> original;
		resize(original, original, cv::Size(width, height), 0, 0, cv::INTER_CUBIC);
		cv::cvtColor(original, gray, cv::COLOR_BGR2GRAY);

		//cv::flip(original, original, -1);
		//original = original(rect);
		//original.copyTo(gaussian_image);	// Keep original picture separate.
		gray.copyTo(gaussian_image);

#if hsv
		cvtColor(original, gaussian_image, cv::COLOR_BGR2HSV); //Convert frame from BGR to HSV

		inRange(gaussian_image, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), gaussian_image);
		inRange(gaussian_image, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), gaussian_image);

		erode(gaussian_image, gaussian_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		dilate(gaussian_image, gaussian_image, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

#endif

#if binarize
		threshold(gaussian_image, gaussian_image, threshL, threshH, cv::THRESH_BINARY_INV);
#endif


		//ipm.applyHomography(gaussian_image, gaussian_image);	// call this for birds-eye view
		//ipm.drawPoints(origPoints, original);

		gaussian_blur(gaussian_image);
		canny_image = canny_edge(gaussian_image);

		//canny_image = sobel_edge(gaussian_image);
		hough_image = hough_transform(canny_image, original);

		show_windows(hough_image, canny_image, gaussian_image, original);
		if (cv::waitKey(30) >= 0)
			break;
	}
}


IPM lane_detect::init_ipm(int width, int height, std::vector<cv::Point2f>& origPoints, std::vector<cv::Point2f>& dstPoints) {

	// The 4-points at the input image
	origPoints.push_back(cv::Point2f(0 + 150, height - 50));
	origPoints.push_back(cv::Point2f(width, height));
	origPoints.push_back(cv::Point2f(width / 2 + 150, 200));
	origPoints.push_back(cv::Point2f(width / 2 - 150, 200));

	// The 4-points correspondences in the destination image
	dstPoints.push_back(cv::Point2f(0, height + 200));
	dstPoints.push_back(cv::Point2f(width + 100, height + 200));
	dstPoints.push_back(cv::Point2f(width, 0));
	dstPoints.push_back(cv::Point2f(0, 0));


	IPM ipm(cv::Size(width, height), cv::Size(width, height), origPoints, dstPoints);
	return ipm;
}

cv::VideoCapture cam = cv::VideoCapture("C:\\Users\\YuYu\\Desktop\\RPI2 stuff\\vid7.mp4");
