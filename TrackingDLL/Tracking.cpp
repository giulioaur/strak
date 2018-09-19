#define WIN32_LEAN_AND_MEAN
#define _CRT_SECURE_NO_WARNINGS

#include "Tracking.h"
#include <limits>
#include <atomic>
#include "getVideo.hpp"
#include "raspberry.hpp"
#include "UDPSocket.hpp"
#include "errorCodes.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define AS_EXE
#define CUT_WIDTH 140
#define RED 2
#define GREEN 1
#define BLUE 0
#define UPPER_LIP 0
#define LOWER_LIP 1
#define LIP_CORNER 2

using namespace std;
using namespace cv;
using namespace strak;

/************************************************** Typedef ******************************************************/

typedef vector<Point> Points;

/************************************************** Constants ******************************************************/

const string RASP_ADDRESS = "192.168.137.1";
const int RASP_PORT = 5252;

/************************************************** Function header ******************************************************/

void receiveImage();
void processImage();
Mat cropMouth(const Mat &croppedFrame, const Mat &filtered);
array<Points, 2> getStartingPoints(const Size &size, const Points middleLine);
Points getMiddleLine(const Size &size);
Mat getRedImage(const Mat &img);
void calibrateLip(const Mat &img, const array<Points, 2> &points);
array<Points, 3> getFeaturePoints(const Mat &img, const Points &startingPoint);
void drawPoints(const Mat &img, const Points &coords, const Scalar &color = { 255, 0, 0 });
void drawRect(const Mat &img, const Points &coords, const Scalar &color = { 255, 0, 0 });
Mat getEdgeImage(const Mat &img);

/************************************************** Global variable ******************************************************/

string pathToScripts;
thread mainThread, receiveThread;
bool stopVideo = false, isCalibrating = true, isLoopActive = false, isReceiving = false, isRaspConnected;
atomic<bool> newFrame = false, newReceivedFrame = false, sendFlag = false;
array<Scalar, 2> lipMean, lipStdDev;
Mat image, normalizedImage, lipImage, video;
uint16_t *featurePoints = nullptr;
uint8_t *PPS;
int PPSSize;
UDPSocket *raspSocket, *senderSocket, *receiverSocket;
struct IPA {
	string localAddress, remoteAddress;
	int localPort, remotePort;
} *ipconfig;
int maxGreen = 80, maxBlue = 108;
Points middleLine;


/************************************************** API Functions ******************************************************/

/// Initializes variables and components.
int __stdcall init(const char *localAddress, int localPort, const char *remoteAddress, int remotePort, const char *cameraScriptsPath) {
	// Init network variable.
	ipconfig = new struct IPA();
	ipconfig->localAddress = localAddress; ipconfig->localPort = localPort;
	ipconfig->remoteAddress = remoteAddress; ipconfig->remotePort = remotePort;

	// Init other variables.
	pathToScripts = cameraScriptsPath;
	featurePoints = new uint16_t[12];
	PPS = nullptr;
	stopVideo = newFrame = newReceivedFrame = isCalibrating = sendFlag = false;
	middleLine = getMiddleLine(Size{ imageWidth - 140, imageHeight - 20 });

	// Init newtork and check for errors.
	int status = init_network();
	if (status)		return status;

	// Init decoder and check for errors.
	status = init_decoder();
	if (status)		return status;

	// Init sockets.
	raspSocket = new UDPSocket();
	senderSocket = new UDPSocket();
	receiverSocket = new UDPSocket();
	status = raspSocket->bind(RASP_ADDRESS, RASP_PORT) ? TrackingError::OK : TrackingError::NO_RASP;
	isRaspConnected = status == TrackingError::OK && init_raspberry(pathToScripts) == TrackingError::OK ? true : false;

	if (isRaspConnected) {
		mainThread = thread(processImage);
	}
	else {
		status = receiverSocket->bind(ipconfig->localAddress, ipconfig->localPort) ? TrackingError::NO_RASP : TrackingError::SOCKET_BIND;
		if (status == TrackingError::NO_RASP)	receiveThread = thread(receiveImage);
	}

	return status;
}

/// Deallocates variables and terminates components.
int __stdcall end() {
	stopVideo = true;
	sendFlag = false;

	// Wait main thread to terminate.
	if (isRaspConnected)	mainThread.join();

	// Delete sockets
	delete raspSocket; delete receiverSocket; delete senderSocket;
	if (!isRaspConnected)	receiveThread.join();

	// Clear variables.
	delete ipconfig;
	delete[] featurePoints;
	delete[] PPS;

	// Clean decoder component.
	int status = end_decoder();
	if (status)		return status;

	// Shut down network.
	status = end_network();
	if (status)		return status;

	// Shut down raspberry application.
	return isRaspConnected ? end_raspberry(pathToScripts) : TrackingError::OK;
}

/// Returns the feature points.
int __stdcall getFeatures(uint16_t *buffer) {
	if (featurePoints != nullptr && newFrame) {
		memcpy(buffer, featurePoints, 12 * sizeof(uint16_t));
		newFrame = false;
		return TrackingError::OK;
	}

	return isLoopActive ? TrackingError::NO_FRAME : TrackingError::UNKNOWN;
}

/// Sets the color's threshold.
void __stdcall setColorFilter(int green, int blue) {
	maxGreen = green; maxBlue = blue;
}

/// Sets the coordinates of the line between two lips.
void __stdcall setMiddleLine(int height) {
	int width = imageWidth - CUT_WIDTH;
	middleLine = { { (width / 2) - 20, height },{ (width / 2) + 20, height } };
}

/// Returns the three kind of images.
void __stdcall getImages(uint8_t *img, uint8_t *normalized, uint8_t *lip) {
	if (img != nullptr) 
		memcpy(img, image.data, image.rows * image.cols * image.channels());
	if (normalized != nullptr)
		memcpy(normalized, normalizedImage.data, image.rows * image.cols * image.channels());
	if (lip != nullptr && lipImage.rows > 0)
		memcpy(lip, lipImage.data, image.rows * image.cols * image.channels());
	//imshow("Image", image);
	//imshow("Normalized", normalizedImage);
	//if(lipImage.rows > 0)
	//	imshow("Lip", lipImage);
}

/// Returns the video streamed from the remote instance of the library.
int __stdcall getVideo(uint8_t *frame) {
	if (!isReceiving)		return TrackingError::NO_RECEIVE;
	if (!newReceivedFrame)	return TrackingError::NO_FRAME;
	else {
		memcpy(frame, video.data, (imageWidth - 140) * (imageHeight - 20) * 3);
		newReceivedFrame = false;
		return TrackingError::OK;
	}
}

/// Starts the calibration phase.
void __stdcall startCalibration() {
	isCalibrating = true;
}

/// Stops the calibration phase.
void __stdcall stopCalibration() {
	isCalibrating = false;
}

/// Changes the flag for the data sending.
void __stdcall setSendFlag(bool flag) {
	sendFlag = flag;

	// Send PPS as first frame.
	if (flag)	senderSocket->sendTo(ipconfig->remoteAddress, ipconfig->remotePort, (char *)PPS, PPSSize);
}

/// Flats the array of features point into a vector of float.
void flatPoints(const array<Points, 3> points) {
	size_t i = 0;
	for (size_t j = 0; j < points.size(); ++j) {
		if (points[j].size() > 0) {
			featurePoints[i++] = points[j][0].x;
			featurePoints[i++] = points[j][0].y;
			featurePoints[i++] = points[j][1].x;
			featurePoints[i++] = points[j][1].y;
		}
	}
}



/************************************************** MAIN ******************************************************/

#ifdef AS_EXE
// Main is only used for testing the .exe file.
int main() {
	init("192.168.137.108", 5858, "192.168.137.110", 5151, "D:\\Documents\\Unity Projects\\Master Thesis\\Assets\\CameraScripts\\");

	namedWindow("frame", WINDOW_AUTOSIZE);
	namedWindow("normalized frame", WINDOW_AUTOSIZE);
	//namedWindow("edges", WINDOW_AUTOSIZE);
	namedWindow("only lip", WINDOW_AUTOSIZE);
	//namedWindow("sliders", WINDOW_AUTOSIZE);

	int key;
	while (!stopVideo) {
		if ((key = cv::waitKey(0)) >= 0) {
			switch (key) {
			case 27:
				stopVideo = true; break;
			case 119:
				setMiddleLine(middleLine[0].y - 5);
				break;
			case 115:
				setMiddleLine(middleLine[0].y + 5);
				break;
			case 32:
				isCalibrating ^= true; break;
			default:
				setSendFlag(sendFlag ? false : true); break;
			}
		}
	}

	end();
	return 0;
}
#endif // AS_EXE

/************************************************** Other functions ******************************************************/

/// The function responsible for receiving the frame from remote.
void receiveImage() {
	const int MAX_SIZE = 100000;
	int size = MAX_SIZE;
	bool isWorking = true;
	isReceiving = true;
	Mat frame{ Size{ (int)imageWidth, (int)imageHeight }, CV_8UC3 };
	Rect crop{ Point{ CUT_WIDTH / 2, 0 }, Point{ imageWidth - CUT_WIDTH / 2, imageHeight - 20 } };

	// Receive video from remote.
	while (!stopVideo && size > 0 && isWorking) {
		AVPacket packet; av_init_packet(&packet);
		size = MAX_SIZE;
		uint8_t *data = new uint8_t[size];

		isWorking = receiverSocket->receiveFrom((char *)data, size);

		// Decode packet.
		if (size > 0) {
			packet.data = data;
			packet.size = size;

			if (decode(packet, frame.data) == 0) {
				// Crop frame
				Mat croppedFrame = frame(crop);
				cvtColor(cropMouth(croppedFrame, getRedImage(croppedFrame)), video, COLOR_BGR2RGB);
				newReceivedFrame = true;

				#ifdef AS_EXE
				imshow("frame", video);
				#endif // AS_EXE

			}

			//cout << "#" << endl;
		}
	}

	isReceiving = false;
}

/// The main function responsible for the processing of lip image.
void processImage() {
	// Init variable
	Mat frame{ Size{ (int)imageWidth, (int)imageHeight }, CV_8UC3 }, lastFrame;
	auto points = getStartingPoints(Size{ imageWidth - 140, imageHeight - 20 }, middleLine);
	Rect crop{ Point{ 70, 0 }, Point{ imageWidth - 70, imageHeight - 20 } };
	float diffThreshold = 1500;
	array<Points, 3> featurePoints;

	#ifdef AS_EXE
	// Sliders window.
	//createTrackbar("MaxGreen", "sliders", &maxGreen, 255);
	//createTrackbar("MaxBlue", "sliders", &maxBlue, 255);
	#endif // AS_EXE

	isLoopActive = true;

	// Start receiving from raspberry.
	while (!stopVideo) {
		// Receive the packet, then forward and decode it.
		AVPacket packet = receive(raspSocket->getSocket());

		if (PPS == nullptr) { // Save PPS.
			PPSSize = packet.size;
			PPS = new uint8_t[PPSSize];
			memcpy(PPS, packet.data, packet.size);
		}

		if (sendFlag) {
			int tmp = senderSocket->sendTo(ipconfig->remoteAddress, ipconfig->remotePort, (char *)packet.data, packet.size);
			//if (tmp <= 0)	cout << WSAGetLastError() << ", ";
		}

		decode(packet, frame.data);

		Mat croppedFrame = frame(crop);

		// Do image processing.
		Mat filtered = getRedImage(croppedFrame);
		cvtColor(filtered, normalizedImage, COLOR_BGR2RGB);

		float diff = lastFrame.rows > 0 ? norm(lastFrame - croppedFrame) : diffThreshold + 1;
		lastFrame = croppedFrame.clone();

		if (isCalibrating) {	// Calibration phase.
			Mat cpy = croppedFrame.clone();
			points = getStartingPoints(Size{ imageWidth - 140, imageHeight - 20 }, middleLine);
			
			// Draw rectangle lips and middle line.
			drawRect(cpy, points[0]);
			drawRect(cpy, points[1]);
			drawPoints(cpy, middleLine);

			calibrateLip(filtered, points);
			cvtColor(cpy, image, COLOR_BGR2RGB);
			newFrame = true;

			#ifdef AS_EXE
			imshow("frame", cpy);
			#endif // AS_EXE
		}						// Feature extraction phase. Only if the frame is different from previous.
		else if (diff >= diffThreshold || featurePoints[0].size() + featurePoints[1].size() + featurePoints[2].size() < 6) {
			featurePoints = getFeaturePoints(filtered, middleLine);
			
			// Draw feature points and flat them.
			drawPoints(croppedFrame, middleLine, Scalar{ 0, 255, 0 });
			for (size_t i = 0; i < featurePoints.size(); ++i)
				drawPoints(croppedFrame, featurePoints[i]);
			flatPoints(featurePoints);

			cvtColor(croppedFrame, image, COLOR_BGR2RGB);
			newFrame = true;

			#ifdef AS_EXE
			imshow("frame", croppedFrame);
			#endif // AS_EXE
		}
	}

	isLoopActive = false;
}

/// Returns an image with only the mouth.
Mat cropMouth(const Mat &croppedFrame, const Mat &filtered) {
	vector<vector<Point> > contours;
	Mat edges = getEdgeImage(filtered), mouth = croppedFrame.clone();

	// Find the mouth contour.
	findContours(edges, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	int max = 0, maxI = -1;
	for (size_t i = 0; i < contours.size(); i++) {
		if (max < contours[i].size()) {
			max = contours[i].size();
			maxI = i;
		}
	}

	// Crop the mouth.
	if (maxI >= 0) {
		Mat mask = Mat::zeros(edges.size(), CV_8UC3);
		drawContours(mask, contours, (int)maxI, Scalar(255, 255, 255), FILLED);
		bitwise_and(croppedFrame, mask, mouth);
	}

	return mouth;
}

/// Set an initial middle line.
Points getMiddleLine(const Size &size) {
	int centerX = size.width / 2, centerY = size.height / 2;

	Points points = { { centerX - 20, centerY - 70 },{ centerX + 20, centerY - 70 } };

	return points;
}

/// Returns the lip contour as rectangle.
array<Points, 2> getStartingPoints(const Size &size, const Points middleLine) {
	Points upperLip = { { middleLine[0].x - 120, middleLine[0].y - 80 }, { middleLine[1].x + 120, middleLine[0].y - 20 } };
	Points lowerLip = { { middleLine[0].x - 90, middleLine[0].y + 20 },{ middleLine[1].x + 90, middleLine[0].y + 70 } };

	return { upperLip, lowerLip };
}

/// Computes the normalized image and discards all the non-lip pixels.
Mat getRedImage(const Mat &img) {
	Mat redImg = img.clone();
	Mat ycrcb;

	//cvtColor(redImg, ycrcb, COLOR_BGR2YCrCb);

	Mat colors[3], floatColors[3];
	split(redImg, colors);

	colors[0].convertTo(floatColors[0], CV_32FC1);
	colors[1].convertTo(floatColors[1], CV_32FC1);
	colors[2].convertTo(floatColors[2], CV_32FC1);

	Mat tot = floatColors[0] + floatColors[1] + floatColors[2];

	floatColors[0] = floatColors[0] / tot * 255;
	floatColors[1] = floatColors[1] / tot * 255;
	floatColors[2] = floatColors[2] / tot * 255;

	floatColors[0].convertTo(colors[0], CV_8UC1);
	floatColors[1].convertTo(colors[1], CV_8UC1);
	floatColors[2].convertTo(colors[2], CV_8UC1);

	merge(colors, 3, redImg);

	for (size_t i = 0; i < redImg.rows; ++i) {
		for (size_t j = 0; j < redImg.cols; ++j) {
			//Vec3b pixel = ycrcb.at<Vec3b>(i, j);
			float red = floatColors[RED].at<float>(i, j), green = floatColors[GREEN].at<float>(i, j), blue = floatColors[BLUE].at<float>(i, j);
			if (!(green <= maxGreen && blue <= maxBlue))
				redImg.at<Vec3b>(i, j) = { 0, 0, 0 };
		}
	}

	#ifdef AS_EXE
	imshow("normalized frame", redImg);
	#endif // AS_EXE
	return redImg;
}

bool isUpperLip(const Vec3b &pixel) {
	return abs(pixel[RED] - lipMean[UPPER_LIP][RED]) < lipStdDev[UPPER_LIP][RED] &&
		   abs(pixel[GREEN] - lipMean[UPPER_LIP][GREEN]) < lipStdDev[UPPER_LIP][GREEN] &&
		   abs(pixel[BLUE] - lipMean[UPPER_LIP][BLUE]) < lipStdDev[UPPER_LIP][BLUE];
}

bool isLowerLip(const Vec3b &pixel) {
	return abs(pixel[RED] - lipMean[LOWER_LIP][RED]) < lipStdDev[LOWER_LIP][RED] &&
		   abs(pixel[GREEN] - lipMean[LOWER_LIP][GREEN]) < lipStdDev[LOWER_LIP][GREEN] &&
		   abs(pixel[BLUE] - lipMean[LOWER_LIP][BLUE]) < lipStdDev[LOWER_LIP][BLUE];
}

bool isLip(const Vec3b &pixel) {
	return isUpperLip(pixel) || isLowerLip(pixel);
}

/// Computes the image with only lip pixels.
void computeLipImage(const Mat &img) {
	// Filter the lip pixels.
	Mat lips = img.clone();
	for (size_t i = 0; i < lips.rows; ++i)
		for (size_t j = 0; j < lips.cols; ++j)
			if (!isLip(lips.at<Vec3b>(i, j)))
				lips.at<Vec3b>(i, j) = { 0, 0, 0 };

	#ifdef AS_EXE
	imshow("only lip", lips);
	#endif // AS_EXE

	cvtColor(lips, lipImage, COLOR_RGB2BGR);
}

/// Retrieves the mean and std deviation of the upper and lower lip.
void calibrateLip(const Mat &img, const array<Points, 2> &points) {
	Mat upperLip = img(Rect{ points[0][0], points[0][1] });
	Mat lowerLip = img(Rect{ points[1][0], points[1][1] });

	meanStdDev(upperLip, lipMean[UPPER_LIP], lipStdDev[UPPER_LIP]);
	meanStdDev(lowerLip, lipMean[LOWER_LIP], lipStdDev[LOWER_LIP]);

	// Get lip image.
	computeLipImage(img);
}

/// Computes the derivative of the image using Sobel filter.
Mat getEdgeImage(const Mat &img) {
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	// Apply Sobel filter.
	Mat grad_x, grad_y, grad;
	Mat abs_grad_x, abs_grad_y;
	float scale = 1;
	float delta = 0;
	int ddepth = CV_16S;
	int kernSize = 3;

	GaussianBlur(gray, gray, Size(3, 3), 0, 0, BORDER_DEFAULT);

	Sobel(gray, grad_x, ddepth, 1, 0, kernSize, scale, delta, BORDER_DEFAULT);
	Sobel(gray, grad_y, ddepth, 0, 1, kernSize, scale, delta, BORDER_DEFAULT);

	convertScaleAbs(grad_x, abs_grad_x);
	convertScaleAbs(grad_y, abs_grad_y);
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

	#ifdef AS_EXE
	//imshow("edges", grad);
	#endif //AS_EXE

	return grad;
}

/// Retrieves feature points of the lip from the image.
array<Points, 3> getFeaturePoints(const Mat &img, const Points &startingPoint) {
	array<Points, 3> points;
	Point center{ img.cols / 2, img.rows / 2 };
	Mat edges = getEdgeImage(img);
	const int edgeTreshold = 80, lipHeight = 45, maxNoisyPoint = 8;

	function<bool(const Vec3b&)> isBlack = [](const Vec3b &pixel) {
		return pixel == Vec3b{ 0, 0, 0 };
	};

	// Get lip image.
	computeLipImage(img);

	// Get upper and lower lip
	for (size_t i = 0; i < 2; ++i) {
		int startLip = -1, noisyPoint = 0, lastNonNoisy = -1;
		bool found = false;
		int dir = -1;//i == 0 ? 1 : -1;//dir = i == 0 ? -1 : 1;
		int x = startingPoint[0].x + (startingPoint[1].x - startingPoint[0].x) / 2, y = i == 0 ? startingPoint[0].y : img.rows - 1; //y = startingPoint[0].y;

		// Iterate until lips have been found.
		do {
			found = false;

			// If not the first iteration, start from the last non noisy point.
			if (startLip > 0)
				y = startLip + dir;

			// Reset position and counters.
			startLip = lastNonNoisy = -1;
			noisyPoint = 0;

			for (; y >= 0 && y < img.rows && !found; y += dir) {
				Vec3b pixel = img.at<Vec3b>(y, x);
				uchar edge = edges.at<uchar>(y, x);

				if (isLip(pixel) && startLip < 0)
					startLip = y;

				// Check if the pixel is a lip or a noisy point.
				if (!isLip(pixel) && startLip >= 0)
					++noisyPoint;
				else if (noisyPoint < 1)
					lastNonNoisy = y;

				if (noisyPoint > maxNoisyPoint)
					found = true;
			}
		} while (found && abs(lastNonNoisy - startLip) < lipHeight);

		// Push the lip if it has been detected.
		if (startLip > 0) {
			points[i].push_back({ x, startLip });
			points[i].push_back({ x, lastNonNoisy > 0 ? lastNonNoisy : y });
		}
	}

	if (points[0].size() > 0 && points[1].size() > 0) {
		// Get lip corner
		int x = points[1][0].x, y;
		int startLip = -1, lastNonNoisy = points[1][1].y, lastY;
		bool found = true;
		const int neighSize = 10;

		int openess = points[1][0].y - points[0][0].y;

		y = points[0][0].y + openess / 2;

		// Search for the lip border.
		x = startingPoint[1].x + 20;
		while (x < img.cols) {
			found = false;
			for (; x < img.cols && !found; ++x) {
				if (edges.at<uchar>(y, x) > edgeTreshold && isBlack(img.at<Vec3b>(y, x + 1))) {
					found = true; lastNonNoisy = x;
				}
			}
		}

		lastY = y;
		// Found corner
		found = true;
		for (; x < img.cols && found; ++x) {
			found = false;
			for (y = lastY - neighSize; y > lastY + neighSize && !found; --y) {
				if (edges.at<uchar>(y, x) > edgeTreshold) {
					found = true;
					lastY = y;
				}
			}
		}

		points[LIP_CORNER].push_back({ lastNonNoisy, y });
		points[LIP_CORNER].push_back({ lastNonNoisy - 20, y });
	}

	return points;
}

/// Draws the points on the image as a continous shape.
void drawPoints(const Mat &img, const Points &coords, const Scalar &color) {
	// Draw lines between points.
	if (coords.size() > 0)
		for (size_t i = 0; i < coords.size() - 1; ++i)
			line(img, coords[i], coords[i != coords.size() - 1 ? i + 1 : 0], color);
}

/// Draws the a rectangle on the image.
void drawRect(const Mat &img, const Points &coords, const Scalar &color) {
	// Draw rectangle using points as corner.
	if (coords.size() >= 2)
		rectangle(img, Rect{ coords[0], coords[1] }, color);
}