#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "roboDNN/roboDNN.h"
#include <time.h>

using namespace cv;
using namespace std;

void grayscaleproba();
void szinesproba();
void splitmerge();
void datatype();
int capture();
void mergeprobak();
void vectorproba();
void robodnn(Network& net);
void robodnn_debug();
void reorder_rgb(cv::Mat& img, std::vector<float>& array);
void reorder_rowwise(cv::Mat& img, std::vector<float>& array);
void showPixelValues(cv::Mat mat, float* img);
void ZeroMeanUnitDev(cv::Mat& Img);
void ZeroMeanUnitDev2(cv::Mat& Img);
void robodnn_live(Network& net, cv::Mat img);


int main(){
	//capture();
	
	Network net("F:/Projects/RobonAUT2/Dipterv/software/RPi/RoboDNN", "example.cfg");
	const int RUNS = 100;
	clock_t begin = clock();
	for (int i = 0; i < RUNS; i++)
	{
		robodnn(net);
	}
	clock_t end = clock();
	float time_spent = (float)(end - begin) / (RUNS*CLOCKS_PER_SEC);
	cout << "time of forwardprop is: " << time_spent <<std::endl;
	getchar();
	
	return 0;
}

void robodnn(Network& net)
{
	Mat testimg(32, 32, CV_8UC3);
	testimg = imread("C:/roadSignDataset/datasetFinal/testFULL/08/img(1601).jpg", CV_LOAD_IMAGE_COLOR);
	cv::copyMakeBorder(testimg, testimg, 2, 2, 2, 2, BORDER_CONSTANT, 0);
	testimg.convertTo(testimg, CV_32FC3, 1.0 / 255.0);
	ZeroMeanUnitDev(testimg);
	std::vector<float> imgvec;
	reorder_rowwise(testimg, imgvec);
	float* imgptr = &imgvec[0];
	float* result = net.forward(imgptr);
	float maxval = 0;
	int index = 0;
	for (int i = 0; i < 54; i++)
	{
		if (result[i] > maxval)
		{
			maxval = result[i];
			index = i;
		}
	}
	printf("predicted class is: %d\n", index);
}

void robodnn_live(Network& net , cv::Mat img)
{
	ZeroMeanUnitDev(img);
	std::vector<float> imgvec;
	reorder_rowwise(img, imgvec);
	float* imgptr = &imgvec[0];
	float* result = net.forward(imgptr);
	float maxval = -1;
	int index = -1;
	for (int i = 0; i < 54; i++)
	{
		if (result[i] > maxval)
		{
			maxval = result[i];
			index = i;
		}
	}
	printf("predicted class is: %d\n", index);
}

void robodnn_debug(){

	//tesztk�p bet�lt�se
	Scalar m;   //Scalar is a class for a 4 variable vector. m is its instance.        
	Scalar std;

	Mat testimg(32, 32, CV_8UC3);
	//testimg = imread("F:/Projects/RobonAUT2/Dipterv/software/RPi/diff/00/img(0).jpg", CV_LOAD_IMAGE_COLOR);
	testimg = imread("C:/roadSignDataset/datasetFinal/testFULL/42/img(8556).jpg", CV_LOAD_IMAGE_COLOR);

	cv::copyMakeBorder(testimg, testimg, 2, 2, 2, 2, BORDER_CONSTANT, 0);
	imshow("testimg1", testimg);

	meanStdDev(testimg, m, std);
	cout << "mean of ch0: " << m[0] << endl;
	cout << "std of ch0: " << std[0] << endl;
	cout << "mean of ch1: " << m[1] << endl;
	cout << "std of ch1: " << std[1] << endl;
	cout << "mean of ch2: " << m[2] << endl;
	cout << "std of ch2: " << std[2] << endl << endl;

	testimg.convertTo(testimg, CV_32FC3, 1.0 / 255.0);

	meanStdDev(testimg, m, std);
	cout << "mean of ch0: " << m[0] << endl;
	cout << "std of ch0: " << std[0] << endl;
	cout << "mean of ch1: " << m[1] << endl;
	cout << "std of ch1: " << std[1] << endl;
	cout << "mean of ch2: " << m[2] << endl;
	cout << "std of ch2: " << std[2] << endl << endl;

	cv::namedWindow("testimg", WINDOW_NORMAL);
	cv::resizeWindow("testimg", 500, 500);
	imshow("testimg", testimg);

	/*
	cv::normalize(testimg, testimg, 1.0 , -1.0);
	meanStdDev(testimg, m, std);
	cout << "mean of ch0: " << m[0] << endl;
	cout << "std of ch0: " << std[0] << endl;
	cout << "mean of ch1: " << m[1] << endl;
	cout << "std of ch1: " << std[1] << endl;
	cout << "mean of ch2: " << m[2] << endl;
	cout << "std of ch2: " << std[2] << endl << endl;
	*/

	ZeroMeanUnitDev(testimg);
	meanStdDev(testimg, m, std);
	cout << "mean of ch0: " << m[0] << endl;
	cout << "std of ch0: " << std[0] << endl;
	cout << "mean of ch1: " << m[1] << endl;
	cout << "std of ch1: " << std[1] << endl;
	cout << "mean of ch2: " << m[2] << endl;
	cout << "std of ch2: " << std[2] << endl;

	std::vector<float> imgvec;
	reorder_rowwise(testimg, imgvec);
	float* imgptr = &imgvec[0];
	//float*  imgptr = reorder_rowwise(testimg);

	Network net("F:/Projects/RobonAUT2/Dipterv/software/RPi/RoboDNN", "example.cfg");
	float* result = net.forward(imgptr);
	for (int i = 0; i < 54; i++)
	{
		printf("class: %d is: %llf\n", i, result[i]);
	}

	printf("forwardprop finished");

	waitKey(0);
}


int capture()
{
	Network net("F:/Projects/RobonAUT2/Dipterv/software/RPi/RoboDNN", "example.cfg");
	const int fps = 20;
	VideoCapture vid(0);
	if (!vid.isOpened())
	{
		return -1;
	}
	Mat edges;
	while (1)
	{
	waitKey(100);
		//k�pek, frame-ek bet�lt�se a kamer�r�l
		Mat frame;
		vid >> frame; // get a new frame from camera, or vid.read(frame);

		//Canny �ldetektor
		//cvtColor(frame, edges, COLOR_BGR2GRAY);
		//GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
		//Canny(edges, edges, 0, 30, 3);
		//imshow("webcam", frame);
		//imshow("edges", edges);
		
		//k�p 640*480-r�l 480*480-ra. Jobb s�v kiv�g�sa.
		Rect ROI(0, 0, frame.rows, frame.rows);
		Mat Frame480 = frame(ROI);		//Mat-nak �tadva egy Rect-et kiv�gja �s visszat�r egy kiv�gott Mat-tal
		//imshow("480x480", Frame480);

		//k�p �tsk�l�z�sa 640*480-r�l 28x28-ra
		Mat Frame28(28,28,CV_8UC3);
		cv::resize(Frame480, Frame28, Frame28.size(), 0, 0, CV_INTER_AREA);
		cv::namedWindow("resized", WINDOW_NORMAL);
		cv::resizeWindow("resized", 500, 500);
		imshow("resized", Frame28);

		//k�p konvert�l�sa ucharr�l float-ra
		Mat FrameFloat;
		Frame28.convertTo(FrameFloat, CV_32FC3, 1.0 / 255.0);
		
		//pixelek �trendez�se bgr-r�l rgb-re.
		//std::vector<float> imgvec;
		//reorder_rgb(FrameFloat, imgvec);
		//float* imgptr = &imgvec[0];

		//pixelek �trendez�se bgr-r�l rrr....gggg.....bbb....-re
		//std::vector<float> imgvec2;
		//reorder_rowwise(FrameFloat , imgvec2);
		//float* imgptr2 = &imgvec2[0];

		float*  imgptr2;  // = reorder_rowwise(FrameFloat);

		//showPixelValues(FrameFloat , imgptr2);
		
		robodnn_live(net , FrameFloat);
		
		//waitKey(0);
		
		if (waitKey(1000/fps) >= 0 )	//if any key pressed
		{
			break;
		}
		
	}

	return 0;
}




void showPixelValues(cv::Mat mat, float* img){
	//pixelek ki�r�sa
	cout << endl << "RRRRRRRR" << endl;
	for (int i = 0; i < mat.rows* mat.cols * mat.channels() / 3; i++)
	{
		cout << img[i] << " ";
	}
	cout << endl << "GGGGGGGG" << endl;
	for (int i = mat.rows* mat.cols * mat.channels() / 3; i < mat.rows* mat.cols * mat.channels() * 2 / 3; i++)
	{
		cout << img[i] << " ";
	}
	cout << endl << "BBBBBBB" << endl;
	for (int i = mat.rows* mat.cols * mat.channels() * 2 / 3; i < mat.rows* mat.cols * mat.channels(); i++)
	{
		cout << img[i] << " ";
	}
}


void reorder_rgb(cv::Mat& img)
{
	std::vector<float> array;

	//edgybf�gg� float* gy�rt�sa a k�pre
	array.clear();
	if (img.isContinuous()) {
		array.assign((float*)img.datastart, (float*)img.dataend);	//�tm�solom az �sszef�gg� Mat mem�riater�let�t egy std::vectorba
	}
	else {
		for (int i = 0; i < img.rows; ++i) {	//ha nem egybef�gg�, soronk�nt akkor is egybef�gg�, ez�rt lek�rem minden sor kezd�c�m�t
			array.insert(array.end(), img.ptr<float>(i), img.ptr<float>(i)+img.cols);	//�s soronk�nt egym�s ut�n m�solom egy std::vectorba.
		}
	}

	//reorder pixel from bgr to rgb
	std::vector<float> temp;
	for (int i = 0; i < array.size(); i += 3)
	{
		temp.push_back(array[i]);
		array[i] = array[i + 2];
	}
	for (int i = 0, j = 0; i < array.size(); i += 3, j++)
	{
		array[i + 2] = temp[j];
	}

	return;
}

void reorder_rowwise(cv::Mat& img, std::vector<float>& array)
{
	//std::vector<float> array;

	//edgybf�gg� float* gy�rt�sa a k�pre
	array.clear();
	if (img.isContinuous()) {
		array.assign((float*)img.datastart, (float*)img.dataend);	//�tm�solom az �sszef�gg� Mat mem�riater�let�t egy std::vectorba
	}
	else {
		for (int i = 0; i < img.rows; ++i) {	//ha nem egybef�gg�, soronk�nt akkor is egybef�gg�, ez�rt lek�rem minden sor kezd�c�m�t
			array.insert(array.end(), img.ptr<float>(i), img.ptr<float>(i)+img.cols);	//�s soronk�nt egym�s ut�n m�solom egy std::vectorba.
		}
	}

	//channelek sz�tv�laszt�sa
	std::vector<float> R;
	std::vector<float> G;
	std::vector<float> B;
	for (int i = 0; i < array.size(); i += 3)
	{
		B.push_back(array[i + 0]);
		G.push_back(array[i + 1]);
		R.push_back(array[i + 2]);
	}

	//channelek egyes�t�se
	array.assign( R.begin(), R.end());
	array.insert( array.end(), G.begin(), G.end());
	array.insert( array.end(), B.begin(), B.end());

	//return &array[0];
	return;
}
/*
void ZeroMeanUnitDev2(cv::Mat& Img)
{
	// accept only flaot type matrices
	CV_Assert(Img.depth() == CV_32F);	//mat.depth = bitm�lys�g

	Scalar means;   //Scalar is a class for a 4 variable vector. m is its instance.        
	Scalar stds;
	meanStdDev(Img , means , stds);

	Mat splitImg[3];
	split(Img, splitImg);

	//int channels = Img.channels();		//mat.channels = csatorn�k sz�ma
	int nRows = splitImg[0].rows;				//mat.rows = k�p sorainak sz�ma
	int nCols = splitImg[0].cols;				//mat.cols = k�p oszlopainak sz�ma. nCols: �sszes oszlop, az�rt kell mert ha nem egybef�gg� a m�trix, 
												//akkor soronk�nt kell lek�rni az adatokat, �s egy sorban ennyit kell majd menni.
	
	int i, j;
	float* ch0;
	float* ch1;
	float* ch2;
	float *p;
	for (i = 0; i < nRows; ++i)
	{
		ch0 = splitImg[0].ptr<float>(i);
		ch1 = splitImg[1].ptr<float>(i);
		ch2 = splitImg[2].ptr<float>(i);
		for (j = 0; j < nCols; j++)
		{
			ch0[j] -= means[0];
			ch0[j] /= stds[0];
			ch1[j] -= means[1];
			ch1[j] /= stds[1];
			ch2[j] -= means[2];
			ch2[j] /= stds[2];
		}
	}
	merge(splitImg, 3 , Img);

	return;
}*/

void ZeroMeanUnitDev(cv::Mat& Img)
{
	// accept only flaot type matrices
	CV_Assert(Img.depth() == CV_32F);	//mat.depth = bitm�lys�g

	Scalar means;   //Scalar is a class for a 4 variable vector. m is its instance.        
	Scalar stds;
	meanStdDev(Img, means, stds);

	int channels = Img.channels();		//mat.channels = csatorn�k sz�ma
	int nRows = Img.rows;				//mat.rows = k�p sorainak sz�ma
	int nCols = Img.cols * channels;	//mat.cols = k�p oszlopainak sz�ma. nCols: �sszes oszlop, az�rt kell mert ha nem egybef�gg� a m�trix, 
										//akkor soronk�nt kell lek�rni az adatokat, �s egy sorban ennyit kell majd menni.
	
	if (Img.isContinuous())
	{
		nCols *= nRows;
		nRows = 1;
	}
	
	int i, j;
	float *p;
	for (i = 0; i < nRows; ++i)
	{
		p = Img.ptr<float>(i);
		for (j = 0; j < nCols; j = j+3)
		{
			p[j] -= 0.5;
			p[j] /= 0.5;
			p[j + 1] -= 0.5;
			p[j + 1] /= 0.5;
			p[j + 2] -= 0.5;
			p[j + 2] /= 0.5;
			//p[j] -= means[0];
			//p[j] /= stds[0];
			//p[j + 1] -= means[1];
			//p[j+1] /= stds[1];
			//p[j + 2] -= means[2];
			//p[j+2] /= stds[2];
		}
	}

	return;
}

void mergeprobak()
{
	Mat ch[3];
	ch[0] = Mat::zeros(400, 400, CV_8UC1);	//b
	ch[1] = Mat::zeros(400, 400, CV_8UC1);	//g
	ch[2] = Mat::ones(400, 400, CV_32FC1);	//r

	for (int r = 0; r < ch[0].rows; r++)
	{
		for (int c = 0; c < ch[0].cols; c++)
		{
			ch[0].at<uint8_t>(c, r) = 255;
		}
	}
	imshow("only ch0", ch[0]);	//full feh�r lesz mert ch0-t 255-el t�lt�tt�k fel, de ez egycsatorn�s k�p, vagyis sz�rke�rnyalaots, vagyis csupa feh�r lesz
	imshow("only ch1", ch[1]);  //full fekete lesz mert ch1-et null�kkal t�lt�ttem fel, �s egycsatorn�sk�nt �rtelmezve ez fekete

	ch[2].convertTo(ch[2], CV_8UC1 , 255.0);	//floatr�l ucharra konvert�lom, mivel 1-esek voltak benne 255-�k lesznek, ami full feh�r
	imshow("only ch2", ch[2]);  

	Mat combined;
	merge(ch, 3, combined);
	imshow("combined", combined);	//A b �s az r csatorna van tele 255-el ez�rt pink k�pet kapunk
	cout << "Press 'q' to quit..." << endl;
	while (char(waitKey(1)) != 'q') {}
	return;
}

void datatype()
{
	Mat original = imread("F:/Projects/RobonAUT2/Dipterv/software/RPi/122820.jpg", CV_LOAD_IMAGE_COLOR);

	//Mat originalFloat;
	//original.convertTo(originalFloat, CV_32FC1, 1.0 / 255.0);
	//imshow("asd", originalFloat);

	//feh�r k�p l�trehoz�sa
	Mat img(28, 28, CV_8UC3, Scalar(255, 128, 80));
	imshow("white", img);
	img.convertTo(img, CV_32FC1, 1.0 / 255.0);


	//get ptr directly from Mat object
	cout << "get float*" << endl;
	float* ptr = img.ptr<float>(0);
	for (int i = 0; i < 28 * 3; i++)	//els� sor kiirat�sa
	{
		cout << ptr[i] << endl;
	}

	//edgybf�gg� float* gy�rt�sa a k�pre
	std::vector<float> array;
	if (img.isContinuous()) {
		array.assign((float*)img.datastart, (float*)img.dataend);
	}
	else {
		for (int i = 0; i < img.rows; ++i) {
			array.insert(array.end(), img.ptr<float>(i), img.ptr<float>(i)+img.cols);
		}
	}

	cout << "printing array:" << endl;
	for (int i = 0; i < 28 * 28 * 3; i++)
	{
		cout << array[i] << " ";
	}

	//reorder pixel from bgr to rgb
	std::vector<float> temp;
	for (int i = 0; i < array.size(); i += 3)
	{
		temp.push_back(array[i]);
		array[i] = array[i + 2];
	}
	for (int i = 0, j = 0; i < array.size(); i += 3 , j++)
	{
		array[i + 2] = temp[j];
	}

	cout << "printing array:" << endl;
	for (int i = 0; i < 28 * 28 * 3; i++)
	{
		cout << array[i] << " ";
	}



	cout << "Press 'q' to quit..." << endl;
	while (char(waitKey(1)) != 'q') {}
	return;
}
void vectorproba()
{
	std::vector<int> myVector;
	myVector.push_back(10);
	myVector.push_back(3);

	myVector.insert(myVector.begin(), 5);	//elej�re sz�r be
	myVector.insert(myVector.begin() + 1, 6);	//m�sodik helyre sz�r be

	for (int i = 0; i < myVector.size(); i++)
	{
		cout << myVector.at(i) << endl;
	}
	cout << "first element is: " << myVector.front() << endl;

	cout << "after erase:" << endl;
	myVector.erase(myVector.begin() + 1);	//kit�lri a 6-ost
	for (int i = 0; i < myVector.size(); i++)
	{
		cout << myVector[i] << endl;
	}

	if (myVector.empty())
		cout << "empty" << endl;
	else
		cout << "not empty" << endl;

	myVector.clear();
	if (myVector.empty())
		cout << "empty" << endl;
	else
		cout << "not empty" << endl;


	//vector.assign(m�ret, value): 
	myVector.assign(5, 100);	//els� vektort felt�lt�m 5 db 100-asal
	//m�sik vektort felt�ltj�k az els� elemeivel
	std::vector<int> myVector2;
	myVector2.assign(myVector.begin(), myVector.end());
	for (int i = 0; i < myVector2.size(); i++)
	{
		cout << myVector2[i] << endl;
	}

	getchar();
}


void splitmerge()
{
	Mat original = imread("F:/Projects/RobonAUT2/Dipterv/software/RPi/122820.jpg", CV_LOAD_IMAGE_COLOR);
	Mat modified = imread("F:/Projects/RobonAUT2/Dipterv/software/RPi/122820.jpg", CV_LOAD_IMAGE_COLOR);

	//split the 3 channels
	Mat splitChannels[3];

	split(original, splitChannels);

	//megjelen�t�s
	imshow("B", splitChannels[0]);
	imshow("G", splitChannels[1]);
	imshow("R", splitChannels[2]);

	splitChannels[2] = Mat::zeros(original.size() , CV_8UC1 );

	//3 csatorna mergel�se
	Mat output;
	merge(splitChannels, 3, output);
	imshow("merged", output);


	cout << "Press 'q' to quit..." << endl;
	while (char(waitKey(1)) != 'q') {}
	return;
}



void szinesproba()
{
	Mat img = imread("F:/Projects/RobonAUT2/Dipterv/software/RPi/122820.jpg", CV_LOAD_IMAGE_COLOR);
	Mat modified = imread("F:/Projects/RobonAUT2/Dipterv/software/RPi/122820.jpg", CV_LOAD_IMAGE_COLOR);
	//pixelek bej�r�sa
	for (int r = 0; r < modified.rows; r++){
		for (int c = 0; c < modified.cols; c++){
			modified.at<cv::Vec3b>(r, c)[0] = modified.at<cv::Vec3b>(r, c)[0] * 0;	//blue channel [0]
		}
	}
	for (int r = 0; r < modified.rows; r++){
		for (int c = 0; c < modified.cols; c++){
			modified.at<cv::Vec3b>(r, c)[1] = modified.at<cv::Vec3b>(r, c)[1] * 1;	//green channel [1]
		}
	}
	for (int r = 0; r < modified.rows; r++){
		for (int c = 0; c < modified.cols; c++){
			modified.at<cv::Vec3b>(r, c)[2] = modified.at<cv::Vec3b>(r, c)[2] * 1;	//red channel [2]
		}
	}

	//megjelen�t�s
	imshow("orig", img);
	imshow("modified", modified);


	cout << "Press 'q' to quit..." << endl;
	while (char(waitKey(1)) != 'q') {}
	return;
}


void grayscaleproba()
{

	//namedWindow("Hello"); 
	//imshow("Hello", img);

	Mat original = imread("F:/Projects/RobonAUT2/Dipterv/software/RPi/122820.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	Mat modified = imread("F:/Projects/RobonAUT2/Dipterv/software/RPi/122820.jpg", CV_LOAD_IMAGE_GRAYSCALE);

	//pixelek bej�r�sa
	for (int r = 0; r < modified.rows; r++){
		for (int c = 0; c < modified.cols; c++){
			modified.at<uint8_t>(r, c) = modified.at<uint8_t>(r, c) * 0.5f;
		}
	}

	//megjelen�t�s
	imshow("orig", original);
	imshow("modified", modified);

	cout << "Press 'q' to quit..." << endl;
	while (char(waitKey(1)) != 'q') {}
	return;
}
