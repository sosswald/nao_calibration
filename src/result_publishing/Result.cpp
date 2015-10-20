#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}
int main(int argc, char** argv) {
	
	string str;	std::string token;std::string token2;std::vector <std::string> fields;std::vector <std::string> fields2;std::vector <std::string> fields3;
	ifstream infile;	
	cv::Mat cam(3, 3, DataType<double>::type);
	cv::Mat translation(3, 3, DataType<double>::type); 	
	cv::Mat rotation(3, 1, DataType<double>::type);
	cv::Mat sub_output(3, 1, DataType<double>::type);
	cv::Mat output_f(3, 3, DataType<double>::type);


	infile.open ("/home/kurt/catkin_ws/src/nao_calibration-master/config/marker_transformation.txt");
	getline(infile,str); // Saves the line in STRING.
	
	fields=split(str, ',');//str.substr(0, str.find(","));
	for (int i=0;i<3;i++){
		cam.at<double>(0,i)= std::stod(fields[i]); // a12 ele of camera
		std::cout<<fields[i]<<std::endl;
	}

	for (int i=0;i<3;i++){
		cam.at<double>(1,i)= std::stod(fields[i+3]); // a12 ele of camera
		std::cout<<fields[i+3]<<std::endl;
	}
	for (int i=0;i<3;i++){
		cam.at<double>(2,i)= std::stod(fields[i+6]); // a12 ele of camera
		std::cout<<fields[i+6]<<std::endl;		
	}

	getline(infile,str); // Saves the line in STRING.
	fields2=split(str, ',');
	for (int i=0;i<3;i++)
		translation.at<double>(0,i)= std::stod(fields2[i]); // a12 ele of camera

	getline(infile,str); // Saves the line in STRING.
	fields3=split(str, ',');
	for (int i=0;i<3;i++)
		rotation.at<double>(i,0)= std::stod(fields3[i]); // a12 ele of camera

	infile.close();

	sub_output = rotation.mul(cam);
	//output_f=sub_output.mul(translation);
/*
	ofstream onfile;std::string out=argv[1];
	onfile.open ("/home/kurt/catkin_ws/"+out+".txt");
	for (int i=0;i<3;i++){
		for (int j=0;j<3;j++){
			onfile<< output_f.at<double>(i,j) <<" ";
		}
		onfile<<"\n";
	}
	
	onfile.close();*/
	return 0;
}

