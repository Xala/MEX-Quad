#include "ekz.h"

using namespace std;

vector<Matrix4f> lastTransformationMatrix;
vector<Matrix4f> transformationMatrix;
vector<double> X;
vector<double> Y;
vector<double> Z;


int main(int argc, char **argv){
	lastTransformationMatrix.push_back(Matrix4f::Identity());
    transformationMatrix = lastTransformationMatrix;
	printf("starting testing software2\n");
	printf("give path to files as input\n");
	string input = argv[1];
	
	string bow_path = argv[2];
	Map3D * m = new Map3D();	//Create a standard map object
	m->setVerbose(true);		//Set the map to give text output
	
	m->loadCalibrationWords(bow_path,"orb", 500);	//set bag of words to orb 500 orb features from bow_path
	m->setFeatureExtractor(new OrbExtractor());		//Use orb features
	int max_points = 300;							//Number of keypoints used by matcher
	int nr_iter = 10;								//Number of iterations the matcher will run
	float shrinking = 0.7;							//The rate of convergence for the matcher
	float bow_threshold = 0.15;						//Bag of words threshold to avoid investigating bad matches
	float distance_threshold = 0.015;				//Distance threshold to discard bad matches using euclidean information.
	float feature_threshold = 0.15;					//Feature threshold to discard bad matches using feature information.
	
	m->setMatcher(new BowAICK(max_points, nr_iter,shrinking,bow_threshold,distance_threshold,feature_threshold));//Create a new matcher

	int k;
	int l;
	int n;
	cout << "Please enter start image: ";
	cin >> k;
	cout << "Please enter end image: " ;
	cin >> l;
	cout << "Enter stepsize: ";
	cin >> n;
	bool hej = false;
	vector< RGBDFrame * > frames;
	for(int i = k; i <= l; i+= n){
		printf("----------------------%i-------------------\nadding a new frame\n",i);
		if (i == 336){
			i = 339;
		}
		//Get paths to image files
		char rgbbuf[512];
		char depthbuf[512];
		sprintf(rgbbuf,"%s/RGB%.10i.png",input.c_str(),i);
		sprintf(depthbuf,"%s/Depth%.10i.png",input.c_str(),i);
		//Add frame to map
		struct timeval start, end;
		gettimeofday(&start, NULL);
		m->addFrame(string(rgbbuf) , string(depthbuf));
		if (hej == true) {
			transformationMatrix = m->estimateCurrentPose(lastTransformationMatrix);
			gettimeofday(&end, NULL);
			float time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
			printf("Total AICK cost: %f\n",time);
			ofstream myfile11;
			myfile11.open ("totaltimes.txt", std::ios_base::app);  		
			myfile11 << time << endl;
			myfile11.close();
			lastTransformationMatrix = transformationMatrix;
			double x = transformationMatrix.front()(0,3);
			double y = transformationMatrix.front()(2,3);
			double z = -transformationMatrix.front()(1,3);
			cout << "X: " << x;
			cout << ", Y: " << y;
			cout << ", Z: " << z << endl;
			X.push_back(x);
			Y.push_back(y);
			Z.push_back(z);

		}
		hej = true;
	}
	
	vector<Matrix4f> poses = m->estimate();	//Estimate poses for the frames using the map object.
	m->savePCD("test.pcd");					//Saves a downsampled pointcloud with aligned data.
	
	ofstream myfileX;
	myfileX.open ("X.txt");  		
	for(unsigned int i = 0; i < X.size(); i++)
	{
		myfileX << X.at(i) << endl;
	}
	myfileX.close();

	ofstream myfileY;
	myfileY.open ("Y.txt");  		
	for(unsigned int i = 0; i < Y.size(); i++)
	{
		myfileY << Y.at(i) << endl;
	}
	myfileY.close();

	ofstream myfileZ;
	myfileZ.open ("Z.txt");  		
	for(unsigned int i = 0; i < Z.size(); i++)
	{
		myfileZ << Z.at(i) << endl;
	}
	myfileZ.close();
	//Print poses
	ofstream myfile;
	myfile.open ("poses.txt");  		
	myfile << "Poses:" << endl;
	for(unsigned int i = 0; i < poses.size(); i++)
	{
		myfile << poses.at(i) << endl << endl;
	}
	myfile.close();
	return 0;
}