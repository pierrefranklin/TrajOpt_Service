/*
 * demo_laser.cpp
 *
 *  Created on: Nov 18, 2014
 *      Author: Peng
 */

#include <openrave_env_generator/demo_laser.h>
using namespace TOService;

int main(){

	boost::shared_ptr <OpenRAVE::SensorBase::LaserSensorData> pdata(new OpenRAVE::SensorBase::LaserSensorData);

	OpenRAVE::RaveInitialize();

	boost::shared_ptr<EnvironmentGenerator> egPtr(new EGWall(false));

	OpenRAVE::EnvironmentBasePtr env = egPtr->getEnvironment();

//	env->Save("/home/peng/pcd_data/testSaveEnv.xml");

	env->Add(env->ReadRobotXMLFile("atlas_description/atlas_head_laser.xml"));



	// get all the sensors, this includes all attached robot sensors
	std::vector <OpenRAVE::SensorBasePtr> sensors;
	env->GetSensors(sensors);

	sensors[0]->Configure(OpenRAVE::SensorBase::CC_PowerOn);
	sensors[0]->Configure(OpenRAVE::SensorBase::CC_RenderDataOn);

	std::ofstream outputFile;
	outputFile.open("/home/peng/ScanedEnv.txt", std::ios::in | std::ios::app);

//	for (int kk = 0; kk < 100; ++kk)
		for (int kk = 0; kk < 100; ++kk)
	{
		std::cout << kk << '\n';
		sensors[0]->SimulationStep(0.1);
		sensors[0]->GetSensorData(pdata);
		double x_center = pdata->positions[0][0];
		double y_center = pdata->positions[0][1];
		double z_center = pdata->positions[0][2];


		for (int ii = 0; ii < pdata->ranges.size(); ++ii)
		{
//			outputFile << std::sqrt(pdata->ranges[ii][0]^2 + pdata->ranges[ii][1]^2 + pdata->ranges[ii][2]^2) << ' ';
			outputFile << TOService::distance(x_center,y_center,z_center, pdata->ranges[ii][0],pdata->ranges[ii][1],pdata->ranges[ii][2])<< ' ';

			outputFile << pdata->intensity[ii] << ' ';
		}



		//std::cin>>a;

	}

	outputFile << "\n";
	outputFile.close();

	std::cout << "One Environment scaned and saved! \n";



//	boost::this_thread::sleep(boost::posix_time::seconds(5));

	OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(env, "qtcoin");

	assert(viewer);

	env->AddViewer(viewer);

	while(1){
		viewer->main();
	}

	return 0;

}


