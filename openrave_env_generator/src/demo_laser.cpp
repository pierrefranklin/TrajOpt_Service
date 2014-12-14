/*
 * demo_laser.cpp
 *
 *  Created on: Nov 18, 2014
 *      Author: Peng
 */

#include <openrave_env_generator/EnvironmentGenerator.h>
#include <openrave_env_generator/EGLoader.h>
#include <openrave_env_generator/EGTable.h>
#include <openrave_env_generator/EGWall.h>
#include <openrave/sensor.h>
#include <iostream>
#include <openrave-core.h>
#include <vector>
#include <fstream>



using namespace TOService;

int main(){

	boost::shared_ptr <OpenRAVE::SensorBase::LaserSensorData> pdata(new OpenRAVE::SensorBase::LaserSensorData);

	OpenRAVE::RaveInitialize();

	boost::shared_ptr<EnvironmentGenerator> egPtr(new EGWall(false));

	OpenRAVE::EnvironmentBasePtr env = egPtr->getEnvironment();

	env->Save("/home/peng/pcd_data/testSaveEnv.xml");

	env->Add(env->ReadRobotXMLFile("atlas_description/atlas_head_laser.xml"));



	// get all the sensors, this includes all attached robot sensors
	std::vector <OpenRAVE::SensorBasePtr> sensors;
	env->GetSensors(sensors);

	sensors[0]->Configure(OpenRAVE::SensorBase::CC_PowerOn);
	sensors[0]->Configure(OpenRAVE::SensorBase::CC_RenderDataOn);

	std::ofstream outputFile;
	outputFile.open("/home/peng/pcd_data/ScanedEnv.txt");

	for (int kk = 0; kk < 100; ++kk)
	{
		std::cout << kk << '\n';
		sensors[0]->SimulationStep(0.01);
		sensors[0]->GetSensorData(pdata);
		//std::cout << pdata->positions[kk];
		std::cout<<" Size =  "<<pdata->intensity.size()<<std::endl;
		std::cout<<" Size =  "<<pdata->ranges.size()<<std::endl;
//		std::cin.get();
//		for (int ii = 0; ii < pdata->positions.size(); ++ii)
//		{
//			std::cout << pdata->positions[ii] << '\n';
//		}
		for (int ii = 0; ii < pdata->ranges.size(); ++ii)
		{
//			outputFile << pdata->ranges[ii] << ' ';

			outputFile << pdata->intensity[ii] << '\n';
		}



		//std::cin>>a;

	}

	outputFile.close();
	std::cout << "Done!\n";




//	boost::this_thread::sleep(boost::posix_time::seconds(5));

	OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(env, "qtcoin");

	assert(viewer);

	env->AddViewer(viewer);

	while(1){
		viewer->main();
	}

	return 0;

}


