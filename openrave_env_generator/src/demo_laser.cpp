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


using namespace TOService;

int main(){

	boost::shared_ptr <SensorBase::LaserSensorData> pdata;

	OpenRAVE::RaveInitialize();

	boost::shared_ptr<EnvironmentGenerator> egPtr(new EGWall(false));

	OpenRAVE::EnvironmentBasePtr env = egPtr->getEnvironment();

	env->Add(env->ReadRobotXMLFile("atlas_description/atlas_head_laser.xml"));

	// get all the sensors, this includes all attached robot sensors
	std::vector <OpenRAVE::SensorBasePtr> sensors;
	env->GetSensors(sensors);

	sensors[0]->Configure(OpenRAVE::SensorBase::CC_PowerOn);
	sensors[0]->Configure(OpenRAVE::SensorBase::CC_RenderDataOn);

	sensors[0]->GetSensorData(pdata);

	std::cout << pdata.positons << '\n';


//	boost::this_thread::sleep(boost::posix_time::seconds(5));

	OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(env, "qtcoin");

	assert(viewer);

	env->AddViewer(viewer);

	while(1){
		viewer->main();
	}

	return 0;

}


