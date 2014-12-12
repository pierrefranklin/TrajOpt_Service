/*
 * demo.cpp
 *
 *  Created on: Nov 18, 2014
 *      Author: perry
 */

#include <openrave_env_generator/EnvironmentGenerator.h>
#include <openrave_env_generator/EGLoader.h>
#include <openrave_env_generator/EGTable.h>
#include <openrave_env_generator/EGWall.h>
#include <openrave/sensor.h>
#include <iostream>


using namespace TOService;

int main(){

	OpenRAVE::RaveInitialize();

	boost::shared_ptr<EnvironmentGenerator> egPtr(new EGWall(false));

	OpenRAVE::EnvironmentBasePtr env = egPtr->getEnvironment();

	env->Add(env->ReadRobotXMLFile("atlas_description/atlas_foot_service.xml"));

	std::vector<boost::shared_ptr<OpenRAVE::RobotBase> > robots;

	OpenRAVE::EnvironmentBase::GetRobots(robots, 30);

	std::vector<OpenRAVE::RobotBase::AttachedSensorPtr> laser = robot[0]->GetAttachedSensors();

	laser[0]->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_PowerOn);

	laser[0]->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_RenderDataOn);


	OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(env, "qtcoin");

	assert(viewer);

	env->AddViewer(viewer);

	while(1){
		viewer->main();
	}

	return 0;

}


