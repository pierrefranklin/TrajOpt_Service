/*
 * demo_laser.cpp
 *
 *  Created on: Nov 18, 2014
 *      Author: Peng
 */

#include <openrave_env_generator/SaveLaserData.h>
using namespace TOService;

int main(){


	OpenRAVE::RaveInitialize();

	boost::shared_ptr<EnvironmentGenerator> egPtr(new EGWall(false));

	OpenRAVE::EnvironmentBasePtr env = egPtr->getEnvironment();

//	env->Save("/home/peng/pcd_data/testSaveEnv.xml");

	env->Add(env->ReadRobotXMLFile("atlas_description/atlas_head_laser.xml"));

	/* This is how to save sensor data, now only support a single laser sensor
	 * In order to have the data store correctly, please change the user name in path "/home/peng/ScanedEnv.txt"
	 * in file "SaveLaserData.cpp" .
	 *
	 * Data form:
	 * each environment will be save as 100 points with (double distance, int hit) in one line of the file;
	 * each run of the mdata.save() will attach one line in ScanedEnv.txt
	 */

	std::vector <OpenRAVE::SensorBasePtr> sensors;
	env->GetSensors(sensors);
	TOService::SaveLaserData mdata(sensors);
	mdata.save();


	OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(env, "qtcoin");

	assert(viewer);

	env->AddViewer(viewer);

	while(1){
		viewer->main();
	}

	return 0;

}


