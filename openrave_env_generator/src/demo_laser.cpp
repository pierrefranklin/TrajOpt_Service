/*
 * demo_laser.cpp
 *
 *  Created on: Nov 18, 2014
 *      Author: Peng
 */

#include <openrave_env_generator/SaveLaserData.h>
using namespace TOService;

int main(){

	long currentIndex = 0;

	OpenRAVE::RaveInitialize();

	// boost::shared_ptr<EnvironmentGenerator> egPtr(new EGWall(false));

	// OpenRAVE::EnvironmentBasePtr env = egPtr->getEnvironment();

//	env->Save("/home/peng/pcd_data/testSaveEnv.xml");

	EGTable envGenerator(false,2);

	char tj_result;

	std::vector <OpenRAVE::SensorBasePtr> sensors;
	TOService::SaveLaserData mdata;
	std::string folder_path = "/home/peng/env_data/";
	// std::string folder_path = ros::package::getPath("training_data_generator") + "/test_envs/";

	for( int numEnv = 0; numEnv < 2000 ; numEnv++){

			OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();

			std::stringstream envfilename;
			envfilename << std::setfill('0') << std::setw(5) << currentIndex << "t.zae";
			
			env->Load(folder_path + envfilename.str());
			env->Add(env->ReadRobotXMLFile("atlas_description/atlas_head_laser.xml"));
			
			std::stringstream resultfilename;
			resultfilename << std::setfill('0') << std::setw(5) << currentIndex << "_result";

			std::ifstream filestream;
			filestream.open(folder_path + resultfilename.str());
			
			if(!filestream.is_open()){
				std::cout<<"Failed to open file '"<<resultfilename<<"'"<<std::endl;
				exit(-1);
			}
			
			filestream.get(tj_result);

			filestream.close();
			currentIndex++;
		/* This is how to save sensor data, now only support a single laser sensor
		 * In order to have the data store correctly, please change the user name in path "/home/peng/ScanedEnv.txt"
		 * in file "SaveLaserData.cpp" .
		 *
		 * Data form:
		 * each environment will be save as 100 points with (double distance, int hit) in one line of the file;
		 * each run of the mdata.save() will attach one line in ScanedEnv.txt
		 */

		env->GetSensors(sensors);
		mdata.getSensors(sensors);
		mdata.save(tj_result);



		env->GetRobot("atlas")->Destroy();
		env->Destroy();
	}





	

	// OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(env, "qtcoin");

	// assert(viewer);

	// env->AddViewer(viewer);

	// while(1){
	// 	viewer->main();
	// }
	std::cout << "Saving Finished\n";

	return 0;

}


