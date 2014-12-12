/*
 * BuildDatabase.cpp
 *
 *  Created on: Dec 11, 2014
 *      Author: perry
 */


#include <training_data_generator/DatabaseTrajectorySearch.h>
#include <openrave_env_generator/EGTable.h>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <ros/ros.h>
#include <ros/package.h>

std::vector<double> start = {0, 0, 0, 0,
							0, 0, -0.23, 0.52, -0.275, -0.06,
							-0.0, -0.064, -0.23, 0.52, -0.275, 0.06,
							0.3, -1.3, 2.0, 0.5, -0,-0.0028,
							0.3, 1.3,	2.0, -0.5, 0.5, 0.0028};

using namespace TOService;

int main(int argc, char *argv[]){

	ros::init(argc, argv, "databaseMaker");
	ros::NodeHandle nh;

	boost::shared_ptr<BezierAtlasIKDB> db(new BezierAtlasIKDB);

	DatabaseTrajectorySearch dts(db);
	EGTable envGenerafdstor(false,4);

	std::string folder_path = ros::package::getPath("training_data_generator") + "/test_envs/";

	int currentIndex = 0;

	for( int numEnv = 0; numEnv < 1000; numEnv++){

		if (!ros::ok()){
			break;
		}
		EGTable envGenerator(false,2);

		OpenRAVE::EnvironmentBasePtr env = envGenerator.getEnvironment();
		env->Load("atlas_description/atlas_foot.xml");

		OpenRAVE::RobotBasePtr robot = env->GetRobot("atlas");
		env->StopSimulation();

		std::vector<int> activejoint = {robot->GetJoint("back_bkz")->GetDOFIndex(),robot->GetJoint("back_bky")->GetDOFIndex(),
									   robot->GetJoint("back_bkx")->GetDOFIndex(),robot->GetJoint("neck_ay")->GetDOFIndex(),
									   robot->GetJoint("l_leg_hpz")->GetDOFIndex(),robot->GetJoint("l_leg_hpx")->GetDOFIndex(),
									   robot->GetJoint("l_leg_hpy")->GetDOFIndex(),robot->GetJoint("l_leg_kny")->GetDOFIndex(),
									   robot->GetJoint("l_leg_aky")->GetDOFIndex(),robot->GetJoint("l_leg_akx")->GetDOFIndex(),
									   robot->GetJoint("r_leg_hpz")->GetDOFIndex(),robot->GetJoint("r_leg_hpx")->GetDOFIndex(),
									   robot->GetJoint("r_leg_hpy")->GetDOFIndex(),robot->GetJoint("r_leg_kny")->GetDOFIndex(),
									   robot->GetJoint("r_leg_aky")->GetDOFIndex(),robot->GetJoint("r_leg_akx")->GetDOFIndex(),
									   robot->GetJoint("l_arm_shy")->GetDOFIndex(),robot->GetJoint("l_arm_shx")->GetDOFIndex(),
									   robot->GetJoint("l_arm_ely")->GetDOFIndex(),robot->GetJoint("l_arm_elx")->GetDOFIndex(),
									   robot->GetJoint("l_arm_wry")->GetDOFIndex(),robot->GetJoint("l_arm_wrx")->GetDOFIndex(),
									   robot->GetJoint("r_arm_shy")->GetDOFIndex(),robot->GetJoint("r_arm_shx")->GetDOFIndex(),
									   robot->GetJoint("r_arm_ely")->GetDOFIndex(),robot->GetJoint("r_arm_elx")->GetDOFIndex(),
									   robot->GetJoint("r_arm_wry")->GetDOFIndex(),robot->GetJoint("r_arm_wrx")->GetDOFIndex()};
		robot->SetDOFValues(start, true, activejoint);
		robot->GetDOFValues(start,activejoint);

		OpenRAVE::KinBodyPtr table= env->GetKinBody("table");

		OpenRAVE::Transform t = table->GetTransform();
		OpenRAVE::Vector v = t*OpenRAVE::Vector(0,0,0.15);

		int x= dts.findBestIndex(env, {v.x,v.y,v.z});
		std::cout<<x<<", current envNum is "<<numEnv<<std::endl;

		if( x != -1){
			std::stringstream envfilename;
			envfilename << std::setfill('0') << std::setw(5) << currentIndex << "t.zae";
			env->Save(folder_path + envfilename.str(), OpenRAVE::EnvironmentBase::SelectionOptions::SO_NoRobots);

			std::stringstream resultfilename;
			resultfilename << std::setfill('0') << std::setw(5) << currentIndex << "_result";

			std::ofstream filestream;
			filestream.open(folder_path + resultfilename.str(), std::ofstream::out | std::ofstream::app);
			if(!filestream.is_open()){
				std::cout<<"Failed to open file '"<<resultfilename<<"'"<<std::endl;
				return 0;
			}
			//writeHeader(filestream);
			filestream << x <<std::endl;
			filestream.close();


			currentIndex++;
		}

	}

	std::cout<<"DONE"<<std::endl;

	return 0;



}




