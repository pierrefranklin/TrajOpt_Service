/*
 * testDTS.cpp
 *
 *  Created on: Dec 9, 2014
 *      Author: perry
 */


#include <training_data_generator/DatabaseTrajectorySearch.h>

std::vector<double> start = {0, 0, 0, 0,
							0, 0, -0.23, 0.52, -0.275, -0.06,
							-0.0, -0.064, -0.23, 0.52, -0.275, 0.06,
							0.3, -1.3, 2.0, 0.5, -0,-0.0028,
							0.3, 1.3,	2.0, -0.5, 0.5, 0.0028};

using namespace TOService;

int main(){



	boost::shared_ptr<BezierAtlasIKDB> db(new BezierAtlasIKDB);

	DatabaseTrajectorySearch dts(db);

	OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();
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

	std::cout<<dts.findBestIndex(env, {0.5,-0.5,1})<<std::endl;

	std::cout<<"DONE"<<std::endl;

	while(1){
	}

	return 0;



}

