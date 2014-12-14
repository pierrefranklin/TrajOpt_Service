/*
 * testDB.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: perry
 */

#include <initial_trajectory/BezierAtlasIKDB.h>

#include <openrave/openrave.h>
#include <openrave-0.9/openrave-core.h>

#include <thread>

using namespace TOService;

std::vector<double> eigenToDouble(Eigen::VectorXf input){

	std::vector<double> returnMe;

	for(int i = 0; i < input.size(); i++){
		returnMe.push_back(input[i]);
	}
	return returnMe;

}

void SetViewer(OpenRAVE::EnvironmentBasePtr penv, const std::string& viewername)
{
    OpenRAVE::ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->AddViewer(viewer);

    // finally you call the viewer's infinite loop (this is why you need a separate thread):
    bool showgui = true;
    viewer->main(showgui);

}

int main(int argc, char *argv[]){

	std::vector<double> start = {0, 0, 0, 0,
			0, 0, -0.23, 0.52, -0.275, -0.06,
			-0.0, -0.064, -0.23, 0.52, -0.275, 0.06,
			0.3, -1.3, 2.0, 0.5, -0,-0.0028,
			0.3, 1.3,	2.0, -0.5, 0.5, 0.0028};

	BezierAtlasIKDB database;
	int index = 0;
	if(argc >1 ){
		index = std::stoi(argv[1]);
	}

	auto trajectory = database.getTrajectory(index,start, {.75,-0.5,0.7});
	std::cout << trajectory <<std::endl;

	OpenRAVE::RaveInitialize();


	OpenRAVE::EnvironmentBasePtr env = OpenRAVE::RaveCreateEnvironment();

	env->Add(env->ReadRobotXMLFile("atlas_description/atlas_foot.xml"));

	OpenRAVE::RobotBasePtr robot = env->GetRobot("atlas");

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

	boost::thread thviewer(boost::bind(SetViewer,env,"qtcoin"));

	while(1){
		for(int i = 0; i < trajectory.rows(); i++){
		robot->SetDOFValues(eigenToDouble(trajectory.row(i).transpose()),1,activejoint);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}

	}

	return 0;

}


