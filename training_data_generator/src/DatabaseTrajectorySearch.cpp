/*
 * DatabaseTrajectorySearch.cpp
 *
 *  Created on: Dec 8, 2014
 *      Author: perry
 */

#include <training_data_generator/DatabaseTrajectorySearch.h>
#include <trajopt/problem_description.hpp>
#include <drc_traj/atlas_utils.hpp>
#include <ros/ros.h>


namespace TOService {

DatabaseTrajectorySearch::DatabaseTrajectorySearch(boost::shared_ptr<BezierAtlasIKDB> database_):
	database(database_){
	TOHak.LoadGains({1,1,1},{0,0,0});
	TOHak.see_viewer=false;
}

DatabaseTrajectorySearch::~DatabaseTrajectorySearch() {
	// TODO Auto-generated destructor stub
}

int DatabaseTrajectorySearch::findBestIndex(OpenRAVE::EnvironmentBaseConstPtr env, Eigen::Vector3d end){
	int numIndices = database->getNumIndices();

	int numWaypoints = database->getTrajGenerator()->num_waypoints;

	int bestIndex = -1;
	trajopt::TrajOptResultPtr bestResult;

	ros::Duration bestTime(100.0);

	std::vector<double> start_state;
	env->GetRobot("atlas")->GetDOFValues(start_state, TOHak.Getactivejoint(TOHak.current_mode));

	Eigen::Affine3d goal = Eigen::Affine3d::Identity();
	goal.translation() = end;

	for (int i = 0; i <  numIndices; i++){

		TOHak.LoadGains({1,1,1},{0,0,0},{0,0.28,0.0});

		std::vector<double> temp_start_state(start_state);
		std::cout<<"On pass "<<i<<std::endl;

		Eigen::MatrixXf initguess = database->getTrajectory(i, temp_start_state, end);

		ros::Time begin = ros::Time::now();

		trajopt::TrajOptResultPtr curResult = computeTrajectory(temp_start_state, goal, ::Side::RIGHT, initguess);

		ros::Duration curTime = ros::Time::now() - begin;

		if (traj_is_safe(curResult->traj, TOHak.robot)){
			continue;
		}

		ROS_INFO_STREAM("TIME = "<<curTime);

		if (!bestResult){
			bestIndex = i;
			bestResult = curResult;
			bestTime = curTime;
			std::cout<<"FIRST RESULT"<<std::endl;
			continue;
		}
		else if (curTime<bestTime){
			bestIndex = i;
			bestResult = curResult;
			bestTime = curTime;
			ROS_INFO_STREAM("UPDATED "<<bestIndex);
			continue;
		}

	}

	return bestIndex;
}


trajopt::TrajOptResultPtr DatabaseTrajectorySearch::computeTrajectory(std::vector<double> start_state,  Eigen::Affine3d hand, ::Side side, Eigen::MatrixXf initguess){
	std::cout << "Computing trajectory" << std::endl;

	for(auto element: start_state){
		std::cout<<element<<" ";
	}
	std::cout<<std::endl;
	OpenRAVE::EnvironmentMutex::scoped_lock lockenv(TOHak.env->GetMutex());
	//Handle viewer:
	if(TOHak.see_viewer) TOHak.viewer = OSGViewer::GetOrCreate(TOHak.env);
	if(TOHak.see_viewer && TOHak.idle_viewer)TOHak.viewer->Idle();


	std::vector<int> activejoint = TOHak.Getactivejoint(TOHak.current_mode);
	int current_active_dof = TOHak.robot->GetActiveDOF();

	UpdateStateForRequest(start_state, hand, side, TOHak.current_mode);

	if(TOHak.multi_init_guess && TOHak.viewer)TOHak.ClearViewer();

	std::stringstream request;

	request.str(std::string()); // Clear the request

	int numJoint = TOHak.robot->GetDOF();

	Eigen::MatrixXf whole_trajectory = Eigen::MatrixXf::Zero(initguess.rows(),numJoint);

	for(int row = 0; row < initguess.rows(); row++){
		for(int col = 0; col<initguess.cols();col++){
			whole_trajectory(row,activejoint[col]) = initguess(row,col);
		}
	}
	TOHak.request_traj = whole_trajectory;

	std::cout<< "Number of rows: " << TOHak.request_traj.rows() << "Number of col: " << TOHak.request_traj.cols() <<std::endl;


	TOHak.ComposeRequest(request, TOHak.current_mode);

	int target_tread = 0;

	trajopt::TrajArray traj;


/*		for(int i=0;i<Num_threads;i++){
			pthread_join(threads[i],NULL);
		}*/

	// Parsing:
	Json::Value root;
	Json::Reader reader;
	bool parsedSuccess = reader.parse(request,
									 root,
									 false);

	if(not parsedSuccess)
	{
		std::cerr<<"Failed to parse JSON"<<std::endl
		<<reader.getFormatedErrorMessages()
		<<std::endl;
	}

	if(TOHak.see_viewer)
	{
		TOHak.viewer->UpdateSceneData();
		TOHak.viewer->Draw();
	}

	trajopt::TrajOptProbPtr prob = trajopt::ConstructProblem(root, TOHak.env);

	trajopt::TrajOptResultPtr result = trajopt::OptimizeProblem(prob, TOHak.viewer);

	int not_safe = traj_is_safe(result->traj,TOHak.robot);

	std::cout<< "Number of collisions: " << not_safe <<std::endl;

	TOHak.PrintTraj(traj,activejoint);

	if(TOHak.see_viewer){
		TOHak.ShowTraj(traj);
	}

	return result;

}

void DatabaseTrajectorySearch::UpdateStateForRequest(std::vector<double>& start_state, Eigen::Affine3d hand, ::Side side, atlas_traj::TrajoptMode mode){

	std::vector<int> activejoint = TOHak.Getactivejoint(mode);

	TOHak.robot = TOHak.env->GetRobot("atlas");

	TOHak.robot->SetDOFValues(start_state, 1, activejoint);
	TOHak.robot->GetDOFValues(start_state,activejoint);

	int current_active_dof = TOHak.robot->GetDOF();

	std::cout<<"  "<<current_active_dof<<std::endl;

	TOHak.robot->SetActiveDOFs(vector_arange(current_active_dof));

	Eigen::Vector3d hand_pos = hand.translation();
	Eigen::Quaterniond hand_q = Eigen::Quaterniond(hand.linear());

	TOHak.xyz_target = {hand_pos.x(),hand_pos.y(),hand_pos.z()};//{0.82, 0.21, 1.23373}
	TOHak.quat_target = {hand_q.w(), hand_q.x(), hand_q.y(), hand_q.z()};
	TOHak.SetVelCost(TOHak.vel_cost,mode);
	TOHak.SetPosCost(TOHak.pos_cost,TOHak.pos_vals,mode);
	TOHak.GetLinkPosandQuat("l_foot", TOHak.l_foot_xyz, TOHak.l_foot_quat);
	TOHak.GetLinkPosandQuat("r_foot", TOHak.r_foot_xyz, TOHak.r_foot_quat);

	if(side == ::Side::RIGHT){
		//The Right hand Y-axis is negative from the Left
		TOHak.hand_offset[1] = -TOHak.hand_offset[1];
		TOHak.hand_str = "r_hand";
		TOHak.other_hand_str = "l_hand";
		TOHak.GetLinkPosandQuat("l_hand",TOHak.other_hand_xyz_target,TOHak.other_hand_quat_target);
	}else{
		TOHak.hand_str = "l_hand";
		TOHak.other_hand_str = "r_hand";
		TOHak.GetLinkPosandQuat("r_hand",TOHak.other_hand_xyz_target,TOHak.other_hand_quat_target);
	}


}




} /* namespace TOService */
