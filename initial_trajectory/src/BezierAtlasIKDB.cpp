/*
 * BezierConstantDB.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: perry
 */

#include <initial_trajectory/BezierAtlasIKDB.h>

#include <ros/package.h>

namespace TOService {


std::vector<double> eigenToDouble(Eigen::Vector3d input){

	std::vector<double> returnMe;

	returnMe.assign(input.data(), input.data()+3);

	return returnMe;

}


void getParamCrap(SFIkCon& ik){
	std::string dir = ros::package::getPath("atlas_controllers");
    std::string ik_path = dir+std::string("/cmu_config/con_param/atlas_bal_ikCon.conf");
    bool ret;
    std::ifstream in;
    in.open(ik_path.c_str());
    if (in.is_open()) {
      ret &= ik.readParams(in);
      in.close();
    }
    else{
  	    std::cout<<"don't expect IK to work"<<std::endl;
        return;
    }

}

BezierAtlasIKDB::BezierAtlasIKDB():
		BQG(new BezierQuadraticGenerator){
	ik.init(rs);
	getParamCrap(ik);
	ik_d.init();
	BQG->num_waypoints = 15;

	gridSize = 5;
	gridScale = 0.3;
}

BezierAtlasIKDB::~BezierAtlasIKDB() {
	// TODO Auto-generated destructor stub
}

Eigen::MatrixXf BezierAtlasIKDB::getTrajectory(int index, std::vector<double> start, Eigen::Vector3d end){

	//Set the robotstate to the start position
	rs.fillZeros();
	rs.joints = Eigen::Map<Vec28>(start.data());
	rs.computeSDFvars();

	ik.setToRobotState(rs);
	ik_d.setToRS(rs);


	//Get hand start and end
	Eigen::Vector3d handstart = ik.ikrs->gripper[1].pose.pos;
	Eigen::Vector3d handend = rs.tfLeftFoot2sdfWorld(Pose(end,{0,0,0,1})).pos;
	Eigen::Vector3d handmiddle = calculateMiddle(index, handstart, handend);

	ik_d.handMode[1] = 1;
	ik_d.rotGain[1] = {0,0,0};

	//Using the hand points, calculate a bezier curve (waypoints)
	Eigen::MatrixXf taskTraj = BQG->generateGuess(eigenToDouble(handstart), eigenToDouble(handend), eigenToDouble(handmiddle));



	Eigen::MatrixXf trajectory(taskTraj.rows(),28);

	trajectory.row(0) = ik.ikrs->joints.transpose().cast<float>();

	for( int row = 1; row < taskTraj.rows(); row++){

		ik_d.hand[1].pos = taskTraj.row(row).transpose().cast<double>();
		for(int i = 0; i < 400; i++){

			Command c;
			ik.IK(rs, ik_d, c);

		}

		trajectory.row(row) = ik.ikrs->joints.transpose().cast<float>();
	}
	return trajectory;

}

void BezierAtlasIKDB::setGridSize(int gridSize_){
	gridSize = gridSize_;
}

void BezierAtlasIKDB::setGridScale(double gridScale_){
	gridScale = gridScale_;
}

int BezierAtlasIKDB::getNumIndices(){
	return gridSize*gridSize;
}

boost::shared_ptr<BezierQuadraticGenerator> BezierAtlasIKDB::getTrajGenerator(){
	return BQG;
}

Eigen::Vector3d BezierAtlasIKDB::calculateMiddle(int index, Eigen::Vector3d start, Eigen::Vector3d end){

	//Given the vector from start to end, find a plane that bisects it
	//The index will determine a point on that plane

	Eigen::Vector3d normal = (end-start);

	//midpoint and 2 vectors define a plane
	Eigen::Vector3d midpoint = end/2 + start/2;

	Eigen::Vector3d basisX = Eigen::Vector3d::UnitZ().cross(normal).normalized();

	Eigen::Vector3d basisY = normal.cross(basisX).normalized();

	//The index is converted into a x y position on the plane
	int x = (index%gridSize) - (gridSize/2);
	int y = (index%gridSize) - (gridSize/2);


	midpoint = midpoint
			   + gridScale * ((double) x) * basisX
			   + gridScale * ((double) y) * basisY;

	return midpoint;




}


} /* namespace TOService */
