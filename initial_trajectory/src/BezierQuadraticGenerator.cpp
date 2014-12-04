/*
 * BezierQuadraticGenerator.cpp
 *
 *  Created on: Dec 2, 2014
 *      Author: perry
 */

#include <initial_trajectory/BezierQuadraticGenerator.h>
#include <iostream>

namespace TOService {

BezierQuadraticGenerator::BezierQuadraticGenerator() {

}

BezierQuadraticGenerator::~BezierQuadraticGenerator() {

}

Eigen::MatrixXf BezierQuadraticGenerator::generateGuess(std::vector<double> start, std::vector<double> end, std::vector<double> parameters){

	Eigen::Map<Eigen::VectorXd> startVec(start.data(), start.size());
	Eigen::Map<Eigen::VectorXd> endVec(end.data(), end.size());
	Eigen::Map<Eigen::VectorXd> midVec(parameters.data(), parameters.size());

	//Error Checking Crap
//	if (startVec.size() != endVec.size() || startVec.size() != midVec.size()){
//		std::cout<< "Dimensions for generateGuess are incorrect"<<std::endl;
//		std::cout<< "Size of start = " << startVec.size();
//		std::cout<< ", Size of mid = " << midVec.size();
//		std::cout<< ", Size of end = " << endVec.size();
//		std::cout<<std::endl;
//	}

	Eigen::MatrixXf trajectory(num_waypoints,start.size());

	for( int row = 0; row < num_waypoints; row++){

		double t = (double) row / (double) (num_waypoints-1);

		//Bezier curves are defined as
		// B(t) = (1-t)^2*P0 + 2*(1-t)*t*P1 +

		Eigen::VectorXd waypoint = ((1-t)*(1-t)*startVec + 2*(1-t)*t*midVec + t*t*endVec);

		trajectory.row(row) = waypoint.cast<float>().transpose();

	}

	return trajectory;

}


} /* namespace TOService */
