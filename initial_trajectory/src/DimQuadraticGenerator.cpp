/*
 * DimQuadraticGenerator.cpp
 *
 *  Created on: Dec 1, 2014
 *      Author: perry
 */

#include <initial_trajectory/DimQuadraticGenerator.h>
#include <iostream>

namespace TOService {

DimQuadraticGenerator::DimQuadraticGenerator() {
	// TODO Auto-generated constructor stub

}

DimQuadraticGenerator::~DimQuadraticGenerator() {
	// TODO Auto-generated destructor stub
}

Eigen::MatrixXf DimQuadraticGenerator::generateGuess(std::vector<double> start, std::vector<double> end, std::vector<double> parameters){

	std::size_t dimensionality = start.size();

	Eigen::MatrixXf trajectory(num_waypoints, dimensionality);

	if(dimensionality != end.size()){
		std::cerr<<"Target configuration is not the same dimensionality as start configuration"<<std::endl;
		std::cerr<<"Start D = "<<dimensionality<<", target D = "<<end.size()<<std::endl;
	}
	if(dimensionality != parameters.size()){
		std::cerr<<"parameters is not the same dimensionality as start configuration"<<std::endl;
		std::cerr<<"Start D = "<<dimensionality<<", target D = "<<parameters.size()<<std::endl;
	}

	for(int column = 0; column < dimensionality; column++){

		//For each dimension, we're going to create a quadratic trajectory
		//a*x^2 + b*x + c
		// c = startPoint
		// a = parameter
		// b = endPoint - startPoint - a*1^2
		// This is valid for  0 <= x <= 1 where 1 is the endpoint

		double & a = parameters[column];
		double & c = start[column];
		double b = end[column] - c - a;

		//And now use this to calculate waypoints

		for(int row = 0; row < num_waypoints; row++){
			double t = (double) row/ (double) (num_waypoints-1);
			trajectory(row, column) = a*pow(t,2) + b*t + c;
		}
	}

	return trajectory;

}

} /* namespace TOService */
