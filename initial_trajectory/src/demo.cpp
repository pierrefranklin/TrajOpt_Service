/*
 * demo.cpp
 *
 *  Created on: Dec 2, 2014
 *      Author: perry
 */


#include <initial_trajectory/DimQuadraticGenerator.h>
#include <initial_trajectory/BezierQuadraticGenerator.h>
#include <iostream>


using namespace TOService;

int main(){

	std::vector<double> start = {0,0,0,0,0};
	std::vector<double> end = {10,10,10,0,10};
	std::vector<double> param = {5,2,0,5,0};

	BezierQuadraticGenerator dqg;
	dqg.num_waypoints = 20;

	std::cout << dqg.generateGuess(start,end,param) <<std::endl;


	return 0;

}


