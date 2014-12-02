/*
 * GuessGenerator.h
 *
 *  Created on: Dec 1, 2014
 *      Author: perry
 */

#ifndef GUESSGENERATOR_H_
#define GUESSGENERATOR_H_

#include <Eigen/Core>

namespace TOService{

class GuessGenerator {

public:

	GuessGenerator(int num_waypoints_ = 5):
			num_waypoints(num_waypoints_){}
	virtual ~GuessGenerator(){};

	virtual Eigen::MatrixXf generateGuess(std::vector<double> start, std::vector<double> end, std::vector<double> parameters) = 0;

	int num_waypoints;

};


} //TOService

#endif /* GUESSGENERATOR_H_ */
