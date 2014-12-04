/*
 * DimQuadraticGenerator.h
 *
 *  Created on: Dec 1, 2014
 *      Author: perry
 */

#ifndef DIMQUADRATICGENERATOR_H_
#define DIMQUADRATICGENERATOR_H_

#include <initial_trajectory/GuessGenerator.h>

namespace TOService {

class DimQuadraticGenerator : GuessGenerator{
public:
	DimQuadraticGenerator();
	virtual ~DimQuadraticGenerator();

	Eigen::MatrixXf generateGuess(std::vector<double> start, std::vector<double> end, std::vector<double> parameters); /*override*/

};

} /* namespace TOService */

#endif /* DIMQUADRATICGENERATOR_H_ */
