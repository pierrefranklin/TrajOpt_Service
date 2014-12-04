/*
 * BezierQuadraticGenerator.h
 *
 *  Created on: Dec 2, 2014
 *      Author: perry
 */

#ifndef BEZIERQUADRATICGENERATOR_H_
#define BEZIERQUADRATICGENERATOR_H_

#include <initial_trajectory/GuessGenerator.h>

namespace TOService {

class BezierQuadraticGenerator : public GuessGenerator {
public:
	BezierQuadraticGenerator();
	virtual ~BezierQuadraticGenerator();

	Eigen::MatrixXf generateGuess(std::vector<double> start, std::vector<double> end, std::vector<double> parameters); /*override*/
};

} /* namespace TOService */

#endif /* BEZIERQUADRATICGENERATOR_H_ */
