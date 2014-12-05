/*
 * BezierConstantDB.h
 *
 *  Created on: Dec 4, 2014
 *      Author: perry
 */

#ifndef BEZIERCONSTANTDB_H_
#define BEZIERCONSTANTDB_H_

#include <initial_trajectory/BezierQuadraticGenerator.h>
#include <cmu_walk/ik_controller.hpp>

namespace TOService {

class BezierAtlasIKDB {
public:
	BezierAtlasIKDB();
	virtual ~BezierAtlasIKDB();

	std::string getType();

	Eigen::MatrixXf getTrajectory(int index, std::vector<double> start, Eigen::Vector3d end);

private:

	boost::shared_ptr<BezierQuadraticGenerator> BQG;

	Eigen::Vector3d calculateMiddle(int index, Eigen::Vector3d start, Eigen::Vector3d end);

	SFIkCon ik;
	IKcmd ik_d;
	PelvRobotState rs;


};

} /* namespace TOService */

#endif /* BEZIERCONSTANTDB_H_ */
