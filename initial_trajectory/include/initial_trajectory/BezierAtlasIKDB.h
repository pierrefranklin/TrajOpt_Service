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

	void setGridSize(int gridSize_);
	void setGridScale(double gridScale_);

	int getNumIndices();

	boost::shared_ptr<BezierQuadraticGenerator> getTrajGenerator();

private:

	boost::shared_ptr<BezierQuadraticGenerator> BQG;

	Eigen::Vector3d calculateMiddle(int index, Eigen::Vector3d start, Eigen::Vector3d end);

	SFIkCon ik;
	IKcmd ik_d;
	PelvRobotState rs;

	//Parameters for the Bezier Curve
	int gridSize; //The X or Y extent of the square grid
	double gridScale; //scaling for the grid

};

} /* namespace TOService */

#endif /* BEZIERCONSTANTDB_H_ */
