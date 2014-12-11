/*
 * DatabaseTrajectorySearch.h
 *
 *  Created on: Dec 8, 2014
 *      Author: perry
 */

#ifndef DATABASETRAJECTORYSEARCH_H_
#define DATABASETRAJECTORYSEARCH_H_

#include <openrave_env_generator/EnvironmentGenerator.h>

//Shoot me
#define private public
#define protected public

#include <drc_traj/atlas_traj.hpp>

#undef private
#undef protected

//Very important - drc_traj comes before BezierAtlasIKDB
//Fucking defines in manip kill bulbasaur
#include <initial_trajectory/BezierAtlasIKDB.h>

namespace TOService {

class DatabaseTrajectorySearch {

public:

	DatabaseTrajectorySearch(boost::shared_ptr<BezierAtlasIKDB> database_);
	virtual ~DatabaseTrajectorySearch();

	int findBestIndex(OpenRAVE::EnvironmentBaseConstPtr env, Eigen::Vector3d end);


protected:

	boost::shared_ptr<BezierAtlasIKDB> database;

	atlas_traj::AtlasTraj TOHak;

	trajopt::TrajOptResultPtr computeTrajectory(std::vector<double> start_state, Eigen::Affine3d hand, ::Side side, Eigen::MatrixXf initguess);

	void UpdateStateForRequest(std::vector<double>& start_state, Eigen::Affine3d hand, ::Side side, atlas_traj::TrajoptMode mode);
};

} /* namespace TOService */

#endif /* DATABASETRAJECTORYSEARCH_H_ */
