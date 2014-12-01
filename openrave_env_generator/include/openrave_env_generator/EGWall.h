/*
 * EGWall.h
 *
 *  Created on: Dec 1, 2014
 *      Author: perry
 */

#ifndef EGWALL_H_
#define EGWALL_H_

#include <openrave_env_generator/EnvironmentGenerator.h>

namespace TOService {

class EGWall : public EnvironmentGenerator {
public:
	EGWall(bool defaultLocation_ = false);
	virtual ~EGWall();

protected:
	bool defaultLocation;

	void generateEnvironment();
};

} /* namespace TOService */

#endif /* EGWALL_H_ */
