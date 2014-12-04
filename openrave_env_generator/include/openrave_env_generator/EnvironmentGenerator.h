/*
 * EnvironmentGenerator.h
 *
 *  Created on: Nov 18, 2014
 *      Author: perry
 */

#ifndef ENVIRONMENTGENERATOR_H_
#define ENVIRONMENTGENERATOR_H_

#include <openrave/openrave.h>
#include <openrave-0.9/openrave-core.h>

namespace TOService {

class EnvironmentGenerator {
public:
	EnvironmentGenerator(){
		env = OpenRAVE::RaveCreateEnvironment();
	}
	virtual ~EnvironmentGenerator(){}

	inline OpenRAVE::EnvironmentBasePtr getEnvironment(){ return env; }

	virtual void specifyParams(std::vector<double>){};

protected:

	OpenRAVE::EnvironmentBasePtr env;
	virtual void generateEnvironment() = 0;


};

} /* namespace TrajOptService */

#endif /* ENVIRONMENTGENERATOR_H_ */
