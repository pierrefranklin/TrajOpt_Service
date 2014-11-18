/*
 * EGLoader.h
 *
 *  Created on: Nov 18, 2014
 *      Author: perry
 */

#ifndef EGLOADER_H_
#define EGLOADER_H_

#include <openrave_env_generator/EnvironmentGenerator.h>

namespace TOService {



class EGLoader : public EnvironmentGenerator {
public:
	EGLoader(std::string filename_);
	virtual ~EGLoader();

protected:

	std::string filename;
	virtual void generateEnvironment(); /*override*/

};

} /* namespace TrajOptService */

#endif /* EGLOADER_H_ */
