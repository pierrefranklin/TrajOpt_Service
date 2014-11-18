/*
 * EGLoader.cpp
 *
 *  Created on: Nov 18, 2014
 *      Author: perry
 */

#include <openrave_env_generator/EGLoader.h>

#include <openrave/openrave.h>

namespace TOService {

EGLoader::EGLoader(std::string filename_): filename(filename_) {
	generateEnvironment();
}

EGLoader::~EGLoader() {
	// TODO Auto-generated destructor stub
}

void EGLoader::generateEnvironment(){
	env->Load(filename);
}

} /* namespace TrajOptService */
