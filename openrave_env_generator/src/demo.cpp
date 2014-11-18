/*
 * demo.cpp
 *
 *  Created on: Nov 18, 2014
 *      Author: perry
 */

#include <openrave_env_generator/EnvironmentGenerator.h>
#include <openrave_env_generator/EGLoader.h>
#include <iostream>


using namespace TOService;

int main(){

	OpenRAVE::RaveInitialize();

	boost::shared_ptr<EnvironmentGenerator> egPtr(new EGLoader("data/lab1.env.xml"));

	OpenRAVE::EnvironmentBasePtr env = egPtr->getEnvironment();

	OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(env, "qtcoin");

	assert(viewer);

	env->AddViewer(viewer);

	while(1){
		viewer->main();
	}

	return 0;

}


