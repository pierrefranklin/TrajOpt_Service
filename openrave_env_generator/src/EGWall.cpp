/*
 * EGWall.cpp
 *
 *  Created on: Dec 1, 2014
 *      Author: perry
 */

#include <openrave_env_generator/EGWall.h>
#include <openrave_env_generator/Utils.h>
#include <iostream>

namespace TOService {

EGWall::EGWall(bool defaultLocation_):
	defaultLocation(defaultLocation_){

	generateEnvironment();
}

EGWall::~EGWall() {

}

void EGWall::generateEnvironment(){

	env->Reset();

	std::vector<OpenRAVE::KinBody::GeometryInfoConstPtr> geoms;

	OpenRAVE::KinBodyPtr wall = OpenRAVE::RaveCreateKinBody(env);
	geoms.push_back(makeBox({0.02,0.05,1},{0,0.20,0},{0,0,0,1}));
	geoms.push_back(makeBox({0.02,0.05,1},{0,-0.20,0},{0,0,0,1}));
	geoms.push_back(makeBox({0.02,0.25,0.475},{0,0,0.625},{0,0,0,1}));
	geoms.push_back(makeBox({0.02,0.25,0.475},{0,0,-0.625},{0,0,0,1}));

	wall->SetName("wall");

	static std::default_random_engine rgen;
    std::uniform_real_distribution<double> loc(-0.2,0.2);
    std::uniform_real_distribution<double> rot(-0.1,0.1);

	wall->InitFromGeometries(geoms);
	if(defaultLocation){
		wall->SetTransform(OpenRAVE::Transform(OpenRAVE::Vector(0,0,0,1), OpenRAVE::Vector(0.75,0,1.5,0)));
	} else	{
		wall->SetTransform(OpenRAVE::Transform(OpenRAVE::Vector(rot(rgen),rot(rgen),rot(rgen),1).normalize4(), OpenRAVE::Vector(loc(rgen)+0.75,loc(rgen)-0.4,1.5+loc(rgen),0)));
		std::cout<<"Randomized"<<std::endl;
	}
	env->Add(wall,true);

}


} /* namespace TOService */
