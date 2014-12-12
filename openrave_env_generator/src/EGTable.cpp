/*
 * EGTable.cpp
 *
 *  Created on: Nov 19, 2014
 *      Author: perry
 */

#include <openrave_env_generator/EGTable.h>
#include <random>
#include <iostream>

namespace TOService {

EGTable::EGTable(bool defaultTable_, int objects_):
	defaultTable(defaultTable_),
	objects(objects_){

	generateEnvironment();
}

EGTable::~EGTable() {

}

void EGTable::generateEnvironment(){
	env->Reset();

	OpenRAVE::KinBody::GeometryInfoPtr box(new OpenRAVE::KinBody::GeometryInfo());

	box->_type = OpenRAVE::GeometryType::GT_Box;
	box->_bModifiable = false;
	box->_vGeomData = OpenRAVE::Vector(.5,1,.05);

	OpenRAVE::KinBodyPtr table = OpenRAVE::RaveCreateKinBody(env);

	table->SetName("table");
	static std::default_random_engine rgen;
    std::uniform_real_distribution<double> size(0.01,0.35);
    std::uniform_real_distribution<double> loc(-0.5,0.5);

	std::vector<OpenRAVE::KinBody::GeometryInfoConstPtr> geoms = {box};
	for ( int i = 0; i < objects; i++){
		OpenRAVE::KinBody::GeometryInfoPtr object(new OpenRAVE::KinBody::GeometryInfo());
		object->_type = OpenRAVE::GeometryType::GT_Box;
		object->_bModifiable = false;
		object->_vGeomData = OpenRAVE::Vector(size(rgen)/4,size(rgen)/4,size(rgen));
		OpenRAVE::Transform pose(OpenRAVE::Vector(0,0,0,1), OpenRAVE::Vector(loc(rgen),loc(rgen),object->_vGeomData.z));

		object->_t = pose;
		std::cout<<object->_t;
		geoms.push_back(object);
		std::cout<<"  "<<objects<<std::endl;
	}





	table->InitFromGeometries(geoms);
	if (!defaultTable){
	    std::uniform_real_distribution<double> loc(-.2,0.2);
	    std::uniform_real_distribution<double> rot(-0.1,0.1);
	    table->SetTransform(OpenRAVE::Transform(OpenRAVE::Vector(rot(rgen),rot(rgen),rot(rgen),1).normalize4(), OpenRAVE::Vector(1+loc(rgen),loc(rgen),0.5+loc(rgen),0)));

	}else{
		table->SetTransform(OpenRAVE::Transform(OpenRAVE::Vector(0,0,0,1), OpenRAVE::Vector(1,0,0.5,0)));
	}
	env->Add(table,true);

}

} /* namespace TOService */
