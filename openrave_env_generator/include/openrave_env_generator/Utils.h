/*
 * Utils.h
 *
 *  Created on: Dec 1, 2014
 *      Author: perry
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <array>
#include <openrave/openrave.h>
#include <openrave-0.9/openrave-core.h>
#include <iostream>

namespace TOService{

OpenRAVE::KinBody::GeometryInfoPtr makeBox(std::vector<double> extents, std::vector<double> location, std::vector<double> quaternion ){

	if(extents.size() < 3){
		std::cout<<"Extents will seg fault"<<std::endl;
	}
	if(location.size() < 3){
		std::cout<<"Location will seg fault"<<std::endl;
	}
	if(quaternion.size() < 3){
		std::cout<<"Quaternion will seg fault"<<std::endl;
	}

	OpenRAVE::KinBody::GeometryInfoPtr object(new OpenRAVE::KinBody::GeometryInfo());
	object->_type = OpenRAVE::GeometryType::GT_Box;
	object->_bModifiable = false;
	object->_vGeomData = OpenRAVE::Vector(extents[0],extents[1],extents[2]);
	OpenRAVE::Transform pose(OpenRAVE::Vector(quaternion[0],quaternion[1],quaternion[2],quaternion[3]), OpenRAVE::Vector(location[0],location[1], location[2]));

	object->_t = pose;

	return object;
}

}//namespace TOService

#endif /* UTILS_H_ */
