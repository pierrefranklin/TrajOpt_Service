/*
 * SAVE_LASER_DATA.h
 *
 *  Created on: Dec 13, 2014
 *      Author: peng
 */

#ifndef SAVE_LASER_DATA_H_
#define SAVE_LASER_DATA_H_

#include <openrave_env_generator/EnvironmentGenerator.h>
#include <openrave_env_generator/EGLoader.h>
#include <openrave_env_generator/EGTable.h>
#include <openrave_env_generator/EGWall.h>
#include <openrave/sensor.h>
#include <iostream>
#include <openrave-core.h>
#include <vector>
#include <fstream>
#include <cmath>

namespace TOService {

    class SaveLaserData {
    public:
        SaveLaserData(std::vector <OpenRAVE::SensorBasePtr> sensors);
        ~SaveLaserData(){}

        inline double distance(double x1, double y1, double z1, double x2,  double y2, double z2)
        {
            double x = x1 - x2;
            double y = y1 - y2;
            double z = z1 - z2;
            double dist;

            dist = std::pow(x,2)+std::pow(y,2) + std::pow(z,2);
            dist = std::sqrt(dist);

            return dist;
        }

        int save();

    private:


        std::vector <OpenRAVE::SensorBasePtr> sensors;


    };
} /* namespace  */

#endif /* DEMO_LASER_H_ */
