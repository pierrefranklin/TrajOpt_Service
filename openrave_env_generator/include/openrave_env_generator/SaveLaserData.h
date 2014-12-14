/*
 * SAVE_LASER_DATA.h
 *
 *  Created on: Dec 13, 2014
 *      Author: peng
 */

/* This is how to save sensor data, now only support a single laser sensor
	 * In order to have the data store correctly, please change the user name in path "/home/peng/ScanedEnv.txt"
	 * in file "SaveLaserData.cpp" .
	 *
	 * Data form:
	 * each environment will be save as 100 points with (double distance, int hit) in one line of the file;
	 * each run of the mdata.save() will attach one line in ScanedEnv.txt
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

        ~SaveLaserData() {
        }

        inline double distance(double x1, double y1, double z1, double x2, double y2, double z2) {
            double x = x1 - x2;
            double y = y1 - y2;
            double z = z1 - z2;
            double dist;

            dist = std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2);
            dist = std::sqrt(dist);

            return dist;
        }

        int save();

    private:
        std::vector <OpenRAVE::SensorBasePtr> sensors;

    };
} /* namespace  */

#endif
