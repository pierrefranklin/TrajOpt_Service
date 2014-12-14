/*
 * savelaserdata.cpp
 *
 *  Created on: Nov 18, 2014
 *      Author: Peng
 */

#include <openrave_env_generator/SaveLaserData.h>

namespace TOService {

    SaveLaserData::SaveLaserData(std::vector <OpenRAVE::SensorBasePtr> sensors) {
        this->sensors = sensors;
    }

    int SaveLaserData::save() {

        int static count = 0;
        ++count;

        boost::shared_ptr <OpenRAVE::SensorBase::LaserSensorData> pdata(new OpenRAVE::SensorBase::LaserSensorData);

        sensors[0]->Configure(OpenRAVE::SensorBase::CC_PowerOn);
        sensors[0]->Configure(OpenRAVE::SensorBase::CC_RenderDataOn);

        std::ofstream outputFile;
        outputFile.open("/home/peng/ScanedEnv.txt", std::ios::in | std::ios::app);

        for (int kk = 0; kk < 100; ++kk) {
            std::cout << kk << '\n';
            sensors[0]->SimulationStep(0.1);
            sensors[0]->GetSensorData(pdata);
            double x_center = pdata->positions[0][0];
            double y_center = pdata->positions[0][1];
            double z_center = pdata->positions[0][2];

            for (int ii = 0; ii < pdata->ranges.size(); ++ii) {
                outputFile << TOService::distance(x_center, y_center, z_center, pdata->ranges[ii][0], pdata->ranges[ii][1], pdata->ranges[ii][2]) << ' ';
                outputFile << pdata->intensity[ii] << ' ';
            }



            //std::cin>>a;

        }

        outputFile << "\n";
        outputFile.close();

        std::cout << "One Environment scaned and saved! \n";

        return count;
    }
}


