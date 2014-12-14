/*
 * savelaserdata.cpp
 *
 *  Created on: Nov 18, 2014
 *      Author: Peng
 */

#include <openrave_env_generator/SaveLaserData.h>

namespace TOService {

    SaveLaserData::SaveLaserData(){}

    SaveLaserData::~SaveLaserData(){}

    void SaveLaserData::getSensors(std::vector <OpenRAVE::SensorBasePtr> sensors) {
        this->sensors = sensors;
    }

    int SaveLaserData::save(char tj_result) {

        int static count = 0;
        int fIndex = 0;
        ++count;

        boost::shared_ptr <OpenRAVE::SensorBase::LaserSensorData> pdata(new OpenRAVE::SensorBase::LaserSensorData);

        sensors[0]->Configure(OpenRAVE::SensorBase::CC_PowerOn);
        sensors[0]->Configure(OpenRAVE::SensorBase::CC_RenderDataOn);




        std::ofstream outputFile;
        outputFile.open("/home/peng/ScanedEnv.txt", std::ios::in | std::ios::app);

        for (int kk = 0; kk < 100; ++kk) {
            sensors[0]->SimulationStep(0.1);
            sensors[0]->GetSensorData(pdata);
            outputFile << tj_result << ' ';
            for (int ii = 0; ii < pdata->ranges.size(); ++ii) {

                outputFile << ++fIndex << ':' << distance(pdata->ranges[ii][0], pdata->ranges[ii][1], pdata->ranges[ii][2]) << ' ';
                outputFile << ++fIndex << ':' << pdata->intensity[ii] << ' ';
             }
            
        }

        outputFile << "\n";
        outputFile.close();

        std::cout << "Save Environment: " << count << " with Index: " << tj_result << '\n';

        std::cout << "One Environment scaned and saved! \n";

        return count;
    }
}


