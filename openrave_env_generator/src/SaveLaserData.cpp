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
        sensors[0]->Configure(OpenRAVE::SensorBase::CC_PowerOn);
        sensors[0]->Configure(OpenRAVE::SensorBase::CC_RenderDataOn);
    }

    int SaveLaserData::saveToSVM(int tj_result) {

        int static count = 0;
        int fIndex = 0;

        boost::shared_ptr <OpenRAVE::SensorBase::LaserSensorData> pdata(new OpenRAVE::SensorBase::LaserSensorData);

        std::ofstream outputFile;
        outputFile.open("/home/peng/ScanedEnvSVM.txt", std::ios::in | std::ios::app);
        
        outputFile << tj_result << ' ';
        for (int kk = 0; kk < 100; ++kk) {
            sensors[0]->SimulationStep(0.1);
            if (kk >45 && kk <= 95)
            {
                sensors[0]->GetSensorData(pdata);
                for (int ii = 0; ii < (pdata->ranges.size() - 1); ++ii) {
                    outputFile << ++fIndex << ':' << distance(pdata->ranges[ii][0], pdata->ranges[ii][1], pdata->ranges[ii][2]) << ' ';
                 }
            }
            
        }

        outputFile << "\n";
        outputFile.close();

        std::cout << "Save Environment: " << count << " with Index: " << tj_result << '\n';

        std::cout << "One Environment scaned and saved! \n";

        return ++count;
    }

    int SaveLaserData::saveToANN(int tj_result) {

        int static count = 0;
        int fIndex = 0;
        

        boost::shared_ptr <OpenRAVE::SensorBase::LaserSensorData> pdata(new OpenRAVE::SensorBase::LaserSensorData);

        std::ofstream outputFile;
        outputFile.open("/home/peng/ScanedEnvAnn.txt", std::ios::in | std::ios::app);
        
        /* topology*/
        if (count == 0)
        {
            outputFile << "topology: 2250 500 25" << '\n';
        }

        /* inputs */
        outputFile << "in: ";
        for (int kk = 0; kk < 100; ++kk) {
            sensors[0]->SimulationStep(0.1);
            if (kk >45 | kk < 95)
            {
                sensors[0]->GetSensorData(pdata);
                for (int ii = 0; ii < (pdata->ranges.size() - 1); ++ii) {
                outputFile << distance(pdata->ranges[ii][0], pdata->ranges[ii][1], pdata->ranges[ii][2]) << ' ';
                }
            }
            
            // outputFile << '@';
        }
        outputFile << '\n';

        /* outputs */
        outputFile << "out: ";
        for (int ii = 0; ii < 25; ++ii)
        {
            if (ii == tj_result)
            {
                outputFile << 1.0 << ' ';
            }
            else
            {
                outputFile << 0.0 << ' ';
            }
        }
        
        outputFile << "\n";
        outputFile.close();

        std::cout << "Save Environment: " << count << " with Index: " << tj_result << '\n';

        std::cout << "One Environment scaned and saved! \n";

        return ++count;
    }
}


