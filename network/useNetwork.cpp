/*
 * useNetwork.cpp
 *
 *  Created on: Dec 14, 2014
 *      Author: perry
 */


#include "artificial_neural_net.h"

int main()
{
    TrainingData trainData("/home/perry/trainingData.txt");

    // e.g., { 3, 2, 1 }
    vector<unsigned> topology;
    trainData.getTopology(topology);

    vector<vector<double>> inputVals, targetVals;

    while (!trainData.isEof()) {

        // Get new input data and feed it forward:
        inputVals.push_back(std::vector<double>());
        if (trainData.getNextInputs(inputVals.back()) != topology[0]) {
        	inputVals.pop_back();
            break;
        }
        showVectorVals(": Inputs:", inputVals.back());

        // Train the net what the outputs should have been:
        targetVals.push_back(std::vector<double>());
        trainData.getTargetOutputs(targetVals.back());
        showVectorVals("Targets:", targetVals.back());
        assert(targetVals.back().size() == topology.back());
    }

    cout<<" inputs.size() = "<< inputVals.size() <<", outputs.size() = "<<targetVals.size()<<endl;
    Net myNet = loadNet("network");

    vector<double> results;

    for(int index = 0; index < inputVals.size(); index++){

		std::vector<double> & currentInput = inputVals[index];
		std::vector<double> & currentTarget = targetVals[index];

		if (currentInput.size() != topology[0]){
			std::cerr<< "inputs for index " << index <<" does not match topology"<<std::endl;
			continue;
		}
		if (currentTarget.size() != topology.back()){
			std::cerr<< "targets for index " << index <<" does not match topology"<<std::endl;
			continue;
		}

		myNet.feedForward(currentInput);
		showVectorVals("Inputs:", currentInput);
        showVectorVals("Targets:", currentTarget);
        myNet.getResults(results);
        showVectorVals("Results:", results);
        std::cout<<std::endl;

	}

    cout<<myNet.getRecentAverageError();

    cout << endl << "Done" << endl;
}
