// Author: David Miller


#include <vector>
#include <iostream>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <fstream>
#include <sstream>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>


using namespace std;

struct Connection
{
    double weight;
    double deltaWeight;

    friend class boost::serialization::access;
	// When the class Archive corresponds to an output archive, the
	// & operator is defined similar to <<.  Likewise, when the class Archive
	// is a type of input archive the & operator is defined similar to >>.
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & weight;
		ar & deltaWeight;
	}

};

class Neuron;

typedef vector<Neuron> Layer;

// Silly class to read training data from a text file -- Replace This.
// Replace class TrainingData with whatever you need to get input data into the
// program, e.g., connect to a database, or take a stream of data from stdin, or
// from a file specified by a command line argument, etc.

class TrainingData
{
public:
    TrainingData(const string filename);
    bool isEof(void) { return m_trainingDataFile.eof(); }
    void getTopology(vector<unsigned> &topology);

    // Returns the number of input values read from the file:
    unsigned getNextInputs(vector<double> &inputVals);
    unsigned getTargetOutputs(vector<double> &targetOutputVals);

private:
    ifstream m_trainingDataFile;
};

class Neuron
{
public:
    Neuron(unsigned numOutputs, unsigned myIndex);
    void setOutputVal(double val) { m_outputVal = val; }
    double getOutputVal(void) const { return m_outputVal; }
    void feedForward(const Layer &prevLayer);
    void calcOutputGradients(double targetVal);
    void calcHiddenGradients(const Layer &nextLayer);
    void updateInputWeights(Layer &prevLayer);

private:
    static double eta;   // [0.0..1.0] overall net training rate
    static double alpha; // [0.0..n] multiplier of last weight change (momentum)
    static double transferFunction(double x);
    static double transferFunctionDerivative(double x);
    static double randomWeight(void) { return rand() / double(RAND_MAX); }
    double sumDOW(const Layer &nextLayer) const;
    double m_outputVal;
    vector<Connection> m_outputWeights;
    unsigned m_myIndex;
    double m_gradient;

	Neuron(){
		eta = 0;
		alpha = 0;
		m_outputVal = 0;
		m_outputWeights = vector<Connection>();
		m_myIndex = 0;
		m_gradient = 0;
	}
    friend class boost::serialization::access;
	// When the class Archive corresponds to an output archive, the
	// & operator is defined similar to <<.  Likewise, when the class Archive
	// is a type of input archive the & operator is defined similar to >>.
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & eta;
		ar & alpha;
		ar & m_outputVal;
		ar & m_outputWeights;
		ar & m_myIndex;
		ar & m_gradient;
	}

};
/*
namespace boost { namespace serialization {
template<class Archive>
inline void save_construct_data(
    Archive & ar, const Neuron * t, const unsigned int file_version
){
    // save data required to construct instance
    ar << t->m_myIndex;
    ar << t->m_outputWeights.size();
}

template<class Archive>
inline void load_construct_data(
    Archive & ar, Neuron * t, const unsigned int file_version
){
    // retrieve data from archive required to construct new instance
	unsigned myIndex;
	ar >> myIndex;
    size_t numOutputs;
    ar >> numOutputs;
    // invoke inplace constructor to initialize instance of my_class
    ::new(t)Neuron(numOutputs, myIndex);
}
}} // namespace ...

*/

class Net
{
public:
    Net(const vector<unsigned> &topology);
    void feedForward(const vector<double> &inputVals);
    void backProp(const vector<double> &targetVals);
    void getResults(vector<double> &resultVals) const;
    double getRecentAverageError(void) const { return m_recentAverageError; }

private:
    vector<Layer> m_layers; // m_layers[layerNum][neuronNum]
    double m_error;
    double m_recentAverageError;
    static double m_recentAverageSmoothingFactor;

    friend class boost::serialization::access;
	// When the class Archive corresponds to an output archive, the
	// & operator is defined similar to <<.  Likewise, when the class Archive
	// is a type of input archive the & operator is defined similar to >>.
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & m_layers;
		ar & m_error;
		ar & m_recentAverageError;
		ar & m_recentAverageSmoothingFactor;
	}
};

Net createNet(vector<unsigned> topology, vector<vector<double>> inputs, vector<vector<double>> targets);

void saveNet(const Net &net, string filename);
Net loadNet(string filename);
void showVectorVals(string label, vector<double> &v);

