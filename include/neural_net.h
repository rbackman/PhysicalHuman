#pragma once

#include <vector>
#include "common.h"

typedef struct perceptron
{
	std::vector<float> weights;
	std::vector<float> dw;
	std::vector<float> dw_old; //for momentum calculation
	float output;
	float error;
}perceptron;

typedef struct layer
{
	std::vector<perceptron> neurons;
	int inputs;
	layer(int x,int in_d)
	{
		neurons.resize(x);
		inputs = in_d;
		for (int i=0;i<x;i++)
		{
			neurons[i].weights.resize(in_d); //= Vec(in_d);
			neurons[i].dw_old.resize(in_d); // = Vec(in_d);
			neurons[i].dw.resize(in_d);// = Vec(in_d);
		}
	}
	
}layer;



class NeuralNet
{
public:

	float weight_range;
	float learning_rate;
	float momentum;
	float weight_decay;

	NeuralNet( int dim_in,int dim_out,int d_hidden, float amin,float amax,float wrange,float learning_rate, float momentum,float weight_decay);
	NeuralNet(const char* cfgfile);

	~NeuralNet(void);
	void saveToFile(const char* file);
	bool loadFromFile(const char* file,bool forceLoad = false);
	void init_weights();
	void propogateForward(std::vector<float>* vin );
	void backPropogate(std::vector<float>* input,std::vector<float>* output);
	void updateWeights();
	void setHiddenLayerSize(int i);
	void printOutput();
	float getSSError(std::vector<float>* output);
	std::vector<float>  getOutput();
	int dimIn(){return _dim_in;}
	int dimOut(){return _dim_out;}
	int dimHidden(){return _dim_hidden;};
	layer* hidden_unit;
	layer* output_unit;
	int dimInW();
	int dimInH();
	void setImgDim( int dx, int dy );
	unsigned int getNumTrain();
	void init_weights_noise(int seeds);

private:
	void init_weights( layer* l );
	float activationFunction(float d);
	void updateWeights(layer* l);
	void setDimensions(int din,int dhid,int dout);
	

	//std::vector<float> input;
	//std::vector<float> output;
	float sse;
	float _sig_min;
	float _sig_max;
	
	unsigned int _num_train;
	int _dim_in;
	int _dim_in_w;
	int _dim_in_h;

	int _dim_out;
	int _dim_hidden;
};

