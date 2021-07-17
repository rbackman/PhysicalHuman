#include "neural_net.h"

#include <fstream>
#include "util.h" //for noise
NeuralNet::NeuralNet( int dim_in,int dim_out,int d_hidden, float amin,float amax,float wrange,float lr, float mm, float wd )
{
	_dim_out = dim_out;
	_dim_in = dim_in;
	_dim_hidden = d_hidden;

	_sig_min = amin;
	_sig_max = amax;
	weight_range  = wrange;
	learning_rate = lr;
	momentum = mm;
	weight_decay = wd;

	hidden_unit = 0;
	output_unit = 0;
	_num_train = 0;
	setDimensions(_dim_in,_dim_hidden,_dim_out);
	
}

NeuralNet::NeuralNet(const char* cfgfile )
{
	hidden_unit = 0;
	output_unit = 0;

	loadFromFile(cfgfile,true);

	weight_range  = 0.3f;
	learning_rate = 0.04f;
	momentum = 0;
	weight_decay = 0;

}
void NeuralNet::setDimensions(int din,int dhid,int dout)
{

	_dim_in = din;
	_dim_hidden = dhid;
	_dim_out = dout;

	if(_dim_hidden == 0)
	{
		hidden_unit = 0;
		if(output_unit)
			delete output_unit;

		output_unit = new layer(_dim_out,_dim_in+1); //add one for the bias
	}
	else
	{
		if(hidden_unit)
			delete hidden_unit;
		if(output_unit)
			delete output_unit;

		hidden_unit = new layer(_dim_hidden,_dim_in+1); //add one for the bias
		output_unit = new layer(_dim_out,_dim_hidden+1);
	}


}
void NeuralNet::setHiddenLayerSize( int i )
{
	if(i!=_dim_hidden)
	{
		if(hidden_unit)
			delete hidden_unit;
		

		if(i==0)
		{
			hidden_unit =0;
			
		}
		else
		{
			_dim_hidden = i;
			hidden_unit = new layer(_dim_hidden,_dim_in+1); //add one for the bias
			if(output_unit)
				delete output_unit;
			output_unit = new layer(_dim_out,_dim_hidden+1);
		}
		
	}
}

float NeuralNet::activationFunction(float d)
{
	return _sig_min + (_sig_max-_sig_min)/(1+exp(-d));
}

void NeuralNet::propogateForward(std::vector<float>* input)
{
	if(input->size() !=_dim_in)
	{
		printf("bad dimensions on example din %d  should be din %d \n",input->size(),_dim_in);
		return;
	}
	
	perceptron* p;
	float op;
	if(hidden_unit)
	{
		for (int i = 0; i < _dim_hidden; i++)
		{
			op = 0;
			p = &hidden_unit->neurons[i];
			for (int j=0;j<_dim_in;j++)
			{
				op += input->at(j) * p->weights[j];
			}
			op+=p->weights[_dim_in]; //add bias weight
			p->output =  activationFunction(op);
		} 

		for (int i = 0; i < _dim_out; i++)
		{
			op = 0;
			p = &output_unit->neurons[i];
			perceptron* hp;
			for (int j=0;j<_dim_hidden;j++)
			{
				hp = &hidden_unit->neurons[j];
				op += hp->output * p->weights[j];
			}
			op+=p->weights[_dim_hidden]; //add bias weight
			p->output =  activationFunction(op);
		} 
	}
	else
	{
		for (int i = 0; i < _dim_out; i++)
		{
			op = 0;
			p = &output_unit->neurons[i];
			for (int j=0;j<_dim_in;j++)
			{
				op += input->at(j)* p->weights[j];
			}
			op+=p->weights[_dim_in]; //add the bias
			p->output =  activationFunction(op);
		} 
	}
}

float NeuralNet::getSSError(std::vector<float>* output)
{
	float sse = 0;
	for (int i = 0; i < _dim_out; i++)
	{
		sse +=  pow(output->at(i) - output_unit->neurons[i].output,2) ;
	}
	return (float)sse;
}

void NeuralNet::backPropogate(std::vector<float>* input,std::vector<float>* output)
{
	
	if(hidden_unit)
	{
		for (int j = 0; j < _dim_out; j++)
		{
			perceptron* out_n = &output_unit->neurons[j];

			//derivative of activation function times the error
			out_n->error = (out_n->output - output->at(j)) * out_n->output * (1-out_n->output) ;

			perceptron* hp;
			for (int i = 0;i < _dim_hidden;i++)
			{
				hp = &hidden_unit->neurons[i];
				out_n->dw[i] +=  out_n->error*hp->output; //accumulate weight change values
			}
			out_n->dw[_dim_hidden] += out_n->error; //update bias weight
		}

		for (int j = 0; j < _dim_hidden; j++)
		{
			perceptron* hid_n = &hidden_unit->neurons[j];

			float sum_wd=0;
			perceptron* out_n;
			for (int i = 0;i < _dim_out ;i++)
			{
				out_n = &output_unit->neurons[i];
				sum_wd +=  out_n->weights[j] * out_n->error;  //sum up from the output layer all the weights * the delta values calculated above
			}

			hid_n->error  = sum_wd * hid_n->output * (1-hid_n->output);

			for (int i = 0;i < _dim_in;i++)
			{
				hid_n->dw[i] +=  hid_n->error*input->at(i);
			}
			hid_n->dw[_dim_in] += hid_n->error; //update bias weight
		}
	}
	else
	{
		for (int j = 0; j < _dim_out; j++)
		{
			perceptron* out_n = &output_unit->neurons[j];

			out_n->error = (out_n->output - output->at(j)) * out_n->output * (1-out_n->output);

			for (int i = 0;i < _dim_in;i++)
			{
				out_n->dw[i] +=  out_n->error*input->at(i);
			}
			out_n->dw[_dim_in] += out_n->error; //update bias weight
		}
	}
	
	
}
void NeuralNet::updateWeights(layer* l)
{
	perceptron* neuron;
	for (unsigned int i = 0; i < l->neurons.size(); i++)
	{		
		for (unsigned int j = 0; j < l->neurons[i].weights.size(); j++) // <= _dim_in to account for bias weight which is the last one
		{
			neuron = &l->neurons[i];
			neuron->weights[j] -= neuron->dw[j]*learning_rate * (1.0f - 2*weight_decay*learning_rate) + neuron->dw_old[j]*momentum;
			neuron->dw_old[j] = neuron->dw[j];
			neuron->dw[j] =0;
		}
	}
}
void NeuralNet::updateWeights()
{
	_num_train++;

	if(hidden_unit)
		updateWeights(hidden_unit);

	updateWeights(output_unit);	
}

NeuralNet::~NeuralNet(void)
{
}
void NeuralNet::init_weights( layer* l )
{
	float wt = 0;
	perceptron* p;
	for (unsigned int i=0;i<l->neurons.size();i++)
	{
		p = &l->neurons[i];
		for (int j = 0; j < l->inputs ; j++)
		{
			wt = -weight_range/2.0f + (weight_range)* ((float)rand()/RAND_MAX);
			p->weights[j] = wt;
		}
	}
}
void NeuralNet::init_weights(  )
{
	if(hidden_unit)
		init_weights(hidden_unit);

	init_weights(output_unit);
	_num_train = 0;
}

void NeuralNet::printOutput()
{
	printf("output weights:\n");
	for (int i=0;i<_dim_out;i++)
	{
		for (unsigned int j=0;j<output_unit->neurons[i].weights.size();j++)
		{
			printf("%g ",output_unit->neurons[i].weights[j]);
		}
		printf("\n");
	}

// 	printf("output:");
// 	for (int i=0;i<_dim_out;i++)
// 	{
// 		printf("%g ",output[i]);
// 	}
	printf("hyp: ");
	for (int i=0;i<_dim_out;i++)
	{
		printf(" %g ",output_unit->neurons[i].output);
	}
	printf("\n");
}

std::vector<float> NeuralNet::getOutput()
{
	std::vector<float> v;
	v.resize(_dim_out);
	for (int i = 0; i < _dim_out; i++)
	{
		v[i] = output_unit->neurons[i].output;
	}
	return v;
}

void NeuralNet::saveToFile( const char* filename )
{
	
	std::ofstream logfile(filename);
	logfile<<_dim_in_w<<" "<<_dim_in_h<<" "<<_dim_in<<" "<<_dim_hidden<<" "<<_dim_out<<" "<<_sig_min<<" "<<_sig_max<<" "<<_num_train<<"\n";

	
	for (int i = 0; i < _dim_hidden; i++)
	{
		for (int j=0;j<_dim_in;j++)
		{
			logfile<<hidden_unit->neurons[i].weights[j]<<" ";
		}
		logfile<<"\n";
	}

	logfile<<-1<<"\n";

	for (int i = 0; i < _dim_out; i++)
	{
		for (int j=0;j<_dim_hidden;j++)
		{
			logfile<<output_unit->neurons[i].weights[j]<<" ";
		}
	}
	logfile.close();
	printf("done saving\n");
}

bool NeuralNet::loadFromFile( const char* filename ,bool forceLoad)
{
	FILE* file = fopen(filename,"r");
	if(!file)
	{
		return false;
	}
	int d_in;
	int d_hid;
	int d_out;
	int d_in_w;
	int d_in_h;
	unsigned int n_train;
	float s_max;
	float s_min;
	fscanf(file,"%d",&d_in_w);
	fscanf(file,"%d",&d_in_h);

	fscanf(file,"%d",&d_in);
	fscanf(file,"%d",&d_hid);
	fscanf(file,"%d",&d_out);
	fscanf(file,"%g",&s_min);
	fscanf(file,"%g",&s_max);
	fscanf(file,"%d",&n_train);

	_num_train = n_train;
	_sig_min = s_min;
	_sig_max = s_max;
	_dim_in_w = d_in_w;
	_dim_in_h = d_in_h;
	_dim_out = d_out;
	if(forceLoad)
	{
		setDimensions(d_in,d_hid,d_out);
	}
	else if(d_in != _dim_in || d_hid != _dim_hidden || d_out != _dim_out)
	{
		gsout<<"dimensions dont match this net  din:"<<_dim_in<<" dhid:"<<_dim_hidden<<" dout:"<<_dim_out;
		return false;
	}
	
	
	for (int i = 0; i < _dim_hidden; i++)
	{
		for (int j=0;j<_dim_in;j++)
		{
			fscanf(file,"%g",&hidden_unit->neurons[i].weights[j]);
		}
	}
	int check;
	fscanf(file,"%d",&check);
	if(check!=-1)
	{
		printf("didn't find marker\n");
		return false;
	}
	for (int i = 0; i < _dim_out; i++)
	{
		for (int j=0;j<_dim_hidden;j++)
		{
			fscanf(file,"%g",&output_unit->neurons[i].weights[j]);
		}
	}
	printf("done loading\n");
	return true;
}

int NeuralNet::dimInW()
{
	return _dim_in_w;
}

int NeuralNet::dimInH()
{
	return _dim_in_h;
}

void NeuralNet::setImgDim( int dx, int dy )
{
	_dim_in_w = dx;
	_dim_in_h = dy;
}

unsigned int NeuralNet::getNumTrain()
{
	return _num_train;
}



void NeuralNet::init_weights_noise(int seeds)
{
	float wt = 0;
	perceptron* p;
	if(hidden_unit)
	{
		for (unsigned int i=0;i<hidden_unit->neurons.size();i++)
		{
			p = &hidden_unit->neurons[i];
			for (int j = 0; j < hidden_unit->inputs ; j++)
			{
				wt = (weight_range/2.0f)*noise(((float)i)/hidden_unit->neurons.size() * seeds,((float)j)/hidden_unit->inputs * seeds);
				p->weights[j] = wt;
			}
		}
	}

	for (unsigned int i=0;i<output_unit->neurons.size();i++)
	{
		p = &output_unit->neurons[i];
		for (int j = 0; j < output_unit->inputs ; j++)
		{
			wt = (weight_range/2.0f)*noise(((float)i)/output_unit->neurons.size()*10.0,((float)j)/output_unit->inputs*10.0);
			p->weights[j] = wt;
		}
	}


}
