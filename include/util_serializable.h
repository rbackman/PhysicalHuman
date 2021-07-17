#pragma once

#include "common.h"
#include "util_parameter.h"

/*
The Savable object is my solution to dealing with all the data of this application. it is a generic class
that has an array of parameters that are sorted based on an enumerator the subclasses must define.
the Savable object can be printed as a string with toString() which is how it is saved. they can also
be loaded from a file with loadParametersFromFile(GsString file) or another savable with 
loadParametersFromSavable(GsString file)

*/

#define CHECK_VEC(x) checkParameter(x,#x,VEC_PARM)
#define CHECK_INT(x) checkParameter(x,#x,INT_PARM)
#define CHECK_FLOAT(x) checkParameter(x,#x,FLOAT_PARM)
#define CHECK_STRING(x) checkParameter(x,#x,STRING_PARM)
#define CHECK_BOOL(x) checkParameter(x,#x,BOOL_PARM)
#define CHECK_QUAT(x) checkParameter(x,#x,QUAT_PARM)
#define CHECK_COLOR(x) checkParameter(x,#x,COLOR_PARM)
#define MAKE_PARM(x,y) makeParameter(x,#x,y)
#define MAKE_TEMP_PARM(x,y) makeParameter(x,#x,y,false)

class SerializableGroup;
class GenericSerializable;

class Serializable
{
private:
	GsString _filename;
	GsString _name;
	GsString _type;
	GsString _short_filename;
public:
	virtual GsString toString(bool printAll = false);
	virtual GsString stateString();
	virtual void applyParameters();
	virtual void updateParameters();
protected:
	
	GsArray<ControlParameter*> parameters;

	void makeParameter(int n,const GsString& name,float v,bool save = true);
	void makeParameter(int n, const GsString& nme,const GsArray<float>& vals,bool save = true);
	void makeParameter(int n,const GsString& name,const GsString& v,bool save = true);
	void makeParameter(int n, const GsString& nme,const GsStrings& str,bool save = true);
	void makeParameter(int n,const GsString& name,const char* v,bool save = true){makeParameter(n,name,GsString(v),save);}


	void makeParameter(int n,const GsString&  name,int v,bool save = true);
	void makeParameter(int n,const GsString&  name,GsVec v,bool save = true);
	void makeParameter(int n,const GsString&  name,GsColor v,bool save = true);
	void makeParameter(int n,const GsString&  name,bool v,bool save = true);
	void makeParameter(int n,const GsString&  name,GsQuat v,bool save = true);
	void makeParameter(int n, const GsString&  nme,const GsArray<int>& v,bool save);

	int parameterType(int n){return parameters.get(n)->type;}
	void makeSpot(int n,const GsString&  nme,parameter_types type,bool checked = true);
	/*this method forces the parameter to be in the array location defined in the enumerator which each Savable object shoudl have.. use CHECK_PARM for simplicity*/
	bool checkParameter(int i,const GsString&  name,int type);

public:
	void setType(const GsString&  n){_type = n;}
	void setName(const GsString&  n){_name = n;}
	void setFileName(const GsString&  n);
	GsString getFileName(){return _filename;}
	GsString getShortFileName(){return _short_filename;}
	void setShortFilename( const GsString&  name )
	{
		_filename.replace(_short_filename,name);
		_short_filename = name;
	}

	GsString name(){return _name;}
	GsString type(){return _type;}
	void toggleBool(int n);
	
	void setP(int n,int v,int idx);
	void setP(int n,int v){setP(n,v,0);}
	void setP(int n,const GsArray<int>& ints);
	void setP(int n,const GsVec& v);
	void setP(int n,bool v);
	
	void setP(int n,float v,int idx);
	void setP(int n,float v){setP(n,v,0);}
	void setP(int n,const GsArray<float>& f);
	void setP(int n,const GsStrings& f);
	void setP(int n,const GsQuat& v);
	void setP(int n,const GsColor& v);

	void setP(int n,const GsString&  v,int i);
	void setP(int n,GsString v){setP(n,v,0);}
	void setP(int n,const char* v){setP(n,GsString(v));}


	int numParameters(){return parameters.size();}

	/*access a quaternion parameter belonging to this Serializable*/
	GsQuat pQuat(int n);
	
	/*access a float array parameter belonging to this Serializable*/
	float pFloat(int n,int idx);
	/*access a float parameter belonging to this Serializable*/
	float pFloat(int n){return pFloat(n,0);}
	/*access the float array for a parameter belonging to this Serializable*/
	GsArray<float>*  pFloatArray(int n);

	/*access an integer parameter belonging to this Serializable*/
	int pInt(int n);
	/*access an integer array parameter belonging to this Serializable*/
	int pInt(int n,int idx);
	/*access a boolean parameter belonging to this Serializable*/
	bool pBool(int n);
	/*access a vector parameter belonging to this Serializable*/
	GsVec pVec(int n);
	/*access a color parameter belonging to this Serializable*/
	GsColor pColor(int n);
	/*access a string array parameter belonging to this Serializable*/
	GsString  pString(int n,int num);
	/*access a string parameter belonging to this Serializable*/
	GsString  pString(int n){return pString(n,0);}
	/*access the entire array of strings for a given parameter*/
	GsStrings*  pStringArray(int n);

	
	ControlParameter* getParameter(const GsString&  nme);
	ControlParameter* getParameter(int num);
	//do some checking
	QuatParameter* getQuatParameter(int num);
	FloatParameter* getFloatParameter(int num);
	StringParameter* getStringParameter(int num);
	IntParameter* getIntParameter(int num);

	/*this will only change the values of the parameters that are found in the file*/
	void setParametersFromFile(const GsString&  file);
	void setParametersFromFile(const GsString&  file,const GsString&  name);
	void setParametersFromSerializableGroup(SerializableGroup* grp);
	int getParameterIndex(const GsString&  nme);

	int  sizeOfParameter(int n);
	bool loadParametersFromFile(const GsString&  file);
	bool loadParametersFromFile(const GsString&  file,const GsString&  labelName);
	 Serializable(const GsString&  n,const GsString&  type = "none");
	   ~Serializable();
protected:
	Serializable(Serializable* sav);
       
      
	
	void loadParameters(GsVars* var);

	#ifdef VERIFY_PARAMETERS
		bool verifyParameters();
	#endif

	
	

	
private:

	void setP(const GsString&  nme,float v,int idx);
	void setP(const GsString&  name,float v){setP(name,v,0);}

	void setP(const GsString&  name,bool v);
	void setP(const GsString&  name,int v);
	void setP(const GsString&  name,const GsVec& v);
	void setP(const GsString&  name,const GsColor& v);
	void setP(const GsString&  name,const GsQuat& v);
	void setP(const GsString&  name,const GsString&  v);
	void setP(const GsString&  name,const char* v){setP(name,GsString(v));}
	void setP(const GsString&  nme,int v,int idx);
	
	

	bool getVarsFromFile(GsVars* v,  const GsString&  file,const GsString&  objectName);

public:
	
	void setParametersFromVars(GsVars* var);
	void setParametersFromSerializable(Serializable* sav);
	void loadParametersFromSerializable(Serializable* sav);

	GsString parameterAsString(int num)
	{return getParameter(num)->getString();}

	GsString nameOfParameter(int num)
	{return getParameter(num)->name;}
	GsArray<ControlParameter*> getParameters();

};

class GenericSerializable:public Serializable
{
public:
	GenericSerializable(const GsString&  name,GsVars* vars);
	GenericSerializable(const GsString&  name);
	~GenericSerializable()
	{
		for (int i=0;i<parameters.size();i++)
		{
			if(parameters[i]!=0)
			{
				delete parameters[i];
			}
		}
		parameters.size(0);
	}
	void makeParameter(ControlParameter* toCopy);
};

class SerializableGroup
{
	GsArray<GenericSerializable*> _children;
public:
	~SerializableGroup();
	
	SerializableGroup()
	{

	}
	int numChildren(){return _children.size();}
	GenericSerializable* getSerializable(int i){return _children[i];}
	GenericSerializable* getSerializable(GsString name)
	{
		for(int i=0;i<numChildren();i++)
		{
			if(name == _children[i]->name())
			{
				return _children[i];
			}
		}
		//phout<<"GenericSavable couldn't find savable "<<name<<gsnl;
		return 0;
	}
	bool loadFromFile(const GsString&  file)
	{
		GsInput in;

		if ( !in.open(file) )
		{ 
			return false;
		}
		in.commentchar ( '#' );
		in.lowercase ( false );

		while ( true )
		{ 
			if ( in.get()==GsInput::End ) break;

			GsString name = in.ltoken();

			GsVars parms;
			in >> parms;
			GenericSerializable* sav = new GenericSerializable(name,&parms);
			_children.push(sav);

		}
		return true;
	}
	void addSerializable( GenericSerializable* human );
};
static GsString parmTypeToString(int i)
{
	switch(i)
	{
	case INT_PARM: return GsString("int"); break;
	case BOOL_PARM: return GsString("bool"); break;
	case VEC_PARM: return GsString("vec"); break;
	case COLOR_PARM: return GsString("color"); break;
	case FLOAT_PARM: return GsString("float"); break;
	case STRING_PARM: return GsString("string"); break;
	case QUAT_PARM: return GsString("quat"); break;
	}
	return GsString("unknown");
}