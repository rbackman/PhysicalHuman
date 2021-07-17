
#include "util_serializable.h"
#include <gsim/gs_strings.h>

#ifdef _DEBUG
#define ASSERT_BOUNDS(x) if(x>=parameters.size() || x<0) { phout<<"parameter "<<x<<" of savable "<<name()<<" is out of bounds size:"<< parameters.size()<<gsnl;} else
#define ASSERT_SET(x,y)  if(parameters.get(x)->type != y){ phout<<"ERROR!! setting "<<parameters.get(x)->name<<" of object "<< name()<< " as "<< parmTypeToString(y) <<" since it is type "<<parmTypeToString(parameters.get(x)->type)<<gsnl;return;}else
#define ASSERT_GET(x,y)  if(!parameters.get(x)){phout<<"parmater "<<x<<" of object "<<name()<<" is null "<<gsnl; }else if(parameters.get(x)->type != y){ phout<<"ERROR!! getting "<<parameters.get(x)->name<<" of object "<< name()<< " as "<< parmTypeToString(y) <<" since it is type "<<parmTypeToString(parameters.get(x)->type)<<gsnl;}else
#else
#define ASSERT_BOUNDS(x)
#define ASSERT_SET(x,y) 
#define ASSERT_GET(x,y) 
#endif

Serializable::Serializable(const GsString& n,const GsString& type)
{
	_name = n;
	_type = type;
}

Serializable::Serializable( Serializable* sav )
{
	_name = sav->name();
	_type = sav->type();
	loadParametersFromSerializable(sav);
}

Serializable::~Serializable()
{
	for(int i=0;i<parameters.size();i++)
	{
		delete parameters.get(i);
	}
	parameters.size(0);
}
void Serializable::applyParameters()
{
	//phout<<"updated parameters\n"<<toString()<<gsnl;
}
void Serializable::updateParameters()
{
	//phout<<"updated parameters\n"<<toString()<<gsnl;
}
void Serializable::makeSpot(int n,const GsString& nme,parameter_types type,bool checked)
{
	if(n>parameters.size()-1)
	{
		while(n>=parameters.size())
		{
			parameters.push() = 0;
		}
	}
	ControlParameter* p = parameters.get(n);
	if(p!=0)
	{
		if (p->name==nme && p->type==type)
		{
			//phout<<"parameter "<<nme<<" is already made\n";
			p->checked = checked;
			return;
		}
		//phout<<"adding "<<nme<<" at index "<<n<<" "<<parameters.get(n)->name<<" was there\n";
		parameters.push();
		parameters.top() = p;
		parameters.get(n) = 0;
	}
	switch(type)
	{
		case STRING_PARM:  p = new StringParameter;  break;
		case INT_PARM:		p = new IntParameter;	break;
		case FLOAT_PARM:	p = new FloatParameter; break;
		case BOOL_PARM:		p = new BoolParameter; break;
		case VEC_PARM:		p = new VecParameter; break;
		case COLOR_PARM:	p = new ColorParameter; break;
		case QUAT_PARM:		p = new QuatParameter; break;
		default: phout<<" you messed up\n";
	}
	#ifdef VERIFY_PARAMETERS
		p->checked = checked;
	#endif

	p->name = nme;
	parameters.get(n) = p;
}

void Serializable::makeParameter(int n, const GsString& nme, const GsString& v,bool save)
{
	GsStrings str;
	 str.push(v);
	makeParameter(n,nme,str,save);
}
void Serializable::makeParameter(int n, const GsString& nme,const GsArray<float>& vals,bool save)
{
	makeSpot(n,nme,FLOAT_PARM);
	((FloatParameter*)parameters.get(n))->val.push(vals);
	parameters.get(n)->save = save;
	parameters.get(n)->checked = true;
}
void Serializable::makeParameter(int n, const GsString& nme, GsColor val,bool save)
{
	makeSpot(n,nme,COLOR_PARM);
	((ColorParameter*)parameters.get(n))->val = val;
	parameters.get(n)->save = save;
	parameters.get(n)->checked = true;
}
void Serializable::makeParameter(int n, const GsString& nme,const GsStrings& str,bool save)
{
	makeSpot(n,nme,STRING_PARM);
	for(int i=0;i<str.size();i++)
		((StringParameter*)parameters.get(n))->val.push(str[i]);
	
	if(str.size()==0)
		parameters.get(n)->save = false;
	else
		parameters.get(n)->save = save;

	parameters.get(n)->checked = true;
}
void Serializable::makeParameter(int n, const GsString& nme, float v,bool save)
{
	makeSpot(n,nme,FLOAT_PARM);
	((FloatParameter*)parameters.get(n))->val.push(v);
	parameters.get(n)->save = save;
	parameters.get(n)->checked = true;
}
void Serializable::makeParameter(int n, const GsString& nme, int v,bool save)
{
	makeSpot(n,nme,INT_PARM);
	((IntParameter*)parameters.get(n))->val.push(v);
	parameters.get(n)->save = save;
	parameters.get(n)->checked = true;
}
void Serializable::makeParameter(int n, const GsString& nme,const GsArray<int>& v,bool save)
{
	makeSpot(n,nme,INT_PARM);
	((IntParameter*)parameters.get(n))->val.push(v);
	parameters.get(n)->save = save;
	parameters.get(n)->checked = true;
}
void Serializable::makeParameter(int n, const GsString& nme, bool v,bool save)
{
	makeSpot(n,nme,BOOL_PARM);
	((BoolParameter*)parameters.get(n))->val = v;
	parameters.get(n)->save = save;
	parameters.get(n)->checked = true;
}
void Serializable::makeParameter(int n, const GsString& nme, GsVec v,bool save)
{
	makeSpot(n,nme,VEC_PARM);
	((VecParameter*)parameters.get(n))->val = v;
	parameters.get(n)->save = save;
	parameters.get(n)->checked = true;
}

void Serializable::makeParameter(int n,const GsString& nme,GsQuat v,bool save)
{
	makeSpot(n,nme,QUAT_PARM);
	((QuatParameter*)parameters.get(n))->val = v;
	parameters.get(n)->save = save;
	parameters.get(n)->checked = true;
}
ControlParameter* Serializable::getParameter(const GsString& nme)
{
	for(int i=0;i<parameters.size();i++)
	{
		if(GsString::compare(parameters.get(i)->name,nme)==0)
		{
			return parameters.get(i);
		}
	}
	return 0;
}

ControlParameter* Serializable::getParameter( int num )
{
	ASSERT_BOUNDS(num)
	{
		return parameters.get(num);
	}
	return 0;
}

int Serializable::getParameterIndex(const GsString& nme)
{
	for(int i=0;i<parameters.size();i++)
	{
		if(GsString::compare(parameters.get(i)->name,nme)==0)
		{
			return i;
		}
	}
	//phout<<"couldnt find index of parameter "<<nme<<" \n";
	return -1;
}



void Serializable::setP(const GsString& nme,float v,int idx)
{
	int i = getParameterIndex(nme);
	if(i>=0)
		setP(i,v,idx);
}

void  Serializable::setP(const GsString& nme,bool v)
{
	int i = getParameterIndex(nme);
	if(i>=0)
		setP(i,v);
}
void  Serializable::setP(const GsString& nme,const GsVec& v)
{
	int i = getParameterIndex(nme);
	if(i>=0)
		setP(i,v);
}
void  Serializable::setP(const GsString& nme,const GsColor& v)
{
	int i = getParameterIndex(nme);
	if(i>=0)
		setP(i,v);
}

void  Serializable::setP(const GsString& nme,int v)
{
	int i = getParameterIndex(nme);
	if(i>=0)
		setP(i,v,0);
}
void  Serializable::setP(const GsString& nme,int v,int idx)
{
	int i = getParameterIndex(nme);
	if(i>=0)
		setP(i,v,idx);
}
void Serializable::setP(const GsString& nme,const GsString& v)
{
	for(int i=0;i<parameters.size();i++)
	{
		if(GsString::compare(parameters.get(i)->name,nme)==0)
		{
			setP(i,v);
			return;
		}
	}
	phout<<"couldnt find parameter "<<nme<<" to set\n";
}
//////////////////////////////
//these are the typical methods that would be used since they acces a parameter by index instead of name
void  Serializable::setP(int n,bool v)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_SET(n,BOOL_PARM)
		((BoolParameter*)parameters.get(n))->val = v;
	}
}

QuatParameter* Serializable::getQuatParameter( int num )
{
	if(getParameter(num))
		if(getParameter(num)->type==QUAT_PARM)
			return (QuatParameter*)getParameter(num);

	phout<<"not quat parm for "<<name(); 
	return 0;
}

FloatParameter* Serializable::getFloatParameter( int num )
{
	if(getParameter(num))
		if(getParameter(num)->type==FLOAT_PARM)
			return (FloatParameter*)getParameter(num);

	phout<<"not float array for "<<name(); 
	return 0;
}
void Serializable::setP(int n,float v,int idx)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_SET(n,FLOAT_PARM )
		{
			if(idx<((FloatParameter*)parameters.get(n))->val.size() && idx>=0)
				((FloatParameter*)parameters.get(n))->val.get(idx) = v;
		}
	}
}

void  Serializable::setP(int n,int v,int idx)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_SET(n,INT_PARM)
		{
			if(((IntParameter*)parameters.get(n))->val.size()<=idx)
			{
				((IntParameter*)parameters.get(n))->val.size(idx+1);
			}
			((IntParameter*)parameters.get(n))->val[idx] = v;
		}
	}
}

void  Serializable::setP(int n,const GsVec& v)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_SET(n,VEC_PARM)
			((VecParameter*)parameters.get(n))->val = v;
	}
}
void  Serializable::setP(int n,const GsColor& v)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_SET(n,COLOR_PARM)
			((ColorParameter*)parameters.get(n))->val = v;
	}
}
void  Serializable::setP(int n,const GsQuat& v)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_SET(n,QUAT_PARM)
			getQuatParameter(n)->val = v;
	}
}

void  Serializable::setP(int n,const GsString& v,int idx)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_SET(n,STRING_PARM)
		{
			if(idx<getStringParameter(n)->val.size())
			{
				getStringParameter(n)->val.set(idx,v);
			}
			else
			{
				getStringParameter(n)->val.size(idx+1);
				getStringParameter(n)->val.set(idx,v);
			}
		}
	}
	
}
void Serializable::setP(int n,const GsArray<int>& ints)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_SET(n,FLOAT_PARM)
		{
			getIntParameter(n)->val.size(0);
			getIntParameter(n)->val.push(ints);
		}
	}
}
void Serializable::setP( int n,const GsArray<float>& f )
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_SET(n,FLOAT_PARM)
		{
			getFloatParameter(n)->val.size(0);
			getFloatParameter(n)->val.push(f);
		}
	}
}

void Serializable::setP( int n,const GsStrings& f)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_SET(n,STRING_PARM)
		{
			((StringParameter*)parameters.get(n))->val.size(0);
			for(int i=0;i<f.size();i++)
				((StringParameter*)parameters.get(n))->val.push(f[i]);
		}
	}
}

GsColor Serializable::pColor(int n)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_GET(n,COLOR_PARM)
			return ((ColorParameter*) parameters.get(n))->val;
	}
	GsColor c = GsColor::blue;
	return c;
}
float Serializable::pFloat(int n,int idx)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_GET(n,FLOAT_PARM)
			return  getFloatParameter(n)->val.get(idx);
	}
	return 0.0f;
}

GsQuat Serializable::pQuat(int n)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_GET(n,QUAT_PARM)
			return getQuatParameter(n)->val;
	}
	return GsQuat();
}
int  Serializable::pInt(int n)
{
	return pInt(n,0);
}
int  Serializable::pInt(int n,int idx)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_GET(n,INT_PARM)
			return getIntParameter(n)->val[idx];
	}
	return 0;
}
GsString  Serializable::pString(int n,int num)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_GET(n,STRING_PARM)
			return getStringParameter(n)->val.get(num);
	}
	return GsString("bogus");
}

GsStrings*  Serializable::pStringArray(int n)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_GET(n,STRING_PARM) 
			return &getStringParameter(n)->val;
	}
	return 0;
}
GsArray<float>* Serializable::pFloatArray( int n )
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_GET(n,FLOAT_PARM) 
			return &getFloatParameter(n)->val;
	}
	return 0;
}


int  Serializable::sizeOfParameter(int n)
{
	if(parameters.get(n)->type==STRING_PARM)
	{
		return ((StringParameter*) parameters.get(n))->val.size();
	}
	if(parameters.get(n)->type==FLOAT_PARM)
	{
		return getFloatParameter(n)->val.size();
	}
	if(parameters.get(n)->type==INT_PARM)
	{
		return getIntParameter(n)->val.size();
	}
	return 1;
}


  bool  Serializable::pBool(int n)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_GET(n,BOOL_PARM)
			return ((BoolParameter*) parameters.get(n))->val;
	}
		return false;
}
 GsVec  Serializable::pVec(int n)
{
	ASSERT_BOUNDS(n)
	{
		ASSERT_GET(n,VEC_PARM)
			return ((VecParameter*) parameters.get(n))->val;
	}
		return GsVec();
}

bool Serializable::getVarsFromFile(GsVars* v, const GsString& file,const GsString& objectName)
{
	v->init();
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

		if ( in.ltoken()== objectName )
		{ 
			GsVars parms;
			in >> parms;
			v->merge ( parms );
			return true;
		}
		
		else if ( in.ltoken()=="end" )
		{ 
			return false; //didnt find the object in the file
		}
		else
		{
			//skipping
			GsVars parms;
			in >> parms;
		}
	}
return false;
}
GsColor varToColor(GsVar* v)
{
	if(v->size() == 4)
	{
		return GsColor(v->geti(1),v->geti(2),v->geti(3));
	}
	else
	{
		GsString label(v->gets(1));
		if(label=="blue")return GsColor::blue;
		if(label=="black")return GsColor::black;
		if(label=="red")return GsColor::red;
		if(label=="yellow")return GsColor::yellow;
		if(label=="brown")return GsColor::brown;
		if(label=="cyan")return GsColor::cyan;
		if(label=="darkblue")return GsColor::darkblue;
		if(label=="green")return GsColor::green;
		if(label=="magenta")return GsColor::magenta;
		if(label=="white")return GsColor::white;
		if(label=="gray")return GsColor::gray;
		if(label=="orange")return GsColor::orange;
		if(label=="darkred")return GsColor::darkred;
		if(label=="darkgreen")return GsColor::darkgreen;
		if(label=="lightblue")return GsColor::lightblue;
	}
	return GsColor(v->geti(1),v->geti(2),v->geti(3));
}
void Serializable::setParametersFromSerializable(Serializable* sav)
{
	if(!sav)
	{
		phout<<"setting "<<name()<<" with a null savable\n";
		return ;
	}
	for(int i=0;i<sav->numParameters();i++)
	{
		
		ControlParameter* parm = sav->getParameter(i);
		int idx = getParameterIndex(parm->name);
		if(idx==-1)
		{
			//phout<<"error Savable::setParametersFromSerializable(Savable* sav): "<<name()<<" does not have a parameter "<<parm->name<<gsnl;
		}
		else
		{
			ControlParameter* p = parameters.get(idx);
			
			if(parm->type != p->type)
			{
				phout<<name()<<" is copying the wrong parm type\n";
				
			}
			else
			{
				//phout<<"setting this parameter "<<p->getString();
				//phout<<"to this one "<<parm->getString();
				p->set(parm);
			}
		
		}
	}
}
void Serializable::setParametersFromVars(GsVars* var)
{
	for(int i=0;i<parameters.size();i++)
	{
		ControlParameter* p = parameters.get(i);
		GsVar* v = var->get(p->name);
		
		if(v)
		{
			
			switch(p->type)
			{
			case 	INT_PARM:
				setP(i, v->geti());
			break;
			case 	BOOL_PARM:
				setP(i,v->getb());
			break;
			case 	VEC_PARM:
			{
				GsVec ve = GsVec(v->getf(1),v->getf(2),v->getf(3));
				//phout<<"copying value "<<p->name<<" "<<" as vec "<<ve<<" from "<<var->name()<<" to "<<name()<<gsnl;
				setP(i,ve);
			}
			break;
			case 	COLOR_PARM:
			{
				setP(i,varToColor(v));
			}
			break;
			case 	QUAT_PARM:
				setP(i,GsQuat(v->getf(1),v->getf(2),v->getf(3),v->getf(4)));
			break;
			case 	FLOAT_PARM:
			{	
				for(int j=0;j<v->size();j++)
					setP(i, v->getf(j) , j );
			}
			break;
			case 	STRING_PARM:
				((StringParameter*)p)->setStrings(v);
			
			break;
				

			}
		}
		else
		{
			//this file doesnt contain that parameter so ignore
				//phout<<"couldnt find "<<p->name<<" for "<<name()<<gsnl;
			
		}
	}
}
void Serializable::setParametersFromSerializableGroup(SerializableGroup* grp)
{
	Serializable* s	= grp->getSerializable(name());
	if(s)
	{
		setParametersFromSerializable(s);
	}
}
void Serializable::setParametersFromFile(const GsString& file,const GsString& labelName)
{
	GsVars* v = new GsVars;
	setFileName(file);
	if(getVarsFromFile(v,file,labelName))
	{
		setParametersFromVars(v);
	}
	else
	{
		phout<<"couldnt set parameters for "<<name()<<" with label "<<labelName<<gsnl;
	}
	delete v;
}
void Serializable::setParametersFromFile(const GsString& file)
{
	setParametersFromFile(file,_name);
}
bool Serializable::loadParametersFromFile(const GsString& file,const GsString& labelName)
{
	GsVars* v = new GsVars;
	setFileName(file);
	if(getVarsFromFile(v,file,labelName))
	{
		loadParameters(v);
		delete v;
		return true;
	}
	else
	{
		delete v;
		return false;
	}
	
	
}
bool Serializable::loadParametersFromFile(const GsString& file)
{
return	loadParametersFromFile(file,_name);
}
#ifdef VERIFY_PARAMETERS
bool Serializable::verifyParameters()
{
	bool allGood = true;
	for(int i=0;i<parameters.size();i++)
	{
		if(parameters.get(i)==0)
		{
			phout<<"for some reason you have a null parameter "<<i<<" for object "<<name()<<" type "<<type()<<gsnl;
			return false;
		}
		if(!parameters.get(i)->checked)
		{

			allGood = false;
			
			#ifdef _DEBUG
			phout<<"WARNING!! extra parameter in file: "<<parameters.get(i)->name<<" not checked for object "<<_name<<gsnl;
			#endif
		}
	}
	return allGood;
}
#endif
bool Serializable::checkParameter(int idx,const GsString& nme,int type)
{
	bool found = false;


	if(idx>=parameters.size())
	{
		int spacesNeeded = parameters.size()+1-idx;
		
		for(int i=0;i<spacesNeeded;i++)
			parameters.push()=0;

		found = false;
	}
	else if(parameters.get(idx)!=0)
	{
		if(parameters.get(idx)->checked)
		{
			//phout<<"something is wrong because you are moving an already checked parameter "<<nme<<"\n";
			return false;
		}

		if( parameters.get(idx)->name == nme )
		{
			

			#ifdef VERIFY_PARAMETERS
				parameters.get(idx)->checked=true;
			#endif
			found = true;
		} 
		
	}

	for(int i=0;i<parameters.size() && !found;i++)
	{
		ControlParameter* pi = parameters.get(i);
		if(pi != 0)
		{
			if(parameters.get(i)->name == nme)
			{
				
				ControlParameter* oldcp = parameters.get(idx);
				parameters.get(idx)=parameters.get(i);
				parameters.get(i) = oldcp;
				#ifdef VERIFY_PARAMETERS
					parameters.get(idx)->checked=true;
				#endif
				found = true;

			}
		}
	}
	
	if(!found)
	{
		//phout<<"never found parameter "<<nme<<" for object "<<name()<<" so making it from scratch\n";
		switch(type)
		{
			case INT_PARM:   makeParameter(idx,nme,0,false); break;
			case BOOL_PARM:  makeParameter(idx,nme,false,false);  break;
			case VEC_PARM:   makeParameter(idx,nme,GsVec(),false);  break;
			case COLOR_PARM:   makeParameter(idx,nme,GsColor::blue,false);  break;
			case FLOAT_PARM:
			{
				GsArray<float> arr;
				arr.push(0.0f);
				makeParameter(idx,nme,arr,false);  
				setP(idx,0.0f);
			}break;
			case STRING_PARM: 
				{
					GsStrings str;
				//	str.push("empty");
					makeParameter(idx,nme,str,false);  
				} break;
			case QUAT_PARM:  makeParameter(idx,nme,GsQuat() ,false);  break;
		}

	}
	
	//phout<<"#"<<idx<<" "<<parameters[idx]->getString();
	return found;

}
GsString Serializable::stateString()
{
	return toString();
}
void Serializable::loadParametersFromSerializable(Serializable* sav)
{

	for (int i=0;i<parameters.size();i++)
	{
		delete parameters[i];
		parameters[i] = 0;
	}
	
	parameters.size(0);

	for (int i=0;i<sav->getParameters().size();i++)
	{
		
		ControlParameter* pcpy = sav->getParameters().get(i);
		if(pcpy)
		{
			bool unique = true;
			for(int j=0;j<parameters.size();j++)
			{
				if(parameters[j]->name == pcpy->name)
				{
					phout<<"loadPsFromSavable() found duplicate savable parameter "<<pcpy->name<<gsnl;
					unique = false;
				}
			}
			if(unique)
			{
				makeSpot( i, pcpy->name , pcpy->type,false/*false since it needs to be checked still*/); 
				parameters[i]->set(pcpy);
			}
		}
		//parameters[i]->checked = false;
	}
	
}

void Serializable::loadParameters(GsVars* var)
{
	for(int i=0;i<parameters.size();i++)
		delete parameters.get(i);
	parameters.size(0);

// 	parameters.size(var->size());
// 	for(int i=0;i<parameters.size();i++)
// 	{
// 		parameters.get(i) = 0;
// 	}
	for(int i=0;i<var->size();i++)
	{
		bool unique = true;
		for (int j=0;j<parameters.size();j++)
		{
			if (var->get(i).name() ==parameters[j]->name )
			{
				unique = false;
				phout<<"loadParameters() found duplicate parameter "<<parameters[j]->name<<gsnl;
			}
		}
		ControlParameter* p = 0;
		if(unique)
		{
			if(var->get(i).size()==0)
			{
				phout<<"param "<<var->get(i).name()<<" has no values\n";
				continue;
			}
			else
			{
				switch(var->get(i).type())
				{//'b', 'i', 'f' or 's' */
				case 'i':{
					p = new IntParameter;
					for(int j=0;j<var->get(i).size();j++)
					{
						((IntParameter*)p)->val.push(var->get(i).geti(j));
					}
					if(var->get(i).size()==0)
						p->save = false;
						 }break;
				case 'b':{
					p = new BoolParameter;
					((BoolParameter* )p)->val = var->get(i).getb();

						 }break;
				case 'f':{

							 {
								 p = new FloatParameter;
								 for(int j=0;j<var->get(i).size();j++)
								 {
									 ((FloatParameter*)p)->val.push(var->get(i).getf(j));
								 }
								 if(var->get(i).size()==0)
									 p->save = false;	
							 }
						 }break;
				case 's':
					{
						const char* lab = var->get(i).gets(0);
						if(GsString::compare(lab,"vec")==0)
						{
							p = new VecParameter;
							if(GsString::compare(var->get(i).gets(1),"null")==0)
							{
								((VecParameter* )p)->val = GsVec();
							}
							else if(GsString::compare(var->get(i).gets(1),"unit_i")==0)
							{
								((VecParameter* )p)->val = GsVec::i;
							}
							else if(GsString::compare(var->get(i).gets(1),"unit_j")==0)
							{
								((VecParameter* )p)->val = GsVec::j;
							}
							else if(GsString::compare(var->get(i).gets(1),"unit_k")==0)
							{
								((VecParameter* )p)->val = GsVec::k;
							}
							else
							{
								((VecParameter* )p)->val.x = var->get(i).getf(1);
								((VecParameter* )p)->val.y = var->get(i).getf(2);
								((VecParameter* )p)->val.z = var->get(i).getf(3);
							}

						}else if(GsString::compare(lab,"quat")==0)
						{

							p = new QuatParameter;
							if(GsString::compare(var->get(i).gets(1),"null")==0)
							{
								((QuatParameter* )p)->val = GsQuat::null;
							}
							else
							{
								((QuatParameter* )p)->val.set(var->get(i).getf(1),var->get(i).getf(2), var->get(i).getf(3), var->get(i).getf(4));

							}


						}
						else if(GsString::compare(lab,"color")==0)
						{

							p = new ColorParameter;
							GsColor c = varToColor(&(var->get(i)));	
							((ColorParameter* )p)->val = c;

						}
						else
						{
							p = new StringParameter;
							((StringParameter*)p)->setStrings(&var->get(i));
						}
						if(var->get(i).size()==0)
							p->save = false;
					}
					break;

				}
				parameters.push() = p;
				parameters.top()->name = var->get(i).name();
			}
		}
	}

}

GsString Serializable::toString(bool printAll)
{
	GsString f = name();
	f<<"\n{\n";
	for(int i=0;i<parameters.size();i++)
	{
		if(parameters.get(i)!=0)
		{
			//f<<"# "<<i<<" ";
			if(parameters.get(i)->save || printAll)
				f<<"\t"<<parameters.get(i)->getString();
			//else
			//	gsout<<type()<<" "<<name()<<" parameter: "<<parameters.get(i)->name<<" does not save\n";
		}

	}
	f<<"}\n";
	return f;
}
GsArray<ControlParameter*> Serializable::getParameters()
{
	return parameters;
}

void Serializable::toggleBool( int n )
{
	if(getParameter(n))
	{
		if(parameters[n]->type==BOOL_PARM)
		{
			setP(n,!pBool(n));
		}
		else
		{
			phout<<"cant toggle parm "<<parameters[n]->name<<" since it isnt bool "<<gsnl;
		}
	}
}

StringParameter* Serializable::getStringParameter( int num )
{
	return (StringParameter*)getParameter(num);
}

IntParameter* Serializable::getIntParameter( int num )
{
	return (IntParameter*)getParameter(num);
}

void Serializable::setFileName( const GsString& n )
{
	_filename = n; _short_filename = n; 
	remove_extension(_short_filename);
	remove_path(_short_filename);
}


GenericSerializable::GenericSerializable( const GsString& name ,GsVars* vars):Serializable(name)
{
		loadParameters(vars);
}

GenericSerializable::GenericSerializable( const GsString& name ):Serializable(name)
{

}
void GenericSerializable::makeParameter( ControlParameter* toCopy )
{
	parameters.push() = 0;
		switch(toCopy->type)
		{
			case INT_PARM:
				parameters.top() = new IntParameter;
				break;
			case BOOL_PARM:
					parameters.top() = new BoolParameter;
				break;
			case VEC_PARM:
					parameters.top() = new VecParameter;
				break;
			case COLOR_PARM:
					parameters.top() = new ColorParameter;
				break;
			case FLOAT_PARM:
					parameters.top() = new FloatParameter;
				break;
			case STRING_PARM:
					parameters.top() = new StringParameter;
				break;
			case QUAT_PARM:
					parameters.top() = new QuatParameter;
				break;
		}
		if(parameters.top()==0)
			parameters.pop();
		else
		{
			parameters.top()->name = toCopy->name;
			parameters.top()->set(toCopy);
		}
}

void SerializableGroup::addSerializable( GenericSerializable* sav )
{
	for (int i=0;i<_children.size();i++)
	{
		if(_children[i]==sav)
		{
			phout<<"error adding savable twice\n";
			return;
		}
	}
	if(sav==0)
	{
		phout<<"error adding null savable\n";
		return ;
	}
	_children.push(sav);
}

SerializableGroup::~SerializableGroup()
{
	for (int i=0;i<_children.size();i++)
	{
		delete _children[i];
	}
	_children.size(0);
}
