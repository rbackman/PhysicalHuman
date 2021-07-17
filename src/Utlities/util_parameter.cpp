
#include "util_parameter.h"


GsString ControlParameter::getString()
{
	GsString l = name ;
	l<<" = ";
	switch(type)
	{
	case INT_PARM:   
		{

			int size = ((IntParameter*)this)->val.size();
			if(size==0)l<< "0";

			for(int i=0;i<size;i++)
			{
				l<<" "<<((IntParameter*)this)->val.get(i);
			}
		} break;
	case FLOAT_PARM:  
		{

			int size = ((FloatParameter*)this)->val.size();
			if(size==0)l<< "0.0";

			for(int i=0;i<size;i++)
			{
				l<<" "<<((FloatParameter*)this)->val.get(i);
			}
		} break;
	case BOOL_PARM:  if(((BoolParameter*)this)->val) l << "true";
					 else l <<"false";
					 break;
	case VEC_PARM:  
		{
			GsVec v = ((VecParameter*)this)->val;
			if(v == GsVec::null)
			{
				l <<"vec null";
			}
			else if(v.len()==1)
			{
				if(v.x==1)
				{
					l <<"vec unit_i";
				}
				else if(v.y==1)
				{
					l <<"vec unit_j"; 
				}
				else if(v.z==1)
				{
					l <<"vec unit_k"; 
				}
			}
			else
			{
				l <<"vec " << v.x <<" "<< v.y <<" "<< v.z; 

			}
		}break;
	case COLOR_PARM:  
		l <<"color "<<  ((ColorParameter*)this)->val.r <<" "<< ((ColorParameter*)this)->val.g<<" "<< ((ColorParameter*)this)->val.b; 
		break;

	case QUAT_PARM: 
		{
			GsQuat q = ((QuatParameter*)this)->val;
			if(q == GsQuat::null)
			{
				l<<"quat null";
			}
			else
			{
				l<<"quat "<<q.w <<" "<<q.x <<" "<<q.y <<" "<<q.z; 
			}
		}break;
	case STRING_PARM:   
		if(((StringParameter*)this)->val.size()==0)
		{
			GsString tmp = "#";
			tmp<<name<<" this was empty \n";
			phout<<tmp;
			return tmp;
		}

		for(int i=0;i<((StringParameter*)this)->val.size();i++)
		{

			GsString str = ((StringParameter*)this)->val.get(i);
			if(str.search('.')!=-1 || str.search('/')!=-1)
				l<< "\""<< str <<"\"";
			else
				l<<" "<<str;
		}
		break;
	default: l<<"bogus";
	}
	l<<";\n";
	return l;
}

void ControlParameter::set( ControlParameter* otherP)
{
	if(otherP->type != type)
	{	
		phout<<"wrong type\n";return;
	}
	switch(type)
	{
	case INT_PARM: 
		((IntParameter*)this)->val.size(0);
		((IntParameter*)this)->val.push(((IntParameter*)otherP)->val);	
		break;
	case FLOAT_PARM:   
		((FloatParameter*)this)->val.size(0);
		((FloatParameter*)this)->val.push(((FloatParameter*)otherP)->val);		
		break;

	case BOOL_PARM: ((BoolParameter*)this)->val = ((BoolParameter*)otherP)->val;  break;
	case VEC_PARM:   ((VecParameter*)this)->val = ((VecParameter*)otherP)->val; break;
	case COLOR_PARM:   ((ColorParameter*)this)->val = ((ColorParameter*)otherP)->val; break;
	case QUAT_PARM:   ((QuatParameter*)this)->val = ((QuatParameter*)otherP)->val; break;
	case STRING_PARM:   

		((StringParameter*)this)->val.size(0);

		for(int i=0;i<((StringParameter*)otherP)->val.size();i++)
		{
			((StringParameter*)this)->val.push(((StringParameter*)otherP)->val[i]);
		}
		break;
	default: phout<<"bogusSet\n";
	}
	save = otherP->save;
	name = otherP->name;
	checked = otherP->checked;
}

StringParameter::~StringParameter()
{
	val.size(0);
}
void StringParameter::setStrings(GsVar* v)
{
	val.size(0);

	if(v->size()==0)
		save = false;
	else
		save = true;

	for(int i=0;i<v->size();i++)
	{
		val.push(v->gets(i));
	}
}

void StringParameter::replace( GsString old, GsString s )
{
	for(int i=0;i<val.size();i++)
	{
		if(val[i] == old)
		{
			val.set(i,s);
			return;
		}
	}
	phout<<"error replacing string\n";
}

bool StringParameter::add( GsString name )
{
	for (int i=0;i<val.size();i++)
	{
		if(val[i] == name)
			return false;
	}
	val.push(name);
	return true;
}

void StringParameter::insert( int idx, GsString st )
{
	val.insert(idx,st);
}
