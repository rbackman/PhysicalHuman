# pragma once

# include <gsim/gs_vars.h>
# include <gsim/fl_vars_win.h>

# include "util_serializable_editor_fluid.h"
#include "util_serializable.h"
#include "util_channel.h"

class SerializableEditor : public SerializableEditorFluid
 { 
 private :
   bool _makesNodes;
	 bool _wantsChannel;
	 channel_dof_types _dof_type;
	 int _dof_array_index;
	 int _parameter_index;
	 ControlParameter* _current_parameter;
	  Serializable* _serializable;

   public :
	   bool makesNodes(){return _makesNodes;}
    SerializableEditor (Serializable* sav,bool makeNode = false);
   ~SerializableEditor ();
    void show ();
	void update();
	//Sets a message about which stance state .. Both Feet, Left Foot etc
	ControlParameter* selectedParm();
	void loadSerializable(Serializable* sav);
	Serializable* getSerializable(){return _serializable;}

	void listSelected();
	void parmEdited();
	GsString name(){return _serializable->name();}
	
   public :
	
	   bool eulerMode;
	bool wantsChannel();

    void event ( SerializableEvent e );
	channel_dof_types getSelectedDof(){return _dof_type;}
	int getDofArrayIdx(){return _dof_array_index;}
	int getParamaterIdx(){return _parameter_index;}
	void reloadList();

};

