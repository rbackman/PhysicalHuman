
# include <gsim/fl.h>

#include <gsim/gs_euler.h>

#include "util_serializable_editor.h"


SerializableEditor::SerializableEditor(Serializable* sav,bool makeNode)
{
	_makesNodes = makeNode;
	ui_make_traj->value(!makeNode);
	loadSerializable(sav);
	_wantsChannel = false;
	_dof_array_index = -1;
	_dof_type = CH_FLOAT;
	eulerMode = true;
}
void SerializableEditor::loadSerializable(Serializable* sav)
{
	_serializable = sav;
	GsString label = sav->name();
	if(sav->type()!="none")
	{
		label<<"_"<<sav->type();
	}
	ui_serializable_name->value(label);
	ui_serializable_editor->label(label);
	ui_serializable_editor->redraw();
	reloadList();
}
void SerializableEditor::parmEdited()
{
	_current_parameter = _serializable->getParameter(ui_parm_list->child(ui_parm_list->value())->label()) ;
	if(!_current_parameter)
	{
		phout<<"parmEdited() error \n";
		return;
	}
	else
	{
		//phout<<"editing "<<p->name<<gsnl;
	}
	switch(_current_parameter->type)
		{
			case STRING_PARM:
				{
					bool found = false;
					for(int i=0;i<ui_string_array_list->children();i++)
					{
						if(ui_string_array_list->child(i)->selected())
						{
							((StringParameter*)_current_parameter)->val.set(i,ui_string_array_current->value());
							phout<<"setting string to "<<((StringParameter*)_current_parameter)->val[i]<<gsnl;
							found = true;
						}
						else
						{
							phout<<i<<" not selected\n";
						}
					}
				}
			break;
	
			case INT_PARM:
				{
					((IntParameter*)_current_parameter)->val[0] = (int)ui_int_input->value();
				}
			break;
			case BOOL_PARM:
				{
					((BoolParameter*)_current_parameter)->val = ui_bool_true->value();
					((BoolParameter*)_current_parameter)->val = !ui_bool_false->value();
				}
			break;
			case FLOAT_PARM:
				{

							int idx = ui_float_array_list->value();
							GsArray<float>* fa = &((FloatParameter*)_current_parameter)->val;

							fa->get(idx) = (float)ui_float_input->value();
							
							ui_float_array_list->remove_all();
	
							GsString l;
							for(int i=0;i<fa->size();i++)
							{
								l = fa->get(i);
								ui_float_array_list->add_leaf(l);
							}

						
				}
			break;
			case VEC_PARM:
				{
					ui_vector_grp->show();
					((VecParameter*)_current_parameter)->val.set((float)ui_vx_input->value(),(float)ui_vy_input->value(),(float)ui_vz_input->value());	
				}
			break;
			case QUAT_PARM:
				{
					if(eulerMode)
					{
						ui_euler_grp->show();
						GsQuat q;
						gs_rot(gsZYX,q,GS_TORAD (ui_eul_x_input->value()),GS_TORAD (ui_eul_y_input->value()),GS_TORAD (ui_eul_z_input->value()));
						((QuatParameter*)_current_parameter)->val = q;
					}
					else
					{
						ui_quat_grp->show();
						((QuatParameter*)_current_parameter)->val.set((float)ui_qw_input->value(),(float)ui_qx_input->value(),(float)ui_qy_input->value(),(float)ui_qz_input->value());	
						((QuatParameter*)_current_parameter)->val.normalize();
						GsQuat v = ((QuatParameter*)_current_parameter)->val;
						ui_qw_input->value(v.w);
						ui_qx_input->value(v.x);
						ui_qy_input->value(v.y);
						ui_qz_input->value(v.z);
					}
				}
				break;
		}

	_serializable->applyParameters();

}
void SerializableEditor::listSelected()
{
	ui_string_grp->hide();
	ui_string_array_grp->hide();
	ui_int_grp->hide();
	ui_bool_grp->hide();
	ui_float_grp->hide();
	ui_vector_grp->hide();
	ui_quat_grp->hide();
	ui_euler_grp->hide();
	if(ui_parm_list->value() == -1)
	{phout<<"nothing selected\n";	return;}
	_parameter_index = _serializable->getParameterIndex(ui_parm_list->child(ui_parm_list->value())->label()); ;
	if(_parameter_index==-1)
		return;

	_current_parameter = _serializable->getParameter(_parameter_index);

	ui_save_parm->value(_current_parameter->save);
	ui_param_name_field->value(_current_parameter->name);
	switch(_current_parameter->type)
	{
		
	case QUAT_PARM:
		{
			if(eulerMode)
			{
				ui_euler_grp->show();
				float x,y,z;
				gs_angles(gsZYX,((QuatParameter*)_current_parameter)->val,x,y,z);
				ui_eul_x_input->value(GS_TODEG(x));
				ui_eul_y_input->value(GS_TODEG(y));
				ui_eul_z_input->value(GS_TODEG(z));
			}
			else
			{
				ui_quat_grp->show();
	
				GsQuat v = ((QuatParameter*)_current_parameter)->val;
				ui_qw_input->value(v.w);
				ui_qx_input->value(v.x);
				ui_qy_input->value(v.y);
				ui_qz_input->value(v.z);
			}
		}
		break;
		case STRING_PARM:
			{
				ui_string_array_grp->show();
				ui_string_array_list->remove_all();
				for(int j=0;j<((StringParameter*)_current_parameter)->val.size();j++)
					ui_string_array_list->add_leaf(((StringParameter*)_current_parameter)->val.get(j));
				ui_string_array_list->child(0)->set_selected();
			}
		break;

		case INT_PARM:
			{
				ui_int_grp->show();

				ui_int_array_list->remove_all();
				GsArray<int>* fa = &((IntParameter*)_current_parameter)->val;
				ui_int_input->value(fa->get(0));
				GsString l;
				for(int i=0;i<fa->size();i++)
				{
					l = fa->get(i);
					ui_int_array_list->add_leaf(l);
				}

				ui_int_input->maximum(1+ui_int_input->value()*2);
				ui_int_max->value(1+ui_int_input->value()*2);

			}
		break;
		case BOOL_PARM:
			{
				if(((BoolParameter*)_current_parameter)->val)
				{
					ui_bool_true->value(true);
					ui_bool_false->value(false);
				}
				else
				{
					ui_bool_false->value(true);
					ui_bool_true->value(false);
				}

				ui_bool_grp->show();
			}
		break;
		case FLOAT_PARM:
			{
	
				ui_float_array_list->remove_all();
				GsArray<float>* fa = &((FloatParameter*)_current_parameter)->val;
				ui_float_input->value(fa->get(0));
				GsString l;
				for(int i=0;i<fa->size();i++)
				{
					l = fa->get(i);
					ui_float_array_list->add_leaf(l);
				}
				
				ui_float_input->maximum(1+ui_float_input->value()*2);
				ui_float_max->value(1+ui_float_input->value()*2);
				ui_float_grp->show();
			
			}
		break;
		case VEC_PARM:
			{
				ui_vector_grp->show();
				GsVec v = ((VecParameter*)_current_parameter)->val;
				ui_vx_input->value(v.x);
				ui_vy_input->value(v.y);
				ui_vz_input->value(v.z);
				ui_vec_max->value(1+v.len()*2);
				
				ui_vx_input->maximum(1+v.len()*2);
				ui_vy_input->maximum(1+v.len()*2);
				ui_vz_input->maximum(1+v.len()*2);

			}
		break;
		case COLOR_PARM:
			{
				GsColor c =((ColorParameter*)_current_parameter)->val;
				if(fl_color_chooser("choose new color",c))
				{
					((ColorParameter*)_current_parameter)->val = c;
				}
					_serializable->applyParameters();
			}
			break;
	}
			 
}
ControlParameter* SerializableEditor::selectedParm()
{
	return _current_parameter;
}
void SerializableEditor::event ( SerializableEvent e )
{
	switch ( e )
	{
		case evParmTraj:
			_makesNodes = !ui_make_traj->value();
		break;

		case evSaveParm:
			{
				_parameter_index = _serializable->getParameterIndex(ui_parm_list->child(ui_parm_list->value())->label()); ;
				if(_parameter_index==-1)
					return;
				_serializable->getParameter(_parameter_index)->save = ui_save_parm->value();
			}break;
		case evSwitchEul:
			{
				eulerMode = true;
				listSelected();
			}break;
		case evSwitchQuat:
			{
				eulerMode = false;
				listSelected();
			}break;
		case evParmEdited:
			parmEdited();
			break;
		case evFloatChoiceSelect:
			{
				int idx = ui_float_array_list->value();
				GsArray<float>* fa = &((FloatParameter*)_current_parameter)->val;
				ui_float_input->value(fa->get(idx));
			}
			break;
		case evLoadList:
			reloadList();
			break;
		case  evCloseParmEditor:  
			_serializable = 0;
			ui_serializable_editor->hide();
		break;
		case evMakeChannelBool:
			_wantsChannel = true;
			_dof_type = CH_BOOL;
		break;
		case evMakeChannelRotX:
			_wantsChannel = true;
			_dof_type = CH_ROT_X;
			break;
		case	evMakeChannelRotY:
			_wantsChannel = true;
				_dof_type = CH_ROT_Y;
			break;
		case evMakeChannelRotZ:
			_wantsChannel = true;
				_dof_type = CH_ROT_Z;
			break;
		case evParmListModified:
			listSelected();
		break;
		case evMakeChannelVecX:
			_wantsChannel = true;
			_dof_type = CH_VEC_X;
			break;
		case evMakeChannelVecY:
			_wantsChannel = true;
			_dof_type = CH_VEC_Y;
			break;
		case evMakeChannelVecZ:
			_wantsChannel = true;
			_dof_type = CH_VEC_Z;
			break;
		case evMakeChannelFloat:
			{
				_wantsChannel = true;
				_dof_type = CH_FLOAT;
				_dof_array_index = ui_float_array_list->value();
			}break;
		default:
			phout<<"Serializable editor doesn't know what to do with that event\n";
			break;
	}
 }
SerializableEditor::~SerializableEditor ()
 {
	 ui_serializable_editor->hide();

 }
void SerializableEditor::update()
{
	if(ui_stream_parm->value())
	{
		if(_serializable)
		{
			listSelected();
			ui_serializable_editor->redraw();
		}
		else
		{
			ui_stream_parm->value(false);
		}
	}
}
void SerializableEditor::show ()
 {
	 if(!ui_serializable_editor->visible())
	  ui_serializable_editor->show();
	
 }

bool SerializableEditor::wantsChannel()
{
	if(_wantsChannel)
	{
		_wantsChannel = false; 
		return true;
	}
	return false;
}
void SerializableEditor::reloadList()
{
	ui_parm_list->remove_all();
	for(int i=0;i<_serializable->numParameters();i++)
	{
		switch(_serializable->getParameter(i)->type)
		{
		case VEC_PARM:
			if(ui_type_choice[0]->value())
				ui_parm_list->add_leaf(_serializable->nameOfParameter(i));
			break;
		case QUAT_PARM:
			if(ui_type_choice[1]->value())
				ui_parm_list->add_leaf(_serializable->nameOfParameter(i));
			break;
		
		case BOOL_PARM:
			if(ui_type_choice[2]->value())
				ui_parm_list->add_leaf(_serializable->nameOfParameter(i));
			break;
		case FLOAT_PARM:
			if(ui_type_choice[3]->value())
				ui_parm_list->add_leaf(_serializable->nameOfParameter(i));
			break;
		case STRING_PARM:
			if(ui_type_choice[4]->value())
				ui_parm_list->add_leaf(_serializable->nameOfParameter(i));
			break;
		case COLOR_PARM:
			if(ui_type_choice[5]->value())
				ui_parm_list->add_leaf(_serializable->nameOfParameter(i));
			break;
		case INT_PARM:
			if(ui_type_choice[6]->value())
				ui_parm_list->add_leaf(_serializable->nameOfParameter(i));
			break;
		
		

		}
		
	}
// 	if(ui_parm_list->children()>0)
// 	{
// 		ui_parm_list->child(0)->set_selected();
// 		ui_serializable_editor->show();
// 		listSelected();
// 	}
}


