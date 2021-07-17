#pragma once

enum base_control_parms
{
	module_active,
	module_visible,
	base_controller_last
};
enum manip_module_parms
{
	manip_module_list = base_controller_last,
	manip_module_match,
	manip_module_match_frame,
	manip_module_origin,
	manip_module_frame,
	manip_module_last,
};
enum ik_controller_parms
{
	ik_reach_mode = manip_module_last,
};
enum puppet_control_parms
{
	puppet_strength = manip_module_last,

};
enum virtual_control_parms
{
	virtual_strength = manip_module_last,
	virtual_align_feet,
	virtual_align_feet_gain
};
enum root_control_parms
{
	root_gain_p = base_controller_last,
	root_gain_d,
	root_max_ang,
	root_stance_hip_damping,
	root_stance_hip_max_velocity,
	root_max_t,
	root_strength,
	root_torque,
};
enum balance_control_parms
{
	balance_jcom_gain_p = base_controller_last,
	balance_jcom_gain_d,
	balance_jcom_velocity_desired,
	balance_jcom_virtual_force,
	balance_linear_p,
	balance_linear_d,
	balance_linear_ankle_scale,
	balance_linear,
	balance_linear_offset,
	balance_linear_max_offset,
	balance_linear_max_speed,
	balance_vf_max,
	balance_d,
	balance_v,
	balance_simbicon_gain_d,
	balance_simbicon_gain_v,
	balance_simbicon_active,
	balance_simbicon_root_scale
};
enum gravity_control_parms
{
	gravity_magnitude = base_controller_last
};
enum contact_control_parms
{
	contact_root_mix = base_controller_last,
	contact_root_transition_speed,
	contact_transition_speed,
	contact_is_balanced,
	contact_stance_swing_ratio,
	contact_last_pos,
	contact_next_pos,
	contact_pos,
	contact_proj_pos,
	contact_transition,
	contact_is_transitioning,
	contact_show_contact_forces,
	contact_show_contact_region,
	contact_floor_height,
	contact_cop_pos,
	contact_pressure,
	contact_show_cop,
	contact_show_ip_prediction,
	contact_toe_heel_ratio,
	contact_stance_offset,
	contact_double_support,

};

enum com_control_params
{

	com_proj_pos = base_controller_last,
	com_pos,
	com_desired_pos,
	com_sample_ratio,
	com_velocity,
	com_velocity_sample,
	com_support_vector,
	com_desired_support_vector
};

enum reference_control_parms
{
	reference_skeleton_position = base_controller_last,
	reference_skeleton_visible,

	
};
enum motion_control_parms
{
	motion_file  = base_controller_last,
	//motion_segment_phase,
	//motion_current_segment,
	//motion_segment_playing,

};



enum human_params
{
	human_character_name,
	human_default_state,
	human_joint_list,
	human_module_list,
	human_reference_skeleton_name,
	human_models_shaded,
	human_collide_feet_only,
	human_double_support,
	human_stance_state,
	human_mass,
	human_visualization_skeleton,
	human_show_collision_geo,
	human_show_visual_geo,
	human_show_axis,
	human_show_skeleton,
	human_show_heading,
	human_show_controllers,
	human_manual_com,
	human_gain_mult,
	human_desired_heading_delta,
	human_desired_v_scale,
	human_show_support_vec,
};
enum human_state_parms
{
	human_state_human_joint_list,
	human_state_config_file,
	human_state_files_character_name,
};
enum joint_params
{
	joint_type,			/*pString joint type name*/
	joint_parent,		/*pString joint parent name*/
	joint_anchor_point,	/*pVec the global position of the anchor point*/
	joint_box_orientation,
	joint_box_offset,	/*pVec the offset vector of the collision geometry and COM*/
	joint_box_dim,		/*pVec the dimension of the collision geometry or bounding box*/
	joint_color,		/*pColor color of the joint for display*/
	joint_use_pd,		/*pBool decides if this joint should be tracked with PD servo*/
	joint_gain_p,		/*pFloat proportional gain component for tracking*/
	joint_gain_d,		/*pFloat derivative gain component for tracking*/
	joint_pd_scale,		/*pVec a scaling factor for PD control gains to allow axis dependent gains*/
	joint_max_t,		/*pFloat maximum magnitude of the torque*/
	joint_grav_comp,	/*pBool if this joint should use gravity compensation*/
	joint_char_frame,	/*pBool if this joint should be tracked in character frame coordinates. if false tracks in local coordinates*/
	joint_rot_offset,	/*pVec represents a 3DOF rotation offset from the setpoint*/
	joint_vf_scale,		/*pVec a scaling factor for the balance torques that are computed in BalanceModule*/
	joint_rot_order,	/*pString string representation of rotation order*/
	joint_limits,		/*pFloat array of joint rotation limits applied based on rotation order*/
	joint_draw_torque,  /*pBool displays a line representing the torque vector*/
	joint_draw_goal_box,/*pBool displays the desired setpoint orientation for this joint*/
	joint_desired_rot,
	joint_setpoint_rot,
	joint_to_copy,
	joint_desired_rotational_velocity,
	joint_gain_mult_p,
	joint_gain_mult_d,
};

