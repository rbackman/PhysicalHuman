Physical Human 
Robert Backman 
rbackman@ucmerced.edu

The data directory is organized as follows

config: 
man.cfg							this is the inital full state of the character that is being loaded
	man_controller.cfg			parameter definitions for the controllers
		man_ik.cfg				parameters for IKModule
		man_puppet.cfg			parameters for Puppet Controller
		man_virtual.cfg			parameters for VirtualModule
	man_joints.cfg				parameters for all the joints

current.cfg  used for the app to save its state.. this gets overwritten constantly

kinematics: input files from graphsim. skeletons and motions
models: various obj models.
