Physical Human 
Robert Backman 
rbackman@ucmerced.edu


--------------------------------------------------------------------------------

TO USE:
compile /labcode/fltk
compile /labcode/graphsim
compile /labcode/ode
compile /labcode/physicalhuman


--------------------------------------------------------------------------------

IDEAS
-I need to assure that the process of discovering motions correctly measures the distance and height of the jump.. this is done sloppy and i need a clean 
version of this to work

- optimize from mocap stifness curve for one skill
- sample curve/endpoints to vary controller behavior
- plan complex motions with 1 controller
- precompute valid samples offline to enable realtime control
- plan concatenation, leading to some parmetrized graph

PAPER PLAN

 - learn a low-dim parameterized Contrller from Mocap
 - method to make controller work for 1 clip
 - learn many clips representing/covering desired variations
 - every result of learning is 1 "precomp point"
 - learn as many points as needed to "cont cover param space"
 - use fast planner at parametric space, then play with dynamics
 - optional: concatenate controllers
 
 at least 3 diff examples:
 - walking with dodging
 - stepping/climbing stairs
 - seping over dynamic rocks
 - stepping over shaking floor
 - stepping planning
 - jumping
 
 => read parameterized motion graphs paper


--------------------------------------------------------------------------------

TODO: Important

-when I rename channels it needs to update the input list of its children


-start making demos!

-better tracking for upper body. get vel and acel from mocap finite diff
	-maybe the proportional component just comes from refskel 


-graphviewer
	-drag and scale view
	-select and drag points or delete
	-fix labels for step traj
	 

-use virtual manips to manipulate motion. snap them to original motion when needed


TODO: When I get a chance

-state and configuration files .. combine all the files into one to clean things up.. something like this:

Human
{
	Joints
	{
		LeftFoot
		{
		ode_position = vec 0 0 0;
		..
		}
	}
	Modules
	{
		ReferenceModule
		{
			module_active = true;
			refskel_position = vec 0 0 0;
		}
		IKModule
		{
			module_active = true;
			LeftFoot
			{
				ik_position = vec 0 0 0;
			}
			Root
			{
				ik_position = vec 0 0 0;
			}
		}
	}
}

-manipulator:
	-make direction of dragging determine the direction of manipulation
	-simplify collision checks to ray circle test instead of poly collision

-AngularMomentum minimization
 
-PD control
	-look into automatic optimization for gains so new characters can be made easily

-Graphics
	-make environemnt from config file

-Physics
	-look into other physics engines
	

	


