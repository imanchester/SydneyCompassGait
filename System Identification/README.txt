----------------------------------------------------------------------------
----------------------------------------------------------------------------
------------------- The Sydney Compass-Gait Robot: -------------------------
--- Design and Modeling of an Open Platform for Dynamic Walking Research ---
----------------------------------------------------------------------------
----------------------------------------------------------------------------

SysID_Experiment 1: 
- DataAcquisition_SimplePendulum.slx		: Simulink file used for the data acquisition
- ParameterEstimation_SimplePendulum.slx	: Simple pendulum model
- realVSmodel_fwAnd5vstepsx3AndChirp_innerSW.mat: data recordings when the inner leg is the swing leg
- realVSmodel_fwAnd5vstepsx3AndChirp_outerSW.mat: data recordings when the outer leg is the swing leg

SysID_Experiment 2: 
- CompassGaitModel_wFriction_Assymetric.m: Assymetric Compass Gait model of the continuous phase
- mappingdot.m				 : Impact map of the stance and the hip velocities
- SysID_data_12exp_120steps.mat		 : data set from walking motion
- sysID_data12exp_AsymmetricWalker.m	 : Script that performs the system identification


Last update: 15/09/2017
----------------------------------------------------------------------------
----------------------------------------------------------------------------
