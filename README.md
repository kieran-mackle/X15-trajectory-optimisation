# X-15 Trajectory Optimisation Framework

## Directory Overview
An overview of directory structure is as follows.

**home_dir/**: contains main.m, the master script to run a GPOPS-II optimisation problem. The 
        input scripts are specified from within Main.m.
        
**home_dir/aero**: Contains aerodynamic force coefficient databases for different vehicle 
    configurations.

**home_dir/functions**: Contains various function scripts which are called during optimisation.

**home_dir/inputs**: Manoeuvre_Scripts: contains the scripts which specify the manoeuvre
        being optimised.
    Objective_Functions: contains the objective functions for optimisation.
    Vehicle_Configurations: contains the vehicle configuration information.

**home_dir/Results**: a structured results directory containing folders of case results. New
    folders will be added to this directory after running the plotting
    scripts.

## General Running Procedure

1. Specify load scripts in main.m,
2. Run main.m,
3. Run post-processing script,
4. Run plotting script.


### Running 6DOF Optimisation

The key working files associated with the 6DOF dynamics model *tensor6* are:
- Dynamics: tensor6.m
- Manoeuvre: Climb6
- Objective: MinTime6
- Post-processing: post6.m
- plotting: plot6.m


## Example Optimisation Problems

The following examples may be run using the manoeuvre and objective scripts 
saved in the "Archive" directories of the respective input. 

Minimum time to climb 15km to 25km:
    - Vehicle: Config1.m
    - Manoeuvre: Climb15to25.m
    - Dynamics: Ellip6DOF.m
    - Objective: MinTimeToClimb.m
    - Post-processing: Post6.m
    - Plotting: SixPlot.m

Altitude hold: (barometric hold)
    - Vehicle: Config1.m
    - Manoeuvre: Hold20km.m
    - Dynamics: Ellip6DOF.m
    - Objective: HoldAltitude.m
    - Post-processing: Post6.m
    - Plotting: SixPlot.m

Minimum time to climb 15km to 25km with trimmed boundary conditions:
    - Vehicle: Config1.m
    - Manoeuvre: Climb15to25.m       ----- I think this can just be Climb
    - Dynamics: Ellip6DOF.m
    - Objective: MinTimeToClimbTrimmed.m
    - Post-processing: Post6.m
    - Plotting: SixPlot.m



## Works in progress

Extended minimum time to climb 15km to 25km:
    - Vehicle: Config1.m
    - Manoeuvre: 
    - Dynamics: nPhaseEllip6DOF.m
    - Objective: 
    - Post-processing: Post6.m
    - Plotting: SixPlot.m 
This requires a lot of work to add the extra phases to the dynamics
function. Might also not be necessary to do this case if I can just trim
at the boundaries.

Trim forces: (Objective function to minimise force integral)
    - Vehicle: Config1.m
    - Manoeuvre: Hold20kmV2.m
    - Dynamics: Ellip6DOF.m
    - Objective: HoldAltitudeV2.m
    - Post-processing: Post6.m
    - Plotting: SixPlot.m


