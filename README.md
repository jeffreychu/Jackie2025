# Jackie2025 - 1880 Documentation


## CommandSwerveDrivetrain 

**Purpose: Default Subsystem which controls swerve drive, odometry, auto config.**

**Visit Subsystem If Encountering:**
- Gyro issues
- Desire to change Auto Configs

//Shouldn’t need to modify this subsystem much as default template works



## ElevatorSubsystem

**Purpose: Subsystem meant to control elevator positions and maintain it.**

**Visit Subsystem If Encountering:** 
- Elevator not reaching position(PID Controller Issue or increase position)
- Intake not lined up (Check elevator state position value)

**Details:**
- Elevator states & Elevator state position values
- Periodic logic for Elevators



## LEDController 

**Purpose: Subsystem meant to control CANdle and LED colors on robot.**

**Visit Subsystem If Encountering:**
- Desire to change colors at any state
- Not all LED glowing (check LED count)

**Details:**
- LED States and colors

//Shouldn’t need to seriously fix this unless messes with Driver logic  



## LimelightSubsystem

**Purpose:  Subsystem meant to control Limelight Configs and logic regarding lining up to reef and to coral feeder.**

**Visit Subsystem If Encountering:**
- Robot not lining up to either reef or coral feeder
- Robot odometry looks off (Fine tune Limelight Standard Deviations(How much the robot trusts Limelight data))

**Details:**
- Lining up logic 
- Standard Deviation values
- Limelight config



## ScoringSubsystem 

**Purpose: subsystem that runs coral shooter and algae intake/scorer**

**Visit Subsystem If Encountering:**
- Intaking issues with sensor
- Coral shooter not running

**Details:**
- Scoring subsystem state holds different states of the coral shooter
- Is not concerned with alignment or elevator height




## Robot 

**Purpose: run code on the robot based on state of the game**

**Visit Subsystem If Encountering:**
- Issues with limelight not being used

**Details:**
- Almost nothing is in this file except limelight. Could be useful to run config code when robot is disabled.

## RobotContainer

**Purpose: subsystem used to bind buttons to operator & drive controllers, instantiate subsystems and run auto nodes**

**Visit Subsystem If Encountering:**
- Controller & button related issues
- Logic issues regarding states
- Strange auto issues with path planner

**Details:**
- General controller for the robot and all subsystems
