// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

    public static final int kDriverController = 0;
    public static final int kOperatorController = 1; // TODO change
    
    public static final double driverRumbleValue = 0.60;

    /******* SWERVE DRIVE MOTORS ******/
    public static class DriveMotors {
        // DRIVE MOTOR ID'S
        public static final int frontRightDriveID = 2;
        public static final int frontLeftDriveID = 5;
        public static final int backRightDriveID = 11;
        public static final int backLeftDriveID = 8;

        // STEER MOTOR ID'S
        public static final int frontRightSteerID = 1;
        public static final int frontLeftSteerID = 4;
        public static final int backRightSteerID = 10;
        public static final int backLeftSteerID = 7;
    }

    
	// Auto constants
    public static class Auto {
    
        public static final double X_REEF_ALIGNMENT_P = 3.3;
	    public static final double Y_REEF_ALIGNMENT_P = 3.3;
        //public static final double ROT_REEF_ALIGNMENT_P = 0.058;

        //public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
        //public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
        public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34;  // Vertical pose
        public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
        public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;  // Horizontal pose
        public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

        public static final double DONT_SEE_TAG_WAIT_TIME = 1;
        public static final double POSE_VALIDATION_TIME = 0.3;

    }



    /******* ELEVATOR SUBSYSTEM ********/
    public static class Elevator {
        public static final double positionThreshold = 0.01;
        public static class Motors {
            // ELEVATOR MOTOR ID'S
            public static final int elevatorLeftID = 13;
            public static final int elevatorRightID = 14;

        }
        public static class Sensors {
            // SENSORS ID, port is on the roborio
            public static final int elevatorSensor = 0;

        }

        public class ElevatorStatePositions {
            public static final double leftHomePostion = 0.0;
            public static final double rightHomePostion = 0.0;

            public static final double trackingPosition = 2.5; //1.50;
            public static final double coralIntakePostion = 7.8;
            public static final double L1Position = 9.0;
            public static final double L2Position = 11.76;
            public static final double L3Position = 18.3;

            public static final double L3AlgaePosition = 15.0;

        }
    }

    /******* SCORING SUBSYSTEM *******/

    public static class Scoring {
        public static class Sensors {
            public static final int leftSensorID = 0;
            public static final int rightSensorID = 1;
        }

        public class ScoringSpeeds {
            public static double coralIntake = -0.4;//-0.5
            public static double coralOutake = -0.2418;
            public static double coralOutake2 = -0.2018;

            
            public static double coralOutake2Auto = -.2018;//- .2418; //-0.418;
            public static double coralAlgae = 0.7;
        }

        public static class Motors {
            // CORAL SHOOTER MOTOR
            public static final int scoringMotorID = 17;
        }
    }

    public static class SwerveConstants {
        public static final double kP = 0.10;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static class Limelight {
        public static final boolean kUseLimelight = false;
        public static final String kLimelightName = "limelight";
    }

    public static class LED {
        public static final int CANdleID = 16;
    }

        /********* ALGAE SUBSYSTEM ***********/
        public static class Algae {
            
            public static final int algaeMotorID = 20;
           //t public static final String AlgaeSpeeds = null;

            public class AlgaeSpeeds {
                public static double algaeIntake1 = -0.2018;
                public static double algaeOutake = 0.2018;
    
                
              //  public static double coralOutake2Auto = -.2018;//- .2418; //-0.418;
             //   public static double coralAlgae = 0.7;
            }

            
            }
        
            
        


        /******* CLIMBER SUBSYSTEM ********/
        public static class Climber {
            public static final double positionThreshold = 0.01;
            public static class Motors {
                // CLIMBER MOTOR ID'S
                public static final int climberMotorID = 20; //set value
    
            }
    
            public static class Sensors {
                // SENSORS ID, port is on the roborio
                public static final int climberSensor = 1;
    
            }
    
            public class ClimberStatePositions {
                //set values
                public static final double motorHomePosition = 0.0; 
                public static final double motorTargetPosition = 7.4;
            }
        }

}