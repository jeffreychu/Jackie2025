// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

    public static final int kDriverController = 0;
    public static final int kOperatorController = 0; // TODO change

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

            public static final double coralIntakePostion = 7.8;
            public static final double L1Position = 8.0;
            public static final double L2Position = 11.76;
            public static final double L3Position = 18.5;

            public static final double L3AlgaePosition = 15.0;

        }
    }

    public static class Scoring {
        public static class Sensors {
            public static final int leftSensorID = 0;
            public static final int rightSensorID = 1;
        }

        public class ScoringSpeeds {
            public static double coralIntake = -0.5;
            public static double coralOutake = -0.558;
            public static double coralOutake2 = -.2418;//- .2418; //-0.418;
            public static double coralAlgae = 0.7;
        }

        public static class Motors {
            // CORAL SHOOTER MOTOR
            public static final int scoringMotorID = 17;
        }
    }

    public static class Limelight {
        public static final boolean kUseLimelight = false;
        public static final String kLimelightName = "limelight";
    }

    public static class LED {
        public static final int CANdleID = 16;
    }

}