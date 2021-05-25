package frc.robot;

import edu.wpi.first.wpilibj.util.Units;
import frc.lib.PolynomialRegression;

public final class Constants {
    /*
    Throughout all code: All units are assumed to be in the following units:
        - Distance: Meters
        - Angle: Degrees
        - Time: Seconds
        - Speed: Meters/Second
        - Acceleration: Meters/Second^2
        - Angular Speed: Radians/Second
    
    Unless otherwise specified
    */

    public static final class RioPortConstants {

        public static enum PwmPorts {
            ePwmPort0(0), ePwmPort1(1), ePwmPort2(2), ePwmPort3(3), ePwmPort4(4), ePwmPort5(5), ePwmPort6(6), ePwmPort7(7), ePwmPort8(8), ePwmPort9(9);
            private int mPortNum;
            private PwmPorts(int PortNum) { this.mPortNum = PortNum; }
            public int getValue() { return mPortNum; }
        };

        public static enum DioPorts {
            eDioPoert0(0), eDioPort1(1), eDioPort2(2), eDioPort3(3), eDioPort4(4), eDioPort5(5), eDioPort6(6), eDioPort7(7), eDioPort8(8), eDioPort9(9);
            private int mPortNum;
            private DioPorts(int PortNum) { this.mPortNum = PortNum; }
            public int getValue() { return mPortNum; }
        };
        
        public static enum CanAddresses {
            eCanAddress0(0), eCanAddress1(1), eCanAddress2(2), eCanAddress3(3), eCanAddress4(4), eCanAddress5(5), eCanAddress6(6), eCanAddress7(7), eCanAddress8(8), eCanAddress9(9), eCanAddress10(10), eCanAddress11(11), eCanAddress12(12), eCanAddress13(13), eCanAddress14(14);
            private int mAddressNum;
            private CanAddresses(int AddressNum) { this.mAddressNum = AddressNum; }
            public int getValue() { return mAddressNum; }
        };

    }
    
    public static final class ControllerConstants {
        public static final int kControllerPrimaryId = 0;
        public static final int kControllerSecondaryId = 1;
    }
    
    public static final class PneumaticConstants {

        public static enum PcmDevices { 
            ePcmId0(0), ePcmId1(1);
            private int mPortNum;
            private PcmDevices(int PortNum) { this.mPortNum = PortNum; }
            public int getValue() { return mPortNum; }
        };
        public static final int kPcm0Id = PneumaticConstants.PcmDevices.ePcmId0.getValue();
        public static final int kPcm1Id = PneumaticConstants.PcmDevices.ePcmId1.getValue();

        public static enum PcmPortsPerPcmDevice { 
            ePcmPort0(0), ePcmPort1(1), ePcmPort2(2), ePcmPort3(3), ePcmPort4(4), ePcmPort5(5), ePcmPort6(6), ePcmPort7(7);
            private int mPortNum;
            private PcmPortsPerPcmDevice(int PortNum) { this.mPortNum = PortNum; }
            public int getValue() { return mPortNum; }
        };
        public static final int kPcmPort0 = PneumaticConstants.PcmPortsPerPcmDevice.ePcmPort0.getValue();
        public static final int kPcmPort1 = PneumaticConstants.PcmPortsPerPcmDevice.ePcmPort1.getValue();
        public static final int kPcmPort2 = PneumaticConstants.PcmPortsPerPcmDevice.ePcmPort2.getValue();
        public static final int kPcmPort3 = PneumaticConstants.PcmPortsPerPcmDevice.ePcmPort3.getValue();
        public static final int kPcmPort4 = PneumaticConstants.PcmPortsPerPcmDevice.ePcmPort4.getValue();
        public static final int kPcmPort5 = PneumaticConstants.PcmPortsPerPcmDevice.ePcmPort5.getValue();
        public static final int kPcmPort6 = PneumaticConstants.PcmPortsPerPcmDevice.ePcmPort6.getValue();
        public static final int kPcmPort7 = PneumaticConstants.PcmPortsPerPcmDevice.ePcmPort7.getValue();
    }

    public static final class LimelightConstants {
        public static final String kShooterTableName = "limelight-shooter";
        public static final String kIntakeTableName = "limelight-intake";
    }

    public static final class CollisionAvoidanceConstants {
        public static final int kCollisionUltrasonicId = RioPortConstants.PwmPorts.ePwmPort3.getValue();
        public static final double kUltrasonicValueToInches = 0.13469388; //(1.0 / 9.8) * 2.0 * 0.66
        public static final int kNumberOfDistanceSamples = 10;
        //TODO: Find good values for the below
        public static final double kCollisionStandoffSlowdownInches = 18.0;
        public static final double kReducedStandoffSlowdownInches = 12.0;
    }
    
    public static final class DriveConstants {
        public static final int kDriveMotorPortAId = RioPortConstants.CanAddresses.eCanAddress1.getValue();
        public static final int kDriveMotorPortBId = RioPortConstants.CanAddresses.eCanAddress2.getValue();
        public static final int kDriveMotorStarboardAId = RioPortConstants.CanAddresses.eCanAddress4.getValue();
        public static final int kDriveMotorStarboardBId = RioPortConstants.CanAddresses.eCanAddress5.getValue();

        public static final double kManualDriveRotationScaleFactor = 0.8;
        
        public static final double kWheelRadiusInches = 3.0;
        private static final double kWheelDiameterInches = 6.0;
        public static final double kWheelCircumferenceInches = kWheelDiameterInches * Math.PI;
        public static final double kTrackWidthInches = 19.125; //Distance in between wheels

        public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kWheelRadiusMeters = Units.inchesToMeters(kWheelRadiusInches);
        public static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);

        public static final double kLowGearRatio = 33.98; //number of motor rotations to a single drive wheel rotation
        public static final double kHighGearRatio = 9.05;
        
        //TODO: Find values for everything below
        //Epsilon is the absolute error from the target that PIDs will consider to acheive the target
        public static final double kDriveEpsilon = 0.1; 
        public static final double kRotateEpsilon = 3.0;
        public static final double[] kLinearGains = {4.2, 0.0, 0.0};
        public static final double[] kRotationalGains = {0.14, 0.0, 0.5};//{0.03, 0.05, 0.05};
        //public static final double kRampRateToMaxSpeed = 0.1;  // Sets the ramp rate for open loop control modes. This is the maximum rate at which the motor controller's output is allowed to change.  Time in seconds to go from 0 to full throttle.



        public static final double kMaxManualLinearAcceleration = 0.02;
        public static final double kMaxManualRotationAcceleration = 0.02;
        public static final double kThrottleDecelerationProportion = 0.8; 
        public static final double kRotationalDecelerationProportion = 0.8;

        public static final double kRegularMaxSpeed = 1.0; // Absolute speed range is 0.0 to 1.0
        public static final double kPrecisionMaxSpeed = 0.5;

        //Robot Characterization Variables
        public static final double ksVolts = 0.721;
        public static final double kvVoltSecondsPerMeter = 0.225;
        public static final double kaVoltSecondsSquaredPerMeter = -0.00603;
        public static final double kPDriveVel = 1.0;//Tuned from above values
        public static final double kMaxSpeed = 1.0;
        public static final double kMaxAcceleration = 1.0;
        public static final double kRamseteB = 1.0;
        public static final double kRamseteZeta = 1.0;

        public static final double kMaxVoltage = 10;

        public static final int kGearShiftForwardId = PneumaticConstants.kPcmPort4;
        public static final int kGearShiftReverseId = PneumaticConstants.kPcmPort5;
    }

    public static final class SuperstructureConstants {

        public static final double kDrivebaseTargetingEpsilon = 1.4;
        
        public static final class ShooterConstants {
            public static final int kShooterBallIntakeForwardId = PneumaticConstants.kPcmPort0;
            public static final int kShooterBallIntakeReverseId = PneumaticConstants.kPcmPort1;
    
            public static final int kLauncherMotorAId = RioPortConstants.CanAddresses.eCanAddress7.getValue();
            public static final int kLauncherMotorBId = RioPortConstants.CanAddresses.eCanAddress8.getValue();

            public static final int kLowLidarId = RioPortConstants.DioPorts.eDioPort5.getValue();
            public static final int kLowLidarMin = 5;
            public static final int kLowLidarMax = 400;
            //public static final int kHighLidarId = 6;
            public static final double kHighLidarHorizontalOffsetIn = 8.5;

            public static final int kPidId = 0;

            public static final double kMaxOutput = 1.0;
            public static final double kMinOutput = 0.0;
            private static final double kP = 0.00218;
            private static final double kI = 0.0;
            private static final double kD = 0.0;
            private static final double kFF = 0.00024;
            private static final double kIz = 0.0;
            public static final double[] kGains = {kP, kI, kD, kFF, kIz}; 
            public static final double kMaxVelocity = 5000;
            public static final double kLowAccelerationRPMPerSecond = 1150;
            public static final double kHighAccelerationRPMPerSecond = kMaxVelocity / 0.5;
            public static final double kVelocityAccelShiftThresholdRPM = 1200.0;
            public static final double kShooterSpeedEpsilon = 65.0;

            public static final int kDistanceIndex = 0;
            public static final int kRPMIndex = 1;

            public static final int kMinRPMCellIndex = 2;

            public static final double[][] kDistanceInToRpmLUT = {
                {50, 4450.0}, 
                {54, 4450.0},
                {66, 4200.0},
                {83, 4200.0},
                {100, 4250.0},
                {113, 4300.0},
                {138, 4475.0},
                {158, 4600.0},
                {180, 4800.0},
                {208, 5100.0}
            };

            //Consider Cubic Spline Interpolation if 2nd Degree Polynomial Regression isn't accurate enough
            // http://mathworld.wolfram.com/CubicSpline.html
            public static final PolynomialRegression kShooterPolynomial = new PolynomialRegression(kDistanceInToRpmLUT, 2);

            public static final double[][] kTyToDistanceIn = {
                {6.3, 90},
                {2.2, 105},
                {-2.3, 120},
                {-4, 134},
                {-6, 148},
                {-7.97, 163},
                {-9.36, 175},
                {-10.8, 187},
                {-11.65, 197},
                {-12.8, 209},
                {-13.6, 222},
                {-14.6, 234},
                {-15.7, 249},
                {-16.6, 264},
                {-17.5, 278},
                {-18, 286}
            };
            
            public static final PolynomialRegression kTyDistanceRegression = new PolynomialRegression(kTyToDistanceIn, 3);
        }  

        public static final class TurretConstants {
            public static final int kTurretMotorId = 11;

            public static final int kTurretEncoderDioId = 0;

            public static final double kManualControlScaleFactor = 0.25;
            public static final double[] kGains = {0.0, 0.0, 0.0};
            public static final double kTurretEpsilon = 2.0;
            public static final double kTurretDegreesPerEncoderRotation = 45.0; // 8 rot == 360 deg

            public static final double kMaxTx = 999.9;

            public static final double kPowerCellDiameterInches = 7.0;
            public static final double kPowerCellClearance = 2.0;

            public static final double kHighTargetWidthInches = 39.25;

            //Might further adjust turretSetpointInDegrees to a lower range than 180deg, depends on physical limitations
            public static final double kPhysicalTurretRotationLimit = 180; 
        }

        public static final class StorageConstants { 
            public static final int kTableMotorId = 10;
            public static final int kLidarId = 0;
        }

        public static final class IntakeConstants {
            public static final int kIntakeMotorId = RioPortConstants.PwmPorts.ePwmPort0.getValue();

            public static final double kIntakeSpeed = 0.5;

            //Pneumatics PCM A   
            public static final int kIntakeForwardId = PneumaticConstants.kPcmPort6;
            public static final int kIntakeReverseId = PneumaticConstants.kPcmPort7;

        }
    }

}
