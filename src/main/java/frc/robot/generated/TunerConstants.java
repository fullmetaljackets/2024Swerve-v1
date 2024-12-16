package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.ReplanningConfig;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import static frc.robot.Config.CONFIG;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.CommandSwerveDrivetrain;

public class TunerConstants {
	public static final double g = 9.81;
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 80.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 2.5;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.122448979591837;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "";
    private static final int kPigeonId = 0;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Module 0
    // Front Left
    private static final int kFrontLeftDriveMotorId = 6;
    private static final int kFrontLeftSteerMotorId = 5;
//     private static final int kFrontLeftDriveMotorId = 2;
//     private static final int kFrontLeftSteerMotorId = 1;
//     private static final int kFrontLeftEncoderId = 21;
    private static final int kFrontLeftEncoderId = 23;
    private static final double kFrontLeftEncoderOffset = -0.381591796875;
//     private static final double kFrontLeftEncoderOffset = 0.287353515625;

    private static final double kFrontLeftXPosInches = 11.3125;
    private static final double kFrontLeftYPosInches = 11.3125;

    //Module 1
    // Front Right
    private static final int kFrontRightDriveMotorId = 8;
    private static final int kFrontRightSteerMotorId = 7;
//     private static final int kFrontRightDriveMotorId = 4;
//     private static final int kFrontRightSteerMotorId = 3;
//     private static final int kFrontRightEncoderId = 22;
    private static final int kFrontRightEncoderId = 24;
    private static final double kFrontRightEncoderOffset = 0.336181640625;
//     private static final double kFrontRightEncoderOffset = -0.19873046875;

    private static final double kFrontRightXPosInches = 11.3125;
    private static final double kFrontRightYPosInches = -11.3125;

    // Module 2
    // Back Left
    private static final int kBackLeftDriveMotorId = 4;
    private static final int kBackLeftSteerMotorId = 3;
//     private static final int kBackLeftDriveMotorId = 8;
//     private static final int kBackLeftSteerMotorId = 7;
//     private static final int kBackLeftEncoderId = 24;
    private static final int kBackLeftEncoderId = 22;
    private static final double kBackLeftEncoderOffset = -0.19873046875;
//     private static final double kBackLeftEncoderOffset = 0.336181640625;

    private static final double kBackLeftXPosInches = -11.3125;
    private static final double kBackLeftYPosInches = 11.3125;

    //Moduel 3
    // Back Right
    private static final int kBackRightDriveMotorId = 2;
    private static final int kBackRightSteerMotorId = 1;
//     private static final int kBackRightDriveMotorId = 6;
//     private static final int kBackRightSteerMotorId = 5;
//     private static final int kBackRightEncoderId = 23;
    private static final int kBackRightEncoderId = 21;
    private static final double kBackRightEncoderOffset = 0.287353515625;
//     private static final double kBackRightEncoderOffset = -0.381591796875;

    private static final double kBackRightXPosInches = -11.3125;
    private static final double kBackRightYPosInches = -11.3125;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);


    public static final class Limelightc {
		public static final String INTAKE_LL_NAME = "limelight-intake";
		public static final String SHOOTER_LL_NAME = "limelight-shooter";

		public static final int[] VALID_IDS = { 4, 7 };

		public static final double STD_DEV_X_METERS = 0.7; // uncertainty of 0.7 meters on the field
		public static final double STD_DEV_Y_METERS = 0.7; // uncertainty of 0.7 meters on the field
		public static final int STD_DEV_HEADING_RADS = 9999999; // (gyro) heading standard deviation, set extremely high
																// to represent unreliable heading
		public static final int MAX_TRUSTED_ANG_VEL_DEG_PER_SEC = 720; // maximum trusted angular velocity

		public static final double ERROR_TOLERANCE_RAD = 0.1;
		public static final double HORIZONTAL_FOV_DEG = 0; // unused
		public static final double RESOLUTION_WIDTH_PIX = 640; // unused
		public static final double MOUNT_ANGLE_DEG_SHOOTER = 55.446;
		public static final double MOUNT_ANGLE_DEG_INTAKE = -29;
		public static final double HEIGHT_FROM_GROUND_METERS_SHOOTER = Units.inchesToMeters(8.891);
		public static final double HEIGHT_FROM_GROUND_METERS_INTAKE =
				Units.inchesToMeters(13);
		public static final double ARM_TO_OUTTAKE_OFFSET_DEG = 115; // unused
		public static final double NOTE_HEIGHT = Units.inchesToMeters(2); // unused
		public static final double MIN_MOVEMENT_METERSPSEC = 1.5;
		public static final double MIN_MOVEMENT_RADSPSEC = 0.5;
		public static final double HEIGHT_FROM_RESTING_ARM_TO_SPEAKER_METERS = Units.inchesToMeters(65.5675);
		public static final double SIDEWAYS_OFFSET_TO_OUTTAKE_MOUTH = Units.inchesToMeters(10.911);
		public static final double END_EFFECTOR_BASE_ANGLE_RADS = Units.degreesToRadians(75);
		public static final double VERTICAL_OFFSET_FROM_ARM_PIVOT = Units.inchesToMeters(3.65); // unused
		public static final class Apriltag {
			public static final int RED_SPEAKER_CENTER_TAG_ID = 4;
			public static final int BLUE_SPEAKER_CENTER_TAG_ID = 7;
			public static final double SPEAKER_CENTER_HEIGHT_METERS = Units.inchesToMeters(60.125);
			public static final double HEIGHT_FROM_BOTTOM_TO_SUBWOOFER = Units.inchesToMeters(26);
			public static final double HEIGHT_FROM_BOTTOM_TO_ARM_RESTING = Units.inchesToMeters(21.875);
		}
	}

    public static final class Drivetrainc {

		// #region Subsystem Constants

		public static final double wheelBase = CONFIG.isSwimShady() ? Units.inchesToMeters(19.75)
				: Units.inchesToMeters(16.75);
		public static final double trackWidth = CONFIG.isSwimShady() ? Units.inchesToMeters(28.75)
				: Units.inchesToMeters(23.75);
		// "swerveRadius" is the distance from the center of the robot to one of the
		// modules
		public static final double swerveRadius = Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));
		// The gearing reduction from the drive motor controller to the wheels
		// Gearing for the Swerve Modules is 6.75 : 1
		public static final double driveGearing = 6.75;
		// Turn motor shaft to "module shaft"
		public static final double turnGearing = 150 / 7;

		public static final double driveModifier = 1;
		public static final double wheelDiameterMeters = Units.inchesToMeters(4.0) * 7.36 / 7.65 /*
																									 * empirical
																									 * correction
																									 */;
		public static final double mu = 1; /* 70/83.2; */

		public static final double NEOFreeSpeed = 5676 * (2 * Math.PI) / 60; // radians/s
		// Angular speed to translational speed --> v = omega * r / gearing
		public static final double maxSpeed = NEOFreeSpeed * (wheelDiameterMeters / 2.0) / driveGearing;
		public static final double maxForward = maxSpeed; // todo: use smart dashboard to figure this out
		public static final double maxStrafe = maxSpeed; // todo: use smart dashboard to figure this out
		// seconds it takes to go from 0 to 12 volts(aka MAX)
		public static final double secsPer12Volts = 0.1;

		// maxRCW is the angular velocity of the robot.
		// Calculated by looking at one of the motors and treating it as a point mass
		// moving around in a circle.
		// Tangential speed of this point mass is maxSpeed and the radius of the circle
		// is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
		// Angular velocity = Tangential speed / radius
		public static final double maxRCW = maxSpeed / swerveRadius;

		public static final boolean[] reversed = { false, false, false, false };
		// public static final boolean[] reversed = {true, true, true, true};
		// Determine correct turnZero constants (FL, FR, BL, BR)
		public static final double[] turnZeroDeg = RobotBase.isSimulation() ? new double[] {-90.0, -90.0, -90.0, -90.0 }
				: (CONFIG.isSwimShady() ? new double[] { 85.7812, 85.0782, -96.9433, -162.9492 }
						: new double[] { -48.6914, 63.3691, 94.1309, -6.7676 });/* real values here */

		// kP, kI, and kD constants for turn motor controllers in the order of
		// front-left, front-right, back-left, back-right.
		// Determine correct turn PID constants
		public static final double[] turnkP = { 51.078, 60.885, 60.946, 60.986 }; // {0.00374, 0.00374, 0.00374,
																					// 0.00374};
		public static final double[] turnkI = { 0, 0, 0, 0 };
		public static final double[] turnkD = { 0/* dont edit */, 0.5, 0.42, 1 }; // todo: use d
		// public static final double[] turnkS = {0.2, 0.2, 0.2, 0.2};
		public static final double[] turnkS = { 0.13027, 0.17026, 0.2, 0.23262 };

		// V = kS + kV * v + kA * a
		// 12 = 0.2 + 0.00463 * v
		// v = (12 - 0.2) / 0.00463 = 2548.596 degrees/s
		public static final double[] turnkV = { 2.6532, 2.7597, 2.7445, 2.7698 };
		public static final double[] turnkA = { 0.17924, 0.17924, 0.17924, 0.17924 };

		// kP is an average of the forward and backward kP values
		// Forward: 1.72, 1.71, 1.92, 1.94
		// Backward: 1.92, 1.92, 2.11, 1.89
		// Order of modules: (FL, FR, BL, BR)
		public static final double[] drivekP = CONFIG.isSwimShady() ? new double[] { 2.8, 2.8, 2.8, 2.8 }
				: new double[] { 1.75, 1.75, 1.75, .75 }; // {1.82/100, 1.815/100, 2.015/100,
																			// 1.915/100};
		public static final double[] drivekI = { 0, 0, 0, 0 };
		public static final double[] drivekD = { 0, 0, 0, 0 };
		public static final boolean[] driveInversion = (CONFIG.isSwimShady()
				? new boolean[] { false, false, false, false }
				: new boolean[] { true, false, true, false });
		public static final boolean[] turnInversion = { true, true, true, true };
		// kS
		public static final double[] kForwardVolts = { 0.26744, 0.31897, 0.27967, 0.2461 };
		public static final double[] kBackwardVolts = kForwardVolts;

		public static final double[] kForwardVels = { 2.81, 2.9098, 2.8378, 2.7391 };
		public static final double[] kBackwardVels = kForwardVels;
		public static final double[] kForwardAccels = { 1.1047 / 2, 0.79422 / 2, 0.77114 / 2, 1.1003 / 2 };
		public static final double[] kBackwardAccels = kForwardAccels;

		public static final double autoMaxSpeedMps = 0.35 * 4.4; // Meters / second
		public static final double autoMaxAccelMps2 = mu * g; // Meters / seconds^2
		public static final double autoMaxVolt = 10.0; // For Drivetrain voltage constraint in RobotPath.java
		// The maximum acceleration the robot can achieve is equal to the coefficient of
		// static friction times the gravitational acceleration
		// a = mu * 9.8 m/s^2
		public static final double autoCentripetalAccel = mu * g * 2;

		public static final boolean isGyroReversed = true;

		// PID values are listed in the order kP, kI, and kD
		public static final double[] xPIDController = CONFIG.isSwimShady() ? new double[] { 4, 0.0, 0.0 }
				: new double[] { 2, 0.0, 0.0 };
		public static final double[] yPIDController = xPIDController;
		public static final double[] thetaPIDController = CONFIG.isSwimShady() ? new double[] { 0.10, 0.0, 0.001 }
				: new double[] {0.05, 0.0, 0.00};


		// public static final Limelight.Transform limelightTransformForPoseEstimation =
		// Transform.BOTPOSE_WPIBLUE;

		// #endregion

		// #region Ports

		public static final int driveFrontLeftPort = CONFIG.isSwimShady() ? 8 : 11; //
		public static final int driveFrontRightPort = CONFIG.isSwimShady() ? 13 : 19; //
		public static final int driveBackLeftPort = CONFIG.isSwimShady() ? 5 : 14; //
		public static final int driveBackRightPort = CONFIG.isSwimShady() ? 11 : 17; // correct

		public static final int turnFrontLeftPort = CONFIG.isSwimShady() ? 7 : 12; //
		public static final int turnFrontRightPort = CONFIG.isSwimShady() ? 14 : 20; // 20
		public static final int turnBackLeftPort = CONFIG.isSwimShady() ? 6 : 15; //
		public static final int turnBackRightPort = CONFIG.isSwimShady() ? 12 : 16; // correct

		public static final int canCoderPortFL = CONFIG.isSwimShady() ? 4 : 0;
		public static final int canCoderPortFR = CONFIG.isSwimShady() ? 2 : 3;
		public static final int canCoderPortBL = CONFIG.isSwimShady() ? 3 : 2;
		public static final int canCoderPortBR = CONFIG.isSwimShady() ? 1 : 1;

		// #endregion

		// #region Command Constants

		public static double kNormalDriveSpeed = 1; // Percent Multiplier
		public static double kNormalDriveRotation = 0.5; // Percent Multiplier
		public static double kSlowDriveSpeed = 0.4; // Percent Multiplier
		public static double kSlowDriveRotation = 0.250; // Percent Multiplier

		//baby speed values, i just guessed the percent multiplier. TODO: find actual ones we wana use
		public static double kBabyDriveSpeed = 0.3;
		public static double kBabyDriveRotation = 0.2;
		public static double kAlignMultiplier = 1D / 3D;
		public static final double kAlignForward = 0.6;

		public static final double wheelTurnDriveSpeed = 0.0001; // Meters / Second ; A non-zero speed just used to
																	// orient the wheels to the correct angle. This
																	// should be very small to avoid actually moving the
																	// robot.

		public static final double[] positionTolerance = { Units.inchesToMeters(.5), Units.inchesToMeters(.5), 5 }; // Meters,
																													// Meters,
																													// Degrees
		public static final double[] velocityTolerance = { Units.inchesToMeters(1), Units.inchesToMeters(1), 5 }; // Meters,
																													// Meters,
																													// Degrees/Second

		// #endregion
		// #region Subsystem Constants

		// "swerveRadius" is the distance from the center of the robot to one of the
		// modules
		public static final double turnkP_avg = (turnkP[0] + turnkP[1] + turnkP[2] + turnkP[3]) / 4;
		public static final double turnIzone = .1;

		public static final double driveIzone = .1;

		public static final class Autoc {
			public static final ReplanningConfig replanningConfig = new ReplanningConfig( /*
																							 * put in
																							 * Constants.Drivetrain.Auto
																							 */
					false, // replan at start of path if robot not at start of path?
					false, // replan if total error surpasses total error/spike threshold?
					1.5, // total error threshold in meters that will cause the path to be replanned
					.8 // error spike threshold, in meters, that will cause the path to be replanned
			);
			public static final PathConstraints pathConstraints = new PathConstraints(1.54, 6.86, 2 * Math.PI,
					2 * Math.PI); // The constraints for this path. If using a differential drivetrain, the
									// angular constraints have no effect.
		}
	}	
}
