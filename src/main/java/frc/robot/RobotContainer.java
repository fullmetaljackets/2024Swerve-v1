// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.IntSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ArmToggle;
import frc.robot.commands.ElevatorLower;
import frc.robot.commands.ElevatorRaise;
import frc.robot.commands.PanToggle;
import frc.robot.commands.TriggerIn;
import frc.robot.commands.TriggerOut;
import frc.robot.commands.TriggerToSetpoint;
import frc.robot.commands.ZeroPigeon;
import frc.robot.commands.groups.ShooterIn;
import frc.robot.commands.groups.ShooterOut;
import frc.robot.commands.groups.ShooterOutAmp;
import frc.robot.commands.groups.ShooterOutAuto;
import frc.robot.commands.groups.ShooterOutSlow;
import frc.robot.commands.groups.ShooterOutTrap;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterOne;
import frc.robot.subsystems.ShooterPan;
import frc.robot.subsystems.ShooterTrigger;
import frc.robot.subsystems.ShooterTwo;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.20 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driveStick = new CommandXboxController(0); // My main driver joystick
  private final CommandXboxController copilotStick = new CommandXboxController(1); // My copilot joystick

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /* Subsystems */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public final Elevator s_Elevator = new Elevator();
  private final Arm s_Arm = new Arm();
  private final ShooterPan s_ShooterPan = new ShooterPan();
  private final ShooterOne s_ShooterOne = new ShooterOne();
  private final ShooterTwo s_ShooterTwo = new ShooterTwo();
  private final ShooterTrigger s_ShooterTrigger = new ShooterTrigger();
  private final SendableChooser<Command> autoChooser;

 
  // set to -1 to invert motors to drive for Red side.
  public IntSupplier allianceOriented = () -> {
        if (!DriverStation.getAlliance().isPresent()) { return 1; }
        return DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1;
    };

  
  // private Command runAuto = drivetrain.getAutoPath("Straight");

  

  public RobotContainer() {
    NamedCommands.registerCommand("shoooterChargeUp", new ShooterOut(s_ShooterOne, s_ShooterTwo).withTimeout(.5));
    NamedCommands.registerCommand("Shooter", new ShooterOutAuto(s_ShooterOne, s_ShooterTwo, s_ShooterTrigger).withTimeout(.5));
    NamedCommands.registerCommand("zeroGyro", new ZeroPigeon(drivetrain).withTimeout(.1));

    drivetrain.configNeutralMode(NeutralModeValue.Coast);
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveStick.getLeftY() * MaxSpeed * allianceOriented.getAsInt()) // Drive forward with negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed * allianceOriented.getAsInt()) // Drive left with negative X (left)
            .withRotationalRate(-driveStick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    
    drivetrain.registerTelemetry(logger::telemeterize);
    
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
        

  }

  private void configureBindings() {
    /********************
     *  DRIVER Controls *
     ********************/
    //driveStick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driveStick.back().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driveStick.getLeftY(), -driveStick.getLeftX()))));
    driveStick.rightTrigger(0.1).whileTrue(drivetrain.applyRequest(() -> brake)); // Just a test to see if we can use a trigger
    // reset the field-centric heading on left bumper press
    driveStick.leftStick().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Shooter in
    driveStick.leftBumper().whileTrue(new ShooterIn(s_ShooterOne, s_ShooterTwo, s_ShooterTrigger).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // Shoot Trap
    driveStick.b().whileTrue(new ShooterOutTrap(s_ShooterOne, s_ShooterTwo, s_ShooterTrigger).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // Shoot Amp
    driveStick.rightBumper().whileTrue(new ShooterOutAmp(s_ShooterOne, s_ShooterTwo, s_ShooterTrigger).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // Trigger In
    driveStick.x().whileTrue(new TriggerIn(-0.25, s_ShooterTrigger).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // Trigger Out
    driveStick.y().whileTrue(new TriggerOut(1, s_ShooterTrigger).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // DPAD definitions
    driveStick.povUp().whileTrue(drivetrain.applyRequest(() -> 
            drive.withVelocityX(0.25 * MaxSpeed * allianceOriented.getAsInt()) // Drive forward 25% of MaxSpeed (forward)
            .withVelocityY(0 * MaxSpeed * allianceOriented.getAsInt()) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));
    driveStick.povDown().whileTrue(drivetrain.applyRequest(() -> 
            drive.withVelocityX(-0.25 * MaxSpeed * allianceOriented.getAsInt()) // Drive forward 25% of MaxSpeed (forward)
            .withVelocityY(0 * MaxSpeed * allianceOriented.getAsInt()) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));
    driveStick.povLeft().whileTrue(drivetrain.applyRequest(() -> 
            drive.withVelocityX(0 * MaxSpeed * allianceOriented.getAsInt()) // Drive forward 25% of MaxSpeed (forward)
            .withVelocityY(0.25 * MaxSpeed * allianceOriented.getAsInt()) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));
    driveStick.povRight().whileTrue(drivetrain.applyRequest(() -> 
            drive.withVelocityX(0 * MaxSpeed * allianceOriented.getAsInt()) // Drive forward 25% of MaxSpeed (forward)
            .withVelocityY(-0.25 * MaxSpeed * allianceOriented.getAsInt()) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));


    /*********************
     *  COPILOT Controls *
     *********************/
    // Elevator
    copilotStick.b().whileTrue(new ElevatorRaise(1, s_Elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    copilotStick.a().whileTrue(new ElevatorRaise(-1, s_Elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // Arm
    copilotStick.x().toggleOnTrue(new ArmToggle(s_Arm));
    // Pan
    copilotStick.y().toggleOnTrue(new PanToggle(s_ShooterPan));
    // Shooter
    //Copilot Bumpers/Triggers
    copilotStick.leftTrigger().whileTrue(new ShooterOut(s_ShooterOne, s_ShooterTwo).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    copilotStick.rightTrigger().whileTrue(new TriggerIn(-0.1, s_ShooterTrigger).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // final JoystickButton shooterOut =new JoystickButton(copilotStick, XboxController.Button.kRightBumper.value);
    // shooterOut.onTrue(new ShooterOut(s_ShooterOne, s_ShooterTwo).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // shooterOut.onFalse(new ShooterOutStop(s_S hooterOne, s_ShooterTwo));
    copilotStick.leftBumper().whileTrue(new ShooterOutSlow(s_ShooterOne, s_ShooterTwo));

    copilotStick.povUp().whileTrue(new ElevatorRaise(1, s_Elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    copilotStick.povDown().whileTrue(new ElevatorLower(-1, s_Elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    copilotStick.rightBumper().onTrue(new TriggerToSetpoint(0, s_ShooterTrigger));
    copilotStick.povRight().onTrue(new TriggerToSetpoint(1, s_ShooterTrigger));
    copilotStick.povLeft().onTrue(new TriggerToSetpoint(20, s_ShooterTrigger));



    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    driveStick.back().and(driveStick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driveStick.back().and(driveStick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driveStick.start().and(driveStick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driveStick.start().and(driveStick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

  }


  public Command getAutonomousCommand() {
    //return new PathPlannerAuto("Straight");
    // return Commands.print("No autonomous command configured");
    // PathPlannerPath  path = PathPlannerPath.fromPathFile("Test path 1");

    // return new PathPlannerAuto("Straight");

    // return runAuto;
    return autoChooser.getSelected();
    
  }
}
