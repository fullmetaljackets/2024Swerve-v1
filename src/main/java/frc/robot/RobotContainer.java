// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driveStick = new CommandXboxController(0); // My main driver joystick
  private final CommandXboxController copilotStick = new CommandXboxController(1); // My copilot joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  
  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveStick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driveStick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driveStick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driveStick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driveStick.getLeftY(), -driveStick.getLeftX()))));
    driveStick.rightTrigger(0.1).whileTrue(drivetrain.applyRequest(() -> brake)); // Just a test to see if we can use a trigger

    // DPAD definitions
    driveStick.povUp().whileTrue(drivetrain.applyRequest(() -> 
            drive.withVelocityX(0.25 * MaxSpeed) // Drive forward 25% of MaxSpeed (forward)
            .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));
    driveStick.povDown().whileTrue(drivetrain.applyRequest(() -> 
            drive.withVelocityX(-0.25 * MaxSpeed) // Drive forward 25% of MaxSpeed (forward)
            .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));
    driveStick.povLeft().whileTrue(drivetrain.applyRequest(() -> 
            drive.withVelocityX(0 * MaxSpeed) // Drive forward 25% of MaxSpeed (forward)
            .withVelocityY(0.25 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));
    driveStick.povRight().whileTrue(drivetrain.applyRequest(() -> 
            drive.withVelocityX(0 * MaxSpeed) // Drive forward 25% of MaxSpeed (forward)
            .withVelocityY(0.25 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));

    // reset the field-centric heading on left bumper press
    driveStick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    driveStick.back().and(driveStick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driveStick.back().and(driveStick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driveStick.start().and(driveStick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driveStick.start().and(driveStick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
