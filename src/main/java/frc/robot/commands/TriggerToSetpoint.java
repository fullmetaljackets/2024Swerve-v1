// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ShooterTrigger;

public class TriggerToSetpoint extends Command {
  private final ShooterTrigger m_ShooterTrigger;
  private double m_setpoint;
  /** Creates a new Arm. */
  public TriggerToSetpoint(double setpoint, ShooterTrigger subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterTrigger = subsystem;
    m_setpoint = setpoint;
    addRequirements(m_ShooterTrigger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterTrigger.setMy_TriggerPos(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
