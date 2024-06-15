package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;



/**
 *
 */
public class ZeroPigeon extends Command {

    private final CommandSwerveDrivetrain s_Swerve;
 

    public ZeroPigeon(CommandSwerveDrivetrain subsystem) {

        s_Swerve = subsystem;
        addRequirements(s_Swerve);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Zero pigeon start");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Swerve.seedFieldRelative();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       System.out.println("Zero pigeon end");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
