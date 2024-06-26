package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Shooter1Out;
import frc.robot.commands.Shooter2Out;
import frc.robot.subsystems.ShooterOne;
import frc.robot.subsystems.ShooterTwo;

public class ShooterOutSlow extends ParallelCommandGroup {

    public ShooterOutSlow(ShooterOne s_ShooterOne, ShooterTwo s_ShooterTwo){

        addCommands(
            //shooter motor 1&2 out
            new Shooter1Out(-.80, s_ShooterOne),
            new Shooter2Out(-.80, s_ShooterTwo)
        );
    }
}
