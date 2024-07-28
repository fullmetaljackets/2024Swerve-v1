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
            new Shooter1Out(-.65, s_ShooterOne),
            new Shooter2Out(-.65, s_ShooterTwo)
        );
    }

    public ShooterOutSlow(ShooterOne s_ShooterOne, ShooterTwo s_ShooterTwo, double speed){

        addCommands(
            //shooter motor 1&2 out
            new Shooter1Out(speed, s_ShooterOne),
            new Shooter2Out(speed, s_ShooterTwo)
        );
    }

    public ShooterOutSlow(ShooterOne s_ShooterOne, double speedOne, ShooterTwo s_ShooterTwo, double speedTwo){

        addCommands(
            //shooter motor 1&2 out
            new Shooter1Out(speedOne, s_ShooterOne),
            new Shooter2Out(speedTwo, s_ShooterTwo)
        );
    }

}
