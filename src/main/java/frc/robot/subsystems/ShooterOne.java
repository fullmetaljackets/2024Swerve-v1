package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Bottom motor
 */
public class ShooterOne extends SubsystemBase{ 
   
    private TalonFX Shooter3Motor;
    private TalonFXConfiguration TalonFXConfig;
    private MotorOutputConfigs MotorOutputConfig;

    /**
    * 
    */
    public ShooterOne() {
        TalonFXConfig = new TalonFXConfiguration();
        MotorOutputConfig = new MotorOutputConfigs();
        MotorOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        MotorOutputConfig.NeutralMode = NeutralModeValue.Coast;
        TalonFXConfig.withMotorOutput(MotorOutputConfig);
        Shooter3Motor = new TalonFX(11, "");
        // Shooter3Motor.setInverted(false);
        // Shooter3Motor.setNeutralMode(NeutralModeValue.Coast);
        Shooter3Motor.getConfigurator().apply(TalonFXConfig);
    }
    
    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void shooter1MotorRun(double setpoint){
        Shooter3Motor.set(setpoint);
        //DriverStation.reportError("******** TrigerMotor **************", false);
    }

}


