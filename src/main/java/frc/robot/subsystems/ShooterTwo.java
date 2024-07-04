package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Top motor
 */
public class ShooterTwo extends SubsystemBase{ 
   
    // private CANSparkMax Shooter3Motor;
    private TalonFX Shooter2Motor;
    private TalonFXConfiguration TalonFXConfig;
    private MotorOutputConfigs MotorOutputConfig;



    /**
    * 
    */
    public ShooterTwo() {
        TalonFXConfig = new TalonFXConfiguration();
        MotorOutputConfig = new MotorOutputConfigs();
        MotorOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        MotorOutputConfig.NeutralMode = NeutralModeValue.Coast;
        TalonFXConfig.withMotorOutput(MotorOutputConfig);
        Shooter2Motor = new TalonFX(12, "");
        // Shooter3Motor.setInverted(false);
        // Shooter3Motor.setNeutralMode(NeutralModeValue.Coast);
        Shooter2Motor.getConfigurator().apply(TalonFXConfig);


        // Shooter2Motor = new CANSparkMax(12, MotorType.kBrushless);
        // Shooter2Motor.restoreFactoryDefaults();  
        // Shooter2Motor.setInverted(false);
        // Shooter2Motor.setIdleMode(IdleMode.kCoast);
        // Shooter2Motor.burnFlash();
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void shooter2MotorRun(double setpoint){
        // Shooter2Motor.set(setpoint);
        Shooter2Motor.set(setpoint);
        //DriverStation.reportError("******** TrigerMotor **************", false);
    }

}


