package frc.robot.subsystems;


import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 *
 */
public class ShooterTrigger extends SubsystemBase{ 
   
    // private CANSparkMax TrigerMotor;
    private TalonFX TriggerMotor;
    private TalonFXConfiguration cfg;
    private MotorOutputConfigs MotorOutputConfig;
    final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    // private final Mechanisms m_mechanisms = new Mechanisms();
    private int m_printCount = 0;
  

    /**
    *
    */
    public ShooterTrigger() {

        cfg = new TalonFXConfiguration();
        MotorOutputConfig = new MotorOutputConfigs();
        MotorOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        MotorOutputConfig.NeutralMode = NeutralModeValue.Brake;
        cfg.withMotorOutput(MotorOutputConfig);
        TriggerMotor = new TalonFX(13, "");
        TriggerMotor.getConfigurator().apply(cfg);
        
        /* Configure gear ratio */
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1; // 12.8 rotor rotations per mechanism rotation
        
    }

    @Override
    public void periodic() {
        if (m_printCount++ > 10) {
            m_printCount = 0;
            // System.out.println("Pos: " + TriggerMotor.getPosition());
            // System.out.println("Vel: " + TriggerMotor.getVelocity());
            // System.out.println();
            SmartDashboard.putNumber("position", TriggerMotor.getPosition().getValue());
            SmartDashboard.putNumber("Velocity", TriggerMotor.getVelocity().getValue());

          }
        //   m_mechanisms.update(TriggerMotor.getPosition(), TriggerMotor.getVelocity());
          }

    @Override
    public void simulationPeriodic() {

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void triggerMotorRun(double setpoint){
        TriggerMotor.set(setpoint);
        //DriverStation.reportError("******** TrigerMotor **************", false);
    }    
}

