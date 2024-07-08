package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
        MotorOutputConfig.NeutralMode = NeutralModeValue.Coast;
        cfg.withMotorOutput(MotorOutputConfig);
        TriggerMotor = new TalonFX(13, "");
        TriggerMotor.getConfigurator().apply(cfg);
        
        /* Configure gear ratio */
        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1; // 12.8 rotor rotations per mechanism rotation
        
        /* Configure Motion Magic */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 3; // 5 (mechanism) rotations per second cruise
        mm.MotionMagicAcceleration = 1; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        mm.MotionMagicJerk = 100;
        
        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output
        
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = TriggerMotor.getConfigurator().apply(cfg);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }


        // TrigerMotor = new CANSparkMax(13, MotorType.kBrushless);   
        // TrigerMotor.restoreFactoryDefaults();  
        // TrigerMotor.setInverted(false);
        // TrigerMotor.setIdleMode(IdleMode.kBrake);
        // TrigerMotor.burnFlash();
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
    public void setMy_TriggerPos(double setpoint){
        TriggerMotor.setControl(m_mmReq.withPosition(setpoint).withSlot(0));
      }
    
}

