package frc.robot.subsystems.swerveModules;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.Swerve.RotationMotors;

public class REVRotator extends ProfiledPIDSubsystem {

    private CANSparkMax m_rotator;
    private SparkMaxPIDController c_pidController;

    private CANCoder s_encoder;

    public REVRotator(int mot_ID, int encoder_ID) {
        super(
            new ProfiledPIDController(
                RotationMotors.MotionProfiling.kP, 
                RotationMotors.MotionProfiling.kI, 
                RotationMotors.MotionProfiling.kD, 
                new TrapezoidProfile.Constraints(
                    RotationMotors.MotionProfiling.maxVel, 
                    RotationMotors.MotionProfiling.maxAccel
                )
            ),
            encoder_ID);
        m_rotator = new CANSparkMax(mot_ID, MotorType.kBrushless);
        m_rotator.restoreFactoryDefaults();
        m_rotator.setSmartCurrentLimit(RotationMotors.kCurrentLim);
        m_rotator.setIdleMode(IdleMode.kCoast);
        m_rotator.burnFlash();

        c_pidController = m_rotator.getPIDController();
        configPID();

        s_encoder = new CANCoder(encoder_ID);
        s_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    }

    public double getPosition() {
        return s_encoder.getAbsolutePosition() + RotationMotors.MotionProfiling.kEncoderOffset;
    }

    public void configPID() {
        c_pidController.setP(RotationMotors.MotionProfiling.kP);
        c_pidController.setI(RotationMotors.MotionProfiling.kI);
        c_pidController.setD(RotationMotors.MotionProfiling.kD);
        c_pidController.setFF(RotationMotors.MotionProfiling.kF);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        
        if (output >= RotationMotors.MotionProfiling.maxOutput) {
            output = RotationMotors.MotionProfiling.maxOutput;
        } else if (output <= -RotationMotors.MotionProfiling.maxOutput) {
            output = -RotationMotors.MotionProfiling.maxOutput;
        }

        m_rotator.setVoltage(output);
    }

    @Override
    protected double getMeasurement() {
        
        return getPosition();
    }
    
    @Override
    public void periodic() {
        super.periodic();
        // This method will be called once per scheduler run
        
    }

}