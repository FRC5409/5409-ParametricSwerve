package frc.robot.subsystems.swerveModules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;

public class Wheels extends SubsystemBase {

    private final CANSparkMax m_wheels;
    private final SparkMaxPIDController c_pidController;

    private final RelativeEncoder s_encoder;

    public Wheels(int motor_ID) {
        m_wheels = new CANSparkMax(motor_ID, MotorType.kBrushless);
        m_wheels.restoreFactoryDefaults();
        m_wheels.setIdleMode(IdleMode.kBrake);
        m_wheels.setSmartCurrentLimit(Swerve.Wheels.kCurrentLim);
        m_wheels.burnFlash();

        c_pidController = m_wheels.getPIDController();
        configPID();
        c_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        c_pidController.setSmartMotionMaxAccel(Swerve.Wheels.MotionProfiling.kMaxWheelAccel, 0);
        c_pidController.setSmartMotionMaxVelocity(Swerve.Wheels.MotionProfiling.kMaxWheelVel, 0);
        c_pidController.setSmartMotionMinOutputVelocity(Swerve.Wheels.MotionProfiling.kMinWheelVel, 0);
        c_pidController.setSmartMotionAllowedClosedLoopError(Swerve.Wheels.MotionProfiling.kError, 0);

        s_encoder = m_wheels.getEncoder();
        s_encoder.setVelocityConversionFactor(Swerve.Wheels.kVelConversionFactor);
        zeroEncoder();
    }

    public void configPID() {
        c_pidController.setP(Constants.Swerve.Wheels.MotionProfiling.kP, 0);
        c_pidController.setI(Constants.Swerve.Wheels.MotionProfiling.kI, 0);
        c_pidController.setD(Constants.Swerve.Wheels.MotionProfiling.kD, 0);
        c_pidController.setFF(Constants.Swerve.Wheels.MotionProfiling.kF, 0);
    }

    public void zeroEncoder() {
        s_encoder.setPosition(0);
    }

    public double getSpeed() {
        return s_encoder.getVelocity();
    }

    public void setSpeed(double vel) {
        c_pidController.setReference(vel, ControlType.kVelocity, 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}