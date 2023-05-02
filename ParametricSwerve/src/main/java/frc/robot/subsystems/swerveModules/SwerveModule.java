package frc.robot.subsystems.swerveModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

    private final REVRotator sys_rotator;
    private final Wheels sys_wheels;

    public SwerveModule(int rotator_ID, int encoder_ID, int wheel_ID) {
        sys_rotator = new REVRotator(rotator_ID, encoder_ID);
        sys_wheels = new Wheels(wheel_ID);
        Commands.runEnd(
        () -> {
            sys_rotator.setGoal(0);
            sys_rotator.enable();
        }, 
        () -> sys_rotator.disable(), 
        sys_rotator);
    }

    public REVRotator getRotator() {
        return sys_rotator;
    }

    public Wheels getWheel() {
        return sys_wheels;
    }

    public SwerveModuleState createModuleState(double velocityX, double velocityY) {
        Rotation2d rotationVector = new Rotation2d(velocityX, velocityY);
        return new SwerveModuleState(Math.hypot(velocityX, velocityY), rotationVector);
    }

    public void setState(SwerveModuleState state) {
        double velocity = state.speedMetersPerSecond;
        Rotation2d angle = state.angle;
        sys_wheels.setSpeed(velocity);
        sys_rotator.setGoal(angle.getDegrees());
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