package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.swerveModules.SwerveModule;

public class SwerveDrive extends SubsystemBase {

    private final SwerveModule mod_topLeft;
    private final SwerveModule mod_botLeft;
    private final SwerveModule mod_topRight;
    private final SwerveModule mod_botRight;

    private final WPI_Pigeon2 s_gyro;
    private double angleHeading;

    public SwerveDrive() {
        mod_topLeft = new SwerveModule(Swerve.RotationMotors.ID.TL_ID, 
            Swerve.RotationMotors.ID.CANCoder_ID, Swerve.Wheels.ID.TL_ID);
        mod_botLeft = new SwerveModule(Swerve.RotationMotors.ID.BL_ID, 
            Swerve.RotationMotors.ID.CANCoder_ID, Swerve.Wheels.ID.BL_ID);
        mod_topRight = new SwerveModule(Swerve.RotationMotors.ID.TR_ID, 
            Swerve.RotationMotors.ID.CANCoder_ID, Swerve.Wheels.ID.TR_ID);
        mod_botRight = new SwerveModule(Swerve.RotationMotors.ID.BR_ID, 
            Swerve.RotationMotors.ID.CANCoder_ID, Swerve.Wheels.ID.BR_ID);

        s_gyro = new WPI_Pigeon2(Swerve.kGyroID);
        zeroHeading();
    }

    public SwerveModule[] getModules() {
        SwerveModule[] modules = {mod_topLeft, mod_botLeft, mod_topRight, mod_botRight};
        return modules;
    }

    public void setStates(SwerveModuleState... modules) {
        mod_topLeft.getRotator().setGoal(modules[0].angle.getDegrees());
        mod_topRight.getRotator().setGoal(modules[1].angle.getDegrees());
        mod_botLeft.getRotator().setGoal(modules[2].angle.getDegrees());
        mod_botRight.getRotator().setGoal(modules[3].angle.getDegrees());

        mod_topLeft.getWheel().setSpeed(modules[0].speedMetersPerSecond);
        mod_topRight.getWheel().setSpeed(modules[1].speedMetersPerSecond);
        mod_botLeft.getWheel().setSpeed(modules[2].speedMetersPerSecond);
        mod_botRight.getWheel().setSpeed(modules[3].speedMetersPerSecond);
    }

    public void zeroHeading() {
        s_gyro.reset();
    }

    public double getHeadingRad() {
        return Math.toRadians(angleHeading);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        angleHeading = s_gyro.getCompassHeading();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}