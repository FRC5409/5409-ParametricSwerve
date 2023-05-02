package frc.robot.subsystems.swerveModules;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;

public class SwerveDrive extends SubsystemBase {

    private final SwerveModule mod_topLeft;
    private final SwerveModule mod_botLeft;
    private final SwerveModule mod_topRight;
    private final SwerveModule mod_botRight;

    private final Pigeon2 s_gyro;
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

        s_gyro = new Pigeon2(Swerve.kGyroID);
        s_gyro.

    }

    public SwerveModule[] getModules() {
        SwerveModule[] modules = {mod_topLeft, mod_botLeft, mod_topRight, mod_botRight};
        return modules;
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