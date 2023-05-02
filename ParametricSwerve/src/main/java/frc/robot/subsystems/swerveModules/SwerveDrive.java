package frc.robot.subsystems.swerveModules;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;

public class SwerveDrive extends SubsystemBase {

    private final SwerveModule mod_topLeft;
    private final SwerveModule mod_botLeft;
    private final SwerveModule mod_topRight;
    private final SwerveModule mod_botRight;

    public SwerveDrive() {
        mod_topLeft = new SwerveModule(Swerve.RotationMotors.ID.TL_ID, 
            Swerve.RotationMotors.ID.CANCoder_ID, Swerve.Wheels.ID.TL_ID);
        mod_botLeft = new SwerveModule(Swerve.RotationMotors.ID.BL_ID, 
            Swerve.RotationMotors.ID.CANCoder_ID, Swerve.Wheels.ID.BL_ID);
        mod_topRight = new SwerveModule(Swerve.RotationMotors.ID.TR_ID, 
            Swerve.RotationMotors.ID.CANCoder_ID, Swerve.Wheels.ID.TR_ID);
        mod_botRight = new SwerveModule(Swerve.RotationMotors.ID.BR_ID, 
            Swerve.RotationMotors.ID.CANCoder_ID, Swerve.Wheels.ID.BR_ID);
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