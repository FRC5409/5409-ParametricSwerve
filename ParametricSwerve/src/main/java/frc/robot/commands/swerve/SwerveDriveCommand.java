package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerveModules.REVRotator;
import frc.robot.subsystems.swerveModules.SwerveDrive;
import frc.robot.subsystems.swerveModules.SwerveModule;
import frc.robot.subsystems.swerveModules.Wheels;

public class SwerveDriveCommand extends CommandBase {

    private final SwerveDrive sys_drive;

    public SwerveDriveCommand(SwerveDrive subsystem) {
        sys_drive = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        SwerveModule mod_1 = sys_drive.getModules()[0];
        SwerveModule mod_2 = sys_drive.getModules()[1];
        SwerveModule mod_3 = sys_drive.getModules()[2];
        SwerveModule mod_4 = sys_drive.getModules()[3];
        REVRotator rot_1 = mod_1.getRotator();
        REVRotator rot_2 = mod_2.getRotator();
        REVRotator rot_3 = mod_3.getRotator();
        REVRotator rot_4 = mod_4.getRotator();
        Wheels wheel_1 = mod_1.getWheel();
        Wheels wheel_2 = mod_2.getWheel();
        Wheels wheel_3 = mod_3.getWheel();
        Wheels wheel_4 = mod_4.getWheel();


        addRequirements(
            sys_drive,
            mod_1,
            mod_2,
            mod_3,
            mod_4,
            rot_1,
            rot_2,
            rot_3,
            rot_4,
            wheel_1,
            wheel_2,
            wheel_3,
            wheel_4
        );
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        
        return false;
    }

}