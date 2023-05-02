package frc.robot.commands.swerve;

import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.swerveModules.REVRotator;
import frc.robot.subsystems.swerveModules.SwerveDrive;
import frc.robot.subsystems.swerveModules.SwerveModule;
import frc.robot.subsystems.swerveModules.Wheels;

public class SwerveDriveCommand extends CommandBase {

    private final SwerveDrive sys_drive;
    private final CommandXboxController c_joystick;

    private final SwerveModule mod_1, mod_2, mod_3, mod_4;

    public SwerveDriveCommand(SwerveDrive subsystem, CommandXboxController joystick) {
        sys_drive = subsystem;
        c_joystick = joystick;
        // Use addRequirements() here to declare subsystem dependencies.
        mod_1 = sys_drive.getModules()[0];
        mod_2 = sys_drive.getModules()[1];
        mod_3 = sys_drive.getModules()[2];
        mod_4 = sys_drive.getModules()[3];
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
        double directionX = c_joystick.getLeftX() * Swerve.Wheels.MotionProfiling.kMaxWheelVel;
        double directionY = c_joystick.getLeftY() * Swerve.Wheels.MotionProfiling.kMaxWheelVel;

        if ((directionX >= 0.05 || directionX <= -0.05) || (directionY >= 0.05 || directionY <= -0.05)) { 
            double directionVel = Math.hypot(directionX, directionY);
            if (directionVel >= 1) {
                directionVel = 1;
            }   

            Rotation2d rotation = new Rotation2d(directionX, directionY);
            SwerveModuleState destinationState = new SwerveModuleState(directionVel, rotation);

            SwerveModuleState tl_moveState = SwerveModuleState.optimize(destinationState, mod_1.getCurrentAngleRad());
            SwerveModuleState tr_moveState = SwerveModuleState.optimize(destinationState, mod_2.getCurrentAngleRad());
            SwerveModuleState bl_moveState = SwerveModuleState.optimize(destinationState, mod_3.getCurrentAngleRad());
            SwerveModuleState br_moveState = SwerveModuleState.optimize(destinationState, mod_4.getCurrentAngleRad());
        } else {
            SwerveModuleState tl_moveState = new SwerveModuleState(0, mod_1.getCurrentAngleRad());
            SwerveModuleState tr_moveState = new SwerveModuleState(0, mod_2.getCurrentAngleRad());
            SwerveModuleState bl_moveState = new SwerveModuleState(0, mod_3.getCurrentAngleRad());
            SwerveModuleState br_moveState = new SwerveModuleState(0, mod_4.getCurrentAngleRad());
        }


        

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