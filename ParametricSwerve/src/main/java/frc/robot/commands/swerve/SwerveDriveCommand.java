package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.swerveModules.REVRotator;
import frc.robot.subsystems.swerveModules.SwerveModule;
import frc.robot.subsystems.swerveModules.Wheels;

public class SwerveDriveCommand extends CommandBase {

    private final SwerveDrive sys_drive;
    private final CommandXboxController c_joystick;

    private final SwerveModule mod_topLeft, mod_topRight, mod_botLeft, mod_botRight;

    public SwerveDriveCommand(SwerveDrive subsystem, CommandXboxController joystick) {
        sys_drive = subsystem;
        c_joystick = joystick;
        // Use addRequirements() here to declare subsystem dependencies.
        mod_topLeft = sys_drive.getModules()[0];
        mod_topRight = sys_drive.getModules()[1];
        mod_botLeft = sys_drive.getModules()[2];
        mod_botRight = sys_drive.getModules()[3];
        REVRotator rot_1 = mod_topLeft.getRotator();
        REVRotator rot_2 = mod_topRight.getRotator();
        REVRotator rot_3 = mod_botLeft.getRotator();
        REVRotator rot_4 = mod_botRight.getRotator();
        Wheels wheel_1 = mod_topLeft.getWheel();
        Wheels wheel_2 = mod_topRight.getWheel();
        Wheels wheel_3 = mod_botLeft.getWheel();
        Wheels wheel_4 = mod_botRight.getWheel();


        addRequirements(
            sys_drive,
            mod_topLeft,
            mod_topRight,
            mod_botLeft,
            mod_botRight,
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
        double turnX = c_joystick.getRightX() * Swerve.Wheels.MotionProfiling.kMaxWheelVel;

        Translation2d tl_moveTranslation;
        Translation2d tr_moveTranslation;
        Translation2d bl_moveTranslation;
        Translation2d br_moveTranslation;
        
        Translation2d tl_turnTranslation;
        Translation2d tr_turnTranslation;
        Translation2d bl_turnTranslation;
        Translation2d br_turnTranslation;

        if ((directionX >= 0.05 || directionX <= -0.05) || (directionY >= 0.05 || directionY <= -0.05)) { 
            double directionVel = Math.hypot(directionX, directionY);

            double fieldAngle = Math.atan(directionY/directionX) * Math.PI/180 + sys_drive.getHeadingRad();

            Rotation2d rotation = new Rotation2d(fieldAngle);
            
            tl_moveTranslation = new Translation2d(directionVel, rotation);
            tr_moveTranslation = new Translation2d(directionVel, rotation);
            bl_moveTranslation = new Translation2d(directionVel, rotation);
            br_moveTranslation = new Translation2d(directionVel, rotation);

        } else {
            tl_moveTranslation = new Translation2d(0, mod_topLeft.getCurrentAngleRad());
            tr_moveTranslation = new Translation2d(0, mod_topRight.getCurrentAngleRad());
            bl_moveTranslation = new Translation2d(0, mod_botLeft.getCurrentAngleRad());
            br_moveTranslation = new Translation2d(0, mod_botRight.getCurrentAngleRad());
        }


        if (turnX >= 0.05 || turnX <= -0.05) {
            
            Rotation2d tl_angle = new Rotation2d(Math.atan((-Swerve.chassisY/2)/(Swerve.chassisX/2)) + Math.PI/2);
            Rotation2d tr_angle = new Rotation2d(Math.atan((-Swerve.chassisY/2)/(-Swerve.chassisX/2)) - Math.PI/2);
            Rotation2d bl_angle = new Rotation2d(Math.atan((Swerve.chassisY/2)/(Swerve.chassisX/2)) + Math.PI/2);
            Rotation2d br_angle = new Rotation2d(Math.atan((Swerve.chassisY/2)/(-Swerve.chassisX/2)) - Math.PI/2);

            tl_turnTranslation = new Translation2d(turnX, tl_angle);
            tr_turnTranslation = new Translation2d(-turnX, tr_angle);
            bl_turnTranslation = new Translation2d(turnX, bl_angle);
            br_turnTranslation = new Translation2d(-turnX, br_angle);

        } else {
            tl_turnTranslation = new Translation2d(0, mod_topLeft.getCurrentAngleRad());
            tr_turnTranslation = new Translation2d(0, mod_topRight.getCurrentAngleRad());
            bl_turnTranslation = new Translation2d(0, mod_botLeft.getCurrentAngleRad());
            br_turnTranslation = new Translation2d(0, mod_botRight.getCurrentAngleRad());
        }

        Translation2d tl_combinedVector = tl_moveTranslation.plus(tl_turnTranslation);
        Translation2d tr_combinedVector = tr_moveTranslation.plus(tr_turnTranslation);
        Translation2d bl_combinedVector = bl_moveTranslation.plus(bl_turnTranslation);
        Translation2d br_combinedVector = br_moveTranslation.plus(br_turnTranslation);

        SwerveModuleState tl_combinedState = SwerveModuleState.optimize(new SwerveModuleState(Math.hypot(tl_combinedVector.getX(), tl_combinedVector.getY()), tl_combinedVector.getAngle()), mod_topLeft.getCurrentAngleRad());
        SwerveModuleState tr_combinedState = SwerveModuleState.optimize(new SwerveModuleState(Math.hypot(tr_combinedVector.getX(), tr_combinedVector.getY()), tr_combinedVector.getAngle()), mod_topRight.getCurrentAngleRad());
        SwerveModuleState bl_combinedState = SwerveModuleState.optimize(new SwerveModuleState(Math.hypot(bl_combinedVector.getX(), bl_combinedVector.getY()), bl_combinedVector.getAngle()), mod_botLeft.getCurrentAngleRad());
        SwerveModuleState br_combinedState = SwerveModuleState.optimize(new SwerveModuleState(Math.hypot(br_combinedVector.getX(), br_combinedVector.getY()), br_combinedVector.getAngle()), mod_botRight.getCurrentAngleRad());

        SwerveModuleState[] states = {tl_combinedState, tr_combinedState, bl_combinedState, br_combinedState};

        sys_drive.setStates(states);

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