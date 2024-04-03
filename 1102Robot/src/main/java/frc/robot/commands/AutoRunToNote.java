package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class AutoRunToNote extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    Limelight m_limelight;
    SwerveRequest.RobotCentric m_forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    double tx = 0;
    double omegaSpeed;

    boolean m_isFinished = false;

    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(.1, .01);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(.1, 0, 0, OMEGA_CONSTRATINTS);

    /** Creates a new aimAtNote. */
    public AutoRunToNote(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        omegaController.setTolerance(1);
        omegaController.setGoal(0);
        SmartDashboard.putData("Note PID",omegaController);
        addRequirements(drivetrain);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putData("Note Detect PID",omegaController);
        omegaController.reset(tx);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        tx = LimelightHelpers.getTX("limelight-note");
        omegaSpeed = omegaController.calculate(tx);
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        if (m_limelight.seesNote()) {
            m_drivetrain.setControl(m_forwardStraight
                    .withVelocityX(Constants.SwerveDrivetrain.maxSpeed * (1 - Math.abs(tx) / 32) * .5) // Constants.halfSpeed
                    .withVelocityY(omegaSpeed)
                    .withRotationalRate(0));
        }
        if (DriverStation.getAlliance().get() != null && DriverStation.isAutonomousEnabled()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                if (m_drivetrain.getState().Pose.getX() > Constants.FieldConstants.BLUE_AUTO_PENALTY_LINE) {
                    m_isFinished = true;
                }
            } else {
                if (m_drivetrain.getState().Pose.getX() < Constants.FieldConstants.RED_AUTO_PENALTY_LINE) {
                    m_isFinished = true;
                }
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(m_forwardStraight
            .withVelocityX(0) // Constants.halfSpeed
            .withVelocityY(0)
            .withRotationalRate(0));        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}