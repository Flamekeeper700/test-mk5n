package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.util.GridDistanceProcessing;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.Constants.PathingConstants;

public class CustomPathing extends Command {

    private final GridDistanceProcessing gdp = new GridDistanceProcessing(PathingConstants.hubMap, PathingConstants.hubAngles);

    private final CommandSwerveDrivetrain swerve;

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* ------------ Tunables ------------ */

    private static final double MAX_SPEED = 4.5;     // m/s
    private static final double MIN_SPEED = 0.6;     // don’t stall near goal
    private static final double ROT_KP    = 3.5;     // heading follow

    public CustomPathing(CommandSwerveDrivetrain drivetrain) {
        this.swerve = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Pathing/Running", true);
    }

    @Override
    public void execute() {

        Pose2d currentPose = swerve.getState().Pose;
        Logger.recordOutput("Pathing/CurrentPose", currentPose);

        int heat = gdp.heatAt(currentPose);

        if (heat < 0) {
            stop();
            Logger.recordOutput("Pathing/Valid", false);
            return;
        }

        Logger.recordOutput("Pathing/Valid", true);
        Logger.recordOutput("Pathing/Heat", heat);

        /* ---------- Flow direction ---------- */

        Rotation2d flowHeading = gdp.flowDirection(currentPose);
        double dirX = Math.cos(flowHeading.getRadians());
        double dirY = Math.sin(flowHeading.getRadians());

        Logger.recordOutput("Pathing/FlowHeadingDeg",
                Math.toDegrees(flowHeading.getRadians()));

        /* ---------- Speed from heat ---------- */

        // far from goal = high heat = faster
        double speed = Math.min(MAX_SPEED,
                        Math.max(MIN_SPEED, heat * 0.08));
        speed += 1;
        speed *= -1;

        double vx = dirX * speed;
        double vy = dirY * speed;

        /* ---------- Face direction of travel ---------- */

        double headingError =
            flowHeading.minus(currentPose.getRotation()).getRadians();

        double omega = headingError * ROT_KP;

        Logger.recordOutput("Pathing/Vx", vx);
        Logger.recordOutput("Pathing/Vy", vy);
        Logger.recordOutput("Pathing/Omega", omega);

        swerve.setControl(
            drive.withVelocityX(vx)
                 .withVelocityY(vy)
                 .withRotationalRate(omega)
        );
    }

    private void stop() {
        swerve.setControl(
            drive.withVelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false; // continuous path following
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Pathing/Running", false);
        stop();
    }
}
