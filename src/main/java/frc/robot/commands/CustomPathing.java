package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.ProfiledPIDController;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;

import frc.robot.util.GridDistanceProcessing;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CustomPathing extends Command {

    private final GridDistanceProcessing gdp = new GridDistanceProcessing();

    private Pose2d currentPose = new Pose2d();
    private Pose2d targetPose  = new Pose2d();

    private final CommandSwerveDrivetrain swerve;

    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final ProfiledPIDController translationController;
    private final ProfiledPIDController rotationController;

    public CustomPathing(
        ProfiledPIDController translation,
        ProfiledPIDController rotation,
        CommandSwerveDrivetrain drivetrain
    ) {
        this.translationController = translation;
        this.rotationController = rotation;

        translationController.setTolerance(Units.inchesToMeters(1.0));
        rotationController.setTolerance(Units.degreesToRadians(2.0));

        this.swerve = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        translationController.reset(0.0);
        rotationController.reset(0.0);

        Logger.recordOutput("Pathing/Running", true);
    }

    @Override
    public void execute() {
        Logger.recordOutput("Pathing/ExecuteAlive", true);
        currentPose = swerve.getState().Pose;
        Logger.recordOutput("Pathing/CurrentPose", currentPose);
        targetPose = gdp.bestAdjacent(currentPose);
        int heat = gdp.heatAt(currentPose);

        if (targetPose == null) {
            Logger.recordOutput("Pathing/TargetValid", false);
            swerve.setControl(
                drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
            );
            return;
        }

        Logger.recordOutput("Pathing/TargetValid", true);
        Logger.recordOutput("Pathing/TargetPose", targetPose);
        Logger.recordOutput("Pathing/Heat", heat);

        // ---- translation control ----
        Translation2d error =
            targetPose.getTranslation().minus(currentPose.getTranslation());

        double distance = error.getNorm();

        Translation2d direction =
            distance > 1e-4 ? error.div(distance) : new Translation2d();

        // drive speed toward zero distance
        double speed = -1 + Math.max(-(heat * .1), -4.5);

        double vx = direction.getX() * speed;
        double vy = direction.getY() * speed;

        // ---- rotation control ----
        double omega = rotationController.calculate(
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians()
        );

        Logger.recordOutput("Pathing/Vx", vx);
        Logger.recordOutput("Pathing/Vy", vy);
        Logger.recordOutput("Pathing/Omega", omega);

        // ---- send to drivetrain ----
        swerve.setControl(
            drive.withVelocityX(vx)
                 .withVelocityY(vy)
                 .withRotationalRate(omega)
        );
    }

    @Override
    public boolean isFinished() {
        return false; // continuous pathing
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Pathing/Running", false);

        swerve.setControl(
            drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
        );
    }
}
