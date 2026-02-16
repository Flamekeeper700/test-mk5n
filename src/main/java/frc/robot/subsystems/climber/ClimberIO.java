package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ClimberIO {
    void run(double speed);

    void setPosition(double targetHeightMeters);

    default void stop() {
        run(0.0);
    }

    double getHeight();

    double getHeightMeters();

    double getVelocityMetersPerSec();

    Trigger atHeight(double heightMeters, double toleranceMeters);

    void periodic();

}
