package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;

public class ClimberSim implements ClimberIO {

    private final ElevatorSim sim;
    private final PIDController pid;
    private final ElevatorFeedforward feedforward;

    private double appliedOutput = 0.0;

    public ClimberSim() {
        sim = new ElevatorSim(
            DCMotor.getNEO(1),               // motor model (change if needed)
            ClimberConstants.kGearing,
            ClimberConstants.kCarriageMass,
            ClimberConstants.kDrumRadius,
            ClimberConstants.kMinHeight,
            ClimberConstants.kMaxHeight,
            true,
            0,
            0.01,
            0.0);


        pid = new PIDController(
            ClimberConstants.kP,
            ClimberConstants.kI,
            ClimberConstants.kD
        );
        pid.setTolerance(0.05);

        feedforward = new ElevatorFeedforward(
            ClimberConstants.kS,
            ClimberConstants.kG,
            ClimberConstants.kV,
            ClimberConstants.kA
        );
    }

    @Override
    public void run(double speed) {
        appliedOutput = MathUtil.clamp(speed, -1.0, 1.0);
    }

    @Override
    public Trigger atHeight(double heightMeters, double toleranceMeters) {
        return new Trigger(() ->
            MathUtil.isNear(
                heightMeters,
                getHeightMeters(),
                toleranceMeters
            )
        );
    }

    @Override
    public double getHeight() {
        return getHeightMeters();
    }

    @Override
    public double getHeightMeters() {
        return sim.getPositionMeters();
    }

    @Override
    public double getVelocityMetersPerSec() {
        return sim.getVelocityMetersPerSecond();
    }

    @Override
    public void setPosition(double targetHeightMeters) {
        double pidOut = pid.calculate(
            getHeightMeters(),
            targetHeightMeters
        );

        double ffVolts = feedforward.calculate(
            getVelocityMetersPerSec()
        );

        double output = pidOut + (ffVolts / 12.0);

        if (getHeight() <= 0 && output < 0) {
            output = 0;
        } 
        if (getHeight() >= ClimberConstants.kMaxHeight && output > 0) {
            output = 0;
        }

        appliedOutput = MathUtil.clamp(output, -1.0, 1.0);
    }

    @Override
    public void periodic() {
        sim.setInputVoltage(appliedOutput * 12.0);
        sim.update(0.02);
    }
}
