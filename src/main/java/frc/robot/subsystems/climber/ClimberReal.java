package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;

public class ClimberReal implements ClimberIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final PIDController pid;
    private final ElevatorFeedforward feedforward;

    public ClimberReal() {
        motor = new SparkMax(0, MotorType.kBrushless);
        motor.setCANTimeout(250);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.voltageCompensation(12.0);
        motorConfig.smartCurrentLimit(ClimberConstants.kCurrentLimit);
        motorConfig.idleMode(IdleMode.kBrake);

        motor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        encoder = motor.getEncoder();

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
        motor.set(MathUtil.clamp(speed, -1.0, 1.0));
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
        return encoder.getPosition();
    }

    @Override
    public double getHeightMeters() {
        return encoder.getPosition()
            * ClimberConstants.kEncoderDistPerPulse;
    }

    @Override
    public double getVelocityMetersPerSec() {
        return encoder.getVelocity()
            * ClimberConstants.kEncoderDistPerPulse / 60.0;
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
        motor.set(MathUtil.clamp(output, -1.0, 1.0));
    }

    @Override
    public void periodic() {

    }
}
