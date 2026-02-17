package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;

public class ShooterLeftRun extends Command {

  private final Shooter shooter;
  private final DoubleSupplier input;

  public ShooterLeftRun(Shooter shooter, DoubleSupplier controllerInput) {
    this.shooter = shooter;
    this.input = controllerInput;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double rpm = 5000.0 * input.getAsDouble();
    shooter.setRPM(ShooterSide.LEFT, rpm);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop(ShooterSide.LEFT);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
