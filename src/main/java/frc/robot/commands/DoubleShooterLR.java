package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSide;

public class DoubleShooterLR extends Command {

  private final Shooter shooter;
  private final DoubleSupplier leftInput;
  private final DoubleSupplier rightInput;


  public DoubleShooterLR(Shooter shooter, DoubleSupplier L_Input, DoubleSupplier R_Input) {
    this.shooter = shooter;
    this.leftInput = L_Input;
    this.rightInput = R_Input;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double leftRPM = 5000.0 * leftInput.getAsDouble();
    double rightRPM = 5000.0 * rightInput.getAsDouble();

    shooter.setRPM(ShooterSide.LEFT, leftRPM);
    shooter.setRPM(ShooterSide.RIGHT, rightRPM);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop(ShooterSide.LEFT);
    shooter.stop(ShooterSide.RIGHT);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
