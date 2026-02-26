package frc.robot.commands;

import frc.robot.subsystems.StorageSub;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class StorageRun extends Command {
  private final StorageSub storage;

  public StorageRun(StorageSub m_sub) {
    storage = m_sub;
    addRequirements(m_sub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    storage.runMotor(6);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}