package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StorageConstants;

public class StorageSub extends SubsystemBase {
    TalonFX motor;

    /**
     * This subsytem that controls the arm.
     */
    public StorageSub() {
        motor = new TalonFX(StorageConstants.motorID);
        
        var motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 55;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    } 

    @Override
    public void periodic() {}
    /** 
     * This is a method that makes the arm move at your desired speed
     *  Positive values make it spin forward and negative values spin it in reverse
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runMotor(double speed){
        motor.set(speed);
    }
}