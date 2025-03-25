package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private TalonFX elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorCanID);
    
    public ElevatorSubsystem(){
        // Configure elevator
        elevatorMotor.getConfigurator().apply(Configs.ElevatorConfigs.elevatorConfig);
        

    }
    
    public void runWithPosition(double position){
        elevatorMotor.setControl(new MotionMagicVoltage(position));
    }
}
