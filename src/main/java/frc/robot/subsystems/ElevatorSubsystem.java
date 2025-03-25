package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public boolean nearSetpoint(double setpoint, double position){
        return Math.abs(setpoint - position) < ElevatorConstants.kPositionTolerance;
    }

    public Command setPosition(int level, int mode){
        final double targetPosition;
        if (mode == 0){
        switch(level){
            case 1:
                targetPosition = ElevatorConstants.kLevel1Position;
                break;
            case 2:
                targetPosition = ElevatorConstants.kLevel2Position;
                break;
            case 3:
                targetPosition = ElevatorConstants.kLevel3Position;
                break;
            case 4:
                targetPosition = ElevatorConstants.kLevel4Position;
                break;
            default:
                targetPosition = 0;
            }
        } else if (mode == 1){
            switch(level){
                case 1:
                    targetPosition = ElevatorConstants.kLevel1Position;
                    break;
                case 2:
                    targetPosition = ElevatorConstants.kLevel2Position;
                    break;
                case 3:
                    targetPosition = ElevatorConstants.kLevel3Position;
                    break;
                case 4:
                    targetPosition = ElevatorConstants.kLevel4Position;
                    break;
                default:
                    targetPosition = 0;
                }
        } else{
            targetPosition = 0;
        }

        return Commands.runOnce(()->runWithPosition(targetPosition)).until(()->nearSetpoint(targetPosition, elevatorMotor.getRotorPosition().getValueAsDouble()));
    }
}
