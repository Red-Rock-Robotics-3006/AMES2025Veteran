package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class FirewoodFeeder extends SubsystemBase{
    private static FirewoodFeeder instance = null;
    
    private final RedRockTalon firewoodFeederMotor = new RedRockTalon(51, "firewoodFeederMotor", "*"); 

    private SmartDashboardNumber firewoodOuttakeSpeed = new SmartDashboardNumber("FirewoodFeeder/firewoodScorerSpeed", 0.8);
    private SmartDashboardNumber firewoodIntakeSpeed = new SmartDashboardNumber("FirewoodFeeder/firewoodIntakeSpeed", -0.4);
    
    private FirewoodFeeder() {
        super("FirewoodFeeder");
        
        this.firewoodFeederMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0) //TODO
            .withKS(0) //TODO
            .withKV(0) //TODO
            .withKP(0.05) //TODO
            .withKI(0) //TODO
            .withKD(0) //TODO
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()

            .withMotionMagicAcceleration(1300)
            .withMotionMagicCruiseVelocity(100)
        )
        .withSpikeThreshold(28)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);
    }

    public void setIntakeSpeedRPM(){ //Alines wood in conveyor
        this.firewoodFeederMotor.motor.set(this.firewoodIntakeSpeed.getNumber());
        // this.firewoodFeederMotor.setMotionMagicVelocity(this.firewoodIntakeSpeed.getNumber());
    }
    
    public void setOuttakeSpeedRPM(){
        this.firewoodFeederMotor.motor.set(this.firewoodOuttakeSpeed.getNumber());
        //this.firewoodFeederMotor.setMotionMagicVelocity(this.firewoodOuttakeSpeed.getNumber());
    }

    public void stopFirewoodFeeder(){
        this.firewoodFeederMotor.motor.set(0);
    }


    public Command startFirewoodIntake(){ //Alines firewood in conveyor
        return Commands.runOnce(() -> this.setIntakeSpeedRPM(), this);
    }

    public Command stopFirewoodIntake(){
        return Commands.runOnce(() -> this.stopFirewoodFeeder(), this);
    }

    public Command startFirewoodOuttake(){
        return Commands.runOnce(() -> this.setOuttakeSpeedRPM(), this);
    }

    @Override
    public void periodic(){
        this.firewoodFeederMotor.update();
    }

    public static FirewoodFeeder getInstance(){
        if(instance == null)
            instance = new FirewoodFeeder();
        return instance;
    }


}
