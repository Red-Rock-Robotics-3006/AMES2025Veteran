package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Shooter extends SubsystemBase {
    private static Shooter instance = null;

    private final RedRockTalon shooterMotor = new RedRockTalon(42, "shooterMotor", "*"); 
    private final RedRockTalon indexMotor = new RedRockTalon(52, "indexMotor", "*"); 

    private SmartDashboardNumber shooterSpeed = new SmartDashboardNumber("Shooter/shooterSpeed", 4800); 
    private SmartDashboardNumber indexSpeed  = new SmartDashboardNumber("Shooter/indexSpeed", 800); 
    //private SmartDashboardNumber indexMotorSpeed  = new SmartDashboardNumber("Shooter/indexMotorSpeed", 0.1); 
    //private SmartDashboardNumber shooterMotorSpeed  = new SmartDashboardNumber("Shooter/shooterMotorSpeed", 0.7); 

    private Shooter() {
        super("Shooter");
                
        this.shooterMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Coast)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0) //TODO
            .withKS(0) //TODO
            .withKV(0) //TODO
            .withKP(0.25) //TODO
            .withKI(0) //TODO
            .withKD(0) //TODO //Took out d value
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(850)
            .withMotionMagicCruiseVelocity(150)
            .withMotionMagicJerk(10000000)
        )
        .withSpikeThreshold(17)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);
                
        this.indexMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Coast)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0) //TODO
            .withKS(0) //TODO
            .withKV(0) //TODO
            .withKP(0.5) //TODO
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


    public void setIndexSpeedRPM(){
        this.indexMotor.setMotionMagicVelocity(this.indexSpeed.getNumber()); //.motor.set(indexMotorSpeed.getNumber());
    }

    public void stopIndex(){
        this.indexMotor.setMotionMagicVelocity(0); //.motor.set(0);
    }

    public void setShooterSpeedRPM(){
        this.shooterMotor.setMotionMagicVelocity(this.shooterSpeed.getNumber()); //.motor.set(shooterMotorSpeed.getNumber());
    }//motor.set(0.1);

    public void stopShooter(){

        this.shooterMotor.setMotionMagicVelocity(0); //.motor.set(0);
    }

    public boolean isAtShooterSpeed(){ //Checks if shooter is up to speed
        return (Math.abs(this.shooterSpeed.getNumber() - 
        shooterMotor.motor.getVelocity().getValueAsDouble()*60.0)) <= 100; //getVelocity() is in RPS so convert to RPM
    }

    public boolean isShooting(){
        return shooterMotor.motor.getVelocity().getValueAsDouble() > 0.1;
    }

    public Command startShooterCommand(){//Starts shooter and waits till at speed when it starts index
        return Commands.sequence(
            Commands.runOnce(() -> this.setShooterSpeedRPM(), this),
            Commands.waitSeconds(1),
            //Commands.waitUntil(() -> this.isAtShooterSpeed()),
            Commands.runOnce(() -> this.setIndexSpeedRPM(), this)
        );
    }

    public Command stopShooterCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.stopIndex(), this), 
            Commands.runOnce(() -> this.stopShooter(), this) 
        );
    }

    @Override
    public void periodic(){
        this.shooterMotor.update();
        this.indexMotor.update();

        SmartDashboard.putBoolean("at velocity", isAtShooterSpeed());
    }

    public static Shooter getInstance(){
        if(instance == null)
            instance = new Shooter();
        return instance;
    }
}
