package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FirewoodFeeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import redrocklib.logging.SmartDashboardNumber;

public class Autos {
    private final AutoFactory factory;

    SmartDashboardNumber waitTimeMiddle = new SmartDashboardNumber("Autos/waitTimeMiddle", 0); //TODO
    SmartDashboardNumber waitTimeRight = new SmartDashboardNumber("Autos/waitTimeTop", 0); //TODO

    
    FirewoodFeeder firewoodFeeder = FirewoodFeeder.getInstance();
    Intake intake = Intake.getInstance();
    Shooter shooter = Shooter.getInstance();
    CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    
    public Autos(AutoFactory f, SwerveRequest.FieldCentric driveRequest){
        this.factory = f;
        this.drive = driveRequest;
    }

    // public Command preLoadleaveAuto(){
    //     return Commands.sequence(
    //         this.shooter.startShooterCommand(),
    //         Commands.waitSeconds(3),
    //         shooter.stopShooterCommand(),
    //         this.factory.trajectoryCmd("2CoralLeft", 0),
    //         firewoodFeeder.startFirewoodIntake()
    //     );
    // }
    
    private double getWaitTimeMiddle(){
        return waitTimeMiddle.getNumber()*1.0;
    }

    private double getWaitTimeRight(){
        return waitTimeRight.getNumber()*1.0;
    }

    private SwerveRequest.FieldCentric drive;

    public void setDriveRequestObject(SwerveRequest.FieldCentric driveRequest) {
        this.drive = driveRequest;
    }
    
    public Command leftPreloadLeaveAuto(){
        return Commands.sequence(
            // factory.resetOdometry("PreloadLeftLeave"),
            // Commands.runOnce(() -> 
            //     drivetrain.resetPose(
            //     factory.cache().loadTrajectory("PreloadLeftLeave", 0).get().getInitialPose(DriverStation.getAlliance().get().equals(Alliance.Red)).get()
            //     ) ),
            this.driveForwardSeconds(1),
            // this.factory.trajectoryCmd("PreloadLeftLeave", 0),
            this.shooter.startShooterCommand(),
            this.firewoodFeeder.startFirewoodOuttake(),
            Commands.waitSeconds(5),
            // this.factory.trajectoryCmd("PreloadLeftLeave", 1),
            this.shooter.stopShooterCommand(),
            this.firewoodFeeder.stopFirewoodIntake(),
            this.driveBackwardSeconds(8)
        

            );

            // Commands.runOnce(() -> {
            //     factory.resetOdometry("PreloadLeftLeave");
            //     // drivetrain.resetKalaman();
            //     drivetrain.resetPose(
            //         factory.cache().loadTrajectory(trajectoryName, index).get().getInitialPose(DriverStation.getAlliance().get().equals(Alliance.Red)).get()
            //         ); 
            // },
    }
    
    public Command rightLeaveAuto(){
        return this.driveForwardSeconds(8);
    }

    public Command middleLeaveAuto(){
        return this.driveForwardSeconds(9);
    }

    public Command driveForwardSeconds(double seconds) {
        return
            Commands.sequence(
                Commands.deadline(
                Commands.waitSeconds(seconds), 
                drivetrain.applyRequest(
                   () -> drive.withVelocityX(-1)
                    .withVelocityY(0)
                    .withRotationalRate(0) 
                )),
                Commands.deadline(
                    Commands.waitSeconds(0.1), 
                    drivetrain.applyRequest(
                        () -> drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                    ))
            ); 
    }

    public Command driveBackwardSeconds(double seconds) {
        return
            Commands.sequence(
                Commands.deadline(
                Commands.waitSeconds(seconds), 
                drivetrain.applyRequest(
                   () -> drive.withVelocityX(1)
                    .withVelocityY(0)
                    .withRotationalRate(0) 
                )),
                Commands.deadline(
                    Commands.waitSeconds(0.1), 
                    drivetrain.applyRequest(
                        () -> drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
                    ))
            ); 
    }
        
    public Command middleTopCabinAuto(){
        return Commands.sequence(
            factory.resetOdometry("MiddleTopCabin"),
            this.factory.trajectoryCmd("MiddleTopCabin", 0),
            this.factory.trajectoryCmd("MiddleTopCabin", 1),
            this.factory.trajectoryCmd("MiddleTopCabin", 2)
        );
    }

    public Command middleBottomCabinAuto(){
        return Commands.sequence(
            this.factory.trajectoryCmd("MiddleBottomCabin", 0),
            this.factory.trajectoryCmd("MiddleBottomCabin",1),
            this.factory.trajectoryCmd("MiddleBottomCabin", 2)
        );
    }

    public Command rightTopCabinAuto(){
        return Commands.sequence(
            this.factory.trajectoryCmd("RightTopCabin", 0),
            Commands.waitSeconds(3),
            this.factory.trajectoryCmd("RightTopCabin", 1),
            this.factory.trajectoryCmd("RightTopCabin", 2)
        );
    }

    public Command rightBottomCabinAuto(){
        return Commands.sequence(
            this.factory.trajectoryCmd("RightBottomCabin", 0),
            Commands.waitSeconds(3),
            this.factory.trajectoryCmd("RightBottomCabin", 1),
            this.factory.trajectoryCmd("RightBottomCabin", 2)
        );
    }

    // public Command rightLeaveAuto(){
    //     return Commands.sequence(
    //         factory.resetOdometry("RightLeave"),
    //         this.factory.trajectoryCmd("RightLeave", 0),
    //         this.factory.trajectoryCmd("RightLeave", 1)
    //     );
    // }

    public Command firewoodPreloadLeftAuto(){
        return Commands.sequence(
            this.factory.trajectoryCmd("FirewoodPreloadLeft",0),
            this.shooter.startShooterCommand(),
            this.firewoodFeeder.startFirewoodOuttake(),
            Commands.waitSeconds(3),
            this.shooter.stopShooterCommand(),
            this.firewoodFeeder.stopFirewoodIntake(),
            this.factory.trajectoryCmd("FirewoodPreloadLeft", 1),
            this.firewoodFeeder.startFirewoodIntake(),
            Commands.waitSeconds(2),
            this.firewoodFeeder.stopFirewoodIntake(),
            this.factory.trajectoryCmd("FirewoodPreloadLeft", 2),
            this.firewoodFeeder.startFirewoodOuttake(),
            Commands.waitSeconds(1),
            this.firewoodFeeder.stopFirewoodIntake(),
            this.factory.trajectoryCmd("FirewoodPreloadLeft", 3)
        );
    }

    public Command middlePreloadBottomAuto(){
        return Commands.sequence(
            Commands.waitSeconds(getWaitTimeMiddle()),
            this.factory.trajectoryCmd("MiddlePreloadBottomCabin", 0),
            this.shooter.startShooterCommand(),
            this.firewoodFeeder.startFirewoodOuttake(),
            Commands.waitSeconds(2),
            this.shooter.stopShooterCommand(),
            this.firewoodFeeder.stopFirewoodIntake(),
            this.factory.trajectoryCmd("MiddlePreloadBottomCabin", 1),
            this.factory.trajectoryCmd("MiddlePreloadBottomCabin", 2),
            this.factory.trajectoryCmd("MiddlePreloadBottomCabin", 3)
        );
    }

    public Command middlePreloadTopAuto(){
        return Commands.sequence(
            Commands.waitSeconds(getWaitTimeMiddle()),
            this.factory.trajectoryCmd("MiddlePreloadTopCabin", 0),
            this.shooter.startShooterCommand(),
            this.firewoodFeeder.startFirewoodOuttake(),
            Commands.waitSeconds(2),
            this.shooter.stopShooterCommand(),
            this.firewoodFeeder.stopFirewoodIntake(),
            this.factory.trajectoryCmd("MiddlePreloadTopCabin", 1),
            this.factory.trajectoryCmd("MiddlePreloadTopCabin", 2),
            this.factory.trajectoryCmd("MiddlePreloadTopCabin", 3)
            );
        }
        
        public Command rightPreloadBottomAuto(){
            return Commands.sequence(
                Commands.waitSeconds(getWaitTimeRight()),
                this.factory.trajectoryCmd("RightPreloadBottomCabin", 0),
                this.shooter.startShooterCommand(),
                this.firewoodFeeder.startFirewoodOuttake(),
                Commands.waitSeconds(2),
                this.shooter.stopShooterCommand(),
                this.firewoodFeeder.stopFirewoodIntake(),
                this.factory.trajectoryCmd("RightPreloadBottomCabin", 1),
                this.factory.trajectoryCmd("RightPreloadBottomCabin", 2),
                this.factory.trajectoryCmd("RightPreloadBottomCabin", 3)
            );
        }

        public Command rightPreloadTopAuto(){
            return Commands.sequence(
                Commands.waitSeconds(getWaitTimeRight()),
                this.factory.trajectoryCmd("RightPreloadTopCabin", 0),
                this.shooter.startShooterCommand(),
                this.firewoodFeeder.startFirewoodOuttake(),
                Commands.waitSeconds(2),
                this.shooter.stopShooterCommand(),
                this.firewoodFeeder.stopFirewoodIntake(),
                this.factory.trajectoryCmd("RightPreloadTopCabin", 1),
                this.factory.trajectoryCmd("RightPreloadTopCabin", 2),
                this.factory.trajectoryCmd("RightPreloadTopCabin", 3)
            );
        }
}
