// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.FirewoodFeeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import redrocklib.logging.SmartDashboardNumber;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

    private SmartDashboardNumber maxAngular = new SmartDashboardNumber("Max Turn", 0.75);
  
    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(maxAngular.getNumber()).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private SmartDashboardNumber MaxSpeed = new SmartDashboardNumber("Max Drive Speed", 6);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.getNumber() * 0.04).withRotationalDeadband(RotationsPerSecond.of(maxAngular.getNumber()).in(RadiansPerSecond) * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController drivestick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
  
  private final AutoFactory autoFactory;
  private final Autos autos;
  
  private final FirewoodFeeder firewoodFeeder = FirewoodFeeder.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final LED led = LED.getInstance();

  private SendableChooser <Command> autoChooser = new SendableChooser<>();
  
  public RobotContainer() {
    autoFactory = drivetrain.createAutoFactory();
    autos = new Autos(autoFactory, drive);
    
    configureBindings();
    configureSelector();
  }

  private void configureSelector(){
    autoChooser.setDefaultOption("No Auto", Commands.print("No auto"));

    autoChooser.addOption("leftPreloadLeaveAuto", autos.leftPreloadLeaveAuto());
    autoChooser.addOption("rightLeaveAuto", autos.rightLeaveAuto());
    autoChooser.addOption("middleLeaveAuto", autos.middleLeaveAuto());
    // autoChooser.addOption("middleTopCabinAuto", autos.middleTopCabinAuto());
    // autoChooser.addOption("middleBottomCabinAuto", autos.middleBottomCabinAuto());
    // autoChooser.addOption("rightTopCabinAuto", autos.rightTopCabinAuto());
    // autoChooser.addOption("rightBottomCabinAuto", autos.rightBottomCabinAuto());
    // autoChooser.addOption("firewoodPreloadLeftAuto", autos.firewoodPreloadLeftAuto());
    // autoChooser.addOption("middlePreloadBottomAuto", autos.middlePreloadBottomAuto());
    // autoChooser.addOption("middlePreloadTopAuto", autos.middlePreloadTopAuto());
    // autoChooser.addOption("rightPreloadBottomAuto", autos.rightPreloadBottomAuto());
    // autoChooser.addOption("rightPreloadTopAuto", autos.rightPreloadTopAuto());

    SmartDashboard.putData("AUTO CHOOSER", autoChooser);
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-drivestick.getLeftY() * MaxSpeed.getNumber()) // Drive forward with negative Y (forward)
                    .withVelocityY(-drivestick.getLeftX() * MaxSpeed.getNumber()) // Drive left with negative X (left)
                    .withRotationalRate(-drivestick.getRightX() * RotationsPerSecond.of(maxAngular.getNumber()).in(RadiansPerSecond)) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // drivestick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // drivestick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-drivestick.getLeftY(), -drivestick.getLeftX()))
        // ));

        // reset the field-centric heading on left bumper press
        drivestick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivestick.start().onTrue(intake.resetIntakeCommand());

    this.drivestick.leftTrigger(0.25) //Starts  Intaking firewood
    .onTrue(firewoodFeeder.startFirewoodIntake())
    .onFalse(firewoodFeeder.stopFirewoodIntake());
    
    this.drivestick.leftBumper() //Starts scoring the firewood
    .onTrue(firewoodFeeder.startFirewoodOuttake())
    .onFalse(firewoodFeeder.stopFirewoodIntake());

    this.drivestick.rightTrigger(.25) //Deploys intake and starts intaking
    .onTrue(
      Commands.sequence(
      intake.deployIntakeCommand(),
      intake.startVeggieIntakeCommand())
      )
    .onFalse(
      Commands.sequence(
        intake.stowIntakeCommand(),
        intake.stopIntakeCommand()
      )
    );

    this.drivestick.a() //Deploys intake and outtakes veggies if stuck
    .onTrue(
      Commands.sequence(
      intake.deployIntakeCommand(),
      intake.startVeggieOttakeCommand())
      )
    .onFalse(
      Commands.sequence(
        intake.stowIntakeCommand(),
        intake.stopIntakeCommand()
      )
    );

    this.drivestick.rightBumper() //Starts shooting veggies, waits until shooter up to speed then indexes
    .onTrue(shooter.startShooterCommand())
    .onFalse(shooter.stopShooterCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
