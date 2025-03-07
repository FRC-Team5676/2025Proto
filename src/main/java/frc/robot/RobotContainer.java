// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AutonManager;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.35) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    //private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final AutonManager autonManager = new AutonManager();
    private final CommandJoystick driver = new CommandJoystick(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

     public RobotContainer() {
        addAutonomousChoices();
        autonManager.displayChoices();
    
        configureBindings();
    }
        
    public Command getAutonomousCommand() {
        return autonManager.getSelected();
    }

    private void addAutonomousChoices() {
        autonManager.addDefaultOption("Middle To Side", AutoCommands.moveMiddleToSide());
        autonManager.addDefaultOption("Middle", AutoCommands.moveMiddle());
        autonManager.addOption("Left", AutoCommands.moveLeft());
        autonManager.addOption("Right", AutoCommands.moveRight());
        
      }
    
    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getY() * MaxSpeed) // Drive forward with
                                                                                                 // negative Y (forward)
                        .withVelocityY(-driver.getX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getTwist() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
                ));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Robot centric driving
        driver.button(12).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getY(), -driver.getX()))));

        // Reset the field-centric heading
        driver.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    }
}
