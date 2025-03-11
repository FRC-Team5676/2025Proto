// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AutonManager;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final AutonManager autonManager = new AutonManager();
    private final CommandJoystick driver = new CommandJoystick(0);

    public final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();

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
        autonManager.addOption("Left", AutoCommands.moveLeft());
        autonManager.addOption("Right", AutoCommands.moveRight());
        
      }
    
    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(getY())
                    .withVelocityY(getX())
                    .withRotationalRate(getTwist())
                ));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Robot centric driving
        driver.button(12).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getY(), -driver.getX()))));

        // Reset the field-centric heading
        driver.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    private double getX() {
        double xDeadband = 0.1;
        double x = -driver.getX(); // Drive left with negative X (left)
        
        if (Math.abs(x) < xDeadband) {
            return 0;
        } else {
            return (1 / (1 - xDeadband)) * (x + (-Math.signum(x) * xDeadband)) * MaxSpeed;
        }
    }

    private double getY() {
        double xDeadband = 0.1;
        double y = -driver.getY(); // Drive forward with negative Y (forward)
        
        if (Math.abs(y) < xDeadband) {
            return 0;
        } else {
            return (1 / (1 - xDeadband)) * (y + (-Math.signum(y) * xDeadband)) * MaxSpeed;
        }
    }

    private double getTwist() {
        double deadband;
        double twist = -driver.getTwist(); // Drive counterclockwise with negative twist (CCW)
        
        if (Math.signum(twist) < 0) {
            // CCW
            deadband = 0.7; // larger on this side because of joystick sensitivity on CCW rotation
        } else if (Math.signum(twist) > 0) {
            // CW
            deadband = 0.1;
        } else {
            return 0;
        }

        if (Math.abs(twist) < deadband) {
            return 0;
        } else {
            return (1 / (1 - deadband)) * (twist + (-Math.signum(twist) * deadband)) * MaxAngularRate;
        }
    }
}
