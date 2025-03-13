// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import frc.robot.commands.auto.AutoCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AutonManager;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 0.75 rotations per second max

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final AutonManager autonManager = new AutonManager();
    private final CommandJoystick driver = new CommandJoystick(0);

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
                drivetrain.applyRequest(() -> driveField
                        .withVelocityX(getY())
                        .withVelocityY(getX())
                        .withRotationalRate(getTwist())));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Limelight driving
        driver.button(1).whileTrue(
                drivetrain.applyRequest(() -> driveRobot
                        .withVelocityX(limelightForward())
                        .withVelocityY(limelightSideToSide())
                        .withRotationalRate(limelightRotation())));

        // Robot centric driving
        driver.button(12).whileTrue(
                drivetrain.applyRequest(() -> driveRobot
                        .withVelocityX(getY())
                        .withVelocityY(getX())
                        .withRotationalRate(getTwist())));

        // Reset the field-centric heading
        driver.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    private double getX() {
        double deadband = 0.05;
        double value = -driver.getX(); // Drive left with negative X (left)

        value = MathUtil.applyDeadband(value, deadband);
        value = Math.signum(value) * Math.pow(value, 2);
        return value * MaxSpeed;
    }

    private double getY() {
        double deadband = 0.05;
        double value = -driver.getY(); // Drive forward with negative Y (forward)

        value = MathUtil.applyDeadband(value, deadband);
        value = Math.signum(value) * Math.pow(value, 2);
        return value * MaxSpeed;
    }

    private double getTwist() {
        double deadband;
        double value = -driver.getTwist(); // Drive counterclockwise with negative twist (CCW)

        // x = Axis 4: 1 bottom to -1 top
        // translate to 1 bottom to 2 top using formula
        // y = -0.5x + 1.5
        double multiplier = -0.5 * driver.getRawAxis(4) + 1.5;

        if (Math.signum(value) <= 0) {
            // CCW
            deadband = 0.5; // larger on this side because of joystick sensitivity on CCW rotation
        } else if (Math.signum(value) > 0) {
            // CW
            deadband = 0.0;
        } else {
            return 0;
        }

        value = MathUtil.applyDeadband(value, deadband);
        value = Math.signum(value) * Math.pow(value, 2);
        return value * MaxAngularRate * multiplier;
    }

    private double limelightRotation() {
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our
        // proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        // rightmost edge of
        // your limelight 3 feed, tx should return roughly 31 degrees.

        double kP = .035;
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // Drive counterclockwise with negative twist (CCW)
        return -targetingAngularVelocity * MaxAngularRate;
    }

    private double limelightForward() {
        double kP = 0.1;
        double currentDistance = DistanceToTargetForward();
        double desiredDistance = Units.inchesToMeters(12);

        double distanceError = desiredDistance - currentDistance;
        double forwardSpeed = distanceError * kP;

        // Drive forward with negative Y (forward)
        return -forwardSpeed * MaxSpeed;
    }

    private double DistanceToTargetForward() {
        // d = (h2-h1) / tan(a1+a2)
        // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance

        double h1 = Units.inchesToMeters(24); // height of the camera
        double h2 = Units.inchesToMeters(12); // height of the target
        double a1 = Units.degreesToRadians(0); // y angle of the camera
        double a2 = Units.degreesToRadians(LimelightHelpers.getTY("limelight")); // y angle of the target

        return (h2 - h1) / Math.tan(a1 + a2);
    }

    private double limelightSideToSide() {
        double kP = 0.1;
        double currentDistance = DistanceToTargetSideToSide();
        double desiredDistance = Units.inchesToMeters(0);

        double distanceError = desiredDistance - currentDistance;
        double sideSpeed = distanceError * kP;

        // Drive left with negative X (left)
        return -sideSpeed * MaxSpeed;
    }

    private double DistanceToTargetSideToSide() {
        // h2 = h1 + d * tan(a1 + a2)
        // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance

        double h1 = Units.inchesToMeters(0); // side offset of the camera
        double d = DistanceToTargetForward(); // distance forward to the target
        double a1 = Units.degreesToRadians(0); // x angle of the camera
        double a2 = Units.degreesToRadians(LimelightHelpers.getTX("limelight")); // x angle of the target

        return h1 + d * Math.tan(a1 + a2);
    }
}
