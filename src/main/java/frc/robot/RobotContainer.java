// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.MoveClimberCommand;
import frc.robot.commands.arms.ArmMoveCommands;
import frc.robot.commands.arms.MoveBallScrewCommand;
import frc.robot.commands.arms.MoveLinearArmCommand;
import frc.robot.commands.arms.MoveRotateArmCommand;
import frc.robot.commands.arms.WristCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BallScrewSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LinearArmSubsystem;
import frc.robot.subsystems.RotateAlgaeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.RotateArmSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    private final RotateArmSubsystem rotateArm = new RotateArmSubsystem();
    private final BallScrewSubsystem ballScrew = new BallScrewSubsystem();
    private final LinearArmSubsystem linearArm = new LinearArmSubsystem();
    private final ClimberSubsystem climber = new ClimberSubsystem();
    private final RotateAlgaeSubsystem rotateAlgae = new RotateAlgaeSubsystem();
    private final WristSubsystem wrist = new WristSubsystem();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.35) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    //private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick driver = new CommandJoystick(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final SwerveSubsystem drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {

        var armCommands = new ArmMoveCommands(ballScrew, rotateArm, linearArm, wrist);

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
        //driver.button(12).whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getY(), -driver.getX()))));

        // reset the field-centric heading
        driver.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Variable Position Commands
        rotateArm.setDefaultCommand(new MoveRotateArmCommand(rotateArm, operator));
        ballScrew.setDefaultCommand(new MoveBallScrewCommand(ballScrew, operator));
        linearArm.setDefaultCommand(new MoveLinearArmCommand(linearArm, operator));
        climber.setDefaultCommand(new MoveClimberCommand(climber, driver));
        wrist.setDefaultCommand(new WristCommand(wrist, operator));

        // Linear Arm Presets
        operator.button(XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(linearArm::moveToRetractedPosition));
        operator.button(XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(linearArm::moveToExtendedPosition));

        // Move Arms
        operator.button(XboxController.Button.kX.value).onTrue(armCommands.MoveAllHome());
        operator.button(XboxController.Button.kY.value).onTrue(armCommands.MoveToL2());

        // Algea
        operator.povDown().onTrue(new InstantCommand(rotateAlgae::intakeAlgea));
        operator.povDown().onFalse(new InstantCommand(rotateAlgae::stop));
        driver.button(2).onTrue(new InstantCommand(rotateAlgae::shootAlgea));
        driver.button(2).onFalse(new InstantCommand(rotateAlgae::stop));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return new PathPlannerAuto("Auto-1");
        // return autoChooser.getSelected();
    }
}
