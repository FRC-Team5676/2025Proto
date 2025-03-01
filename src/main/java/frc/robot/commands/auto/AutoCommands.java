package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TraySubsystem;

public class AutoCommands extends Command {

    public static Command moveToReefCoralTopRightTOP(TraySubsystem tray) {
        return Commands.sequence(
            new PathPlannerAuto("Coral Top Right - TOP"),
            new InstantCommand(() -> tray.moveToDownPosition()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> tray.moveToUpPosition())
        );
    }

    public static Command moveToReefCoralMiddleRightTOP(TraySubsystem tray) {
        return Commands.sequence(
            new PathPlannerAuto("Coral Middle Right - TOP"),
            new InstantCommand(() -> tray.moveToDownPosition()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> tray.moveToUpPosition())
        );
    }

    public static Command moveToReefCoralBottomRightTOP(TraySubsystem tray) {
        return Commands.sequence(
            new PathPlannerAuto("Coral Bottom Right - TOP"),
            new InstantCommand(() -> tray.moveToDownPosition()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> tray.moveToUpPosition())
        );
    }

    public static Command moveToReefCoralTopRightBOTTOM(TraySubsystem tray) {
        return Commands.sequence(
            new PathPlannerAuto("Coral Top Right - BOTTOM"),
            new InstantCommand(() -> tray.moveToDownPosition()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> tray.moveToUpPosition())
        );
    }

    public static Command moveToReefCoralMiddleRightBOTTOM(TraySubsystem tray) {
        return Commands.sequence(
            new PathPlannerAuto("Coral Middle Right - BOTTOM"),
            new InstantCommand(() -> tray.moveToDownPosition()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> tray.moveToUpPosition())
        );
    }

    public static Command moveToReefCoralBottomRightBOTTOM(TraySubsystem tray) {
        return Commands.sequence(
            new PathPlannerAuto("Coral Bottom Right - BOTTOM"),
            new InstantCommand(() -> tray.moveToDownPosition()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> tray.moveToUpPosition())
        );
    }
}
