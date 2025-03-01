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

}
