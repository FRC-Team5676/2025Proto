package frc.robot.commands.tray;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TraySubsystem;

public class TrayCommands extends Command {

    public static Command dumpTray(TraySubsystem tray) {
        return Commands.sequence(
            new InstantCommand(() -> tray.moveToDownPosition()),
            Commands.waitSeconds(1),
            new InstantCommand(() -> tray.moveToUpPosition())
        );
    }

}
