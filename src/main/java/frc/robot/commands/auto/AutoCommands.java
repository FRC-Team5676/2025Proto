package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoCommands extends Command {

    public static Command moveLeft() {
        return Commands.sequence(
            new PathPlannerAuto("Left")
        );
    }

    public static Command moveRight() {
        return Commands.sequence(
            new PathPlannerAuto("Right")
        );
    }

    public static Command moveMiddleToSide() {
        return Commands.sequence(
            new PathPlannerAuto("Middle To Side")
        );
    }

    public static Command moveOut() {
        return Commands.sequence(
            new PathPlannerAuto("Move Out")
        );
    }
}
