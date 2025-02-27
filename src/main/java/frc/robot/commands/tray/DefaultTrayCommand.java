package frc.robot.commands.tray;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.TraySubsystem;

public class DefaultTrayCommand extends Command {

    private final TraySubsystem m_controlArm;
    private final CommandXboxController m_controller;

    /** Driver control */
    public DefaultTrayCommand(TraySubsystem controlArm, CommandXboxController controller) {
        m_controlArm = controlArm;
        m_controller = controller;

        addRequirements(controlArm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double trigger = 0.0;
        if (m_controller.povUp().getAsBoolean()) {
          trigger = -1;
        } else if (m_controller.povRight().getAsBoolean()) {
          trigger = 1;
        } else {
          trigger = 0;
        }
        
        m_controlArm.driveTray(trigger);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
