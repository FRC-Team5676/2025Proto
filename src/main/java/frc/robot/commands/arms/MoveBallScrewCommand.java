package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.BallScrewSubsystem;

public class MoveBallScrewCommand extends Command {

    private final BallScrewSubsystem m_controlArm;
    private final CommandXboxController m_controller;

    /** Driver control */
    public MoveBallScrewCommand(BallScrewSubsystem controlArm, CommandXboxController controller) {
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
        double trigger = m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis();
        m_controlArm.driveArm(trigger);
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
