package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.controllers.xbox;
import frc.robot.subsystems.LowerArmSubsystem;

public class MoveLowerArmCommand extends CommandBase {

    private final LowerArmSubsystem m_controlArm;
    private final CommandXboxController m_controller;

    /** Driver control */
    public MoveLowerArmCommand(LowerArmSubsystem controlArm, CommandXboxController controller) {
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
        m_controlArm.driveArm(m_controller.getRightY());
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
