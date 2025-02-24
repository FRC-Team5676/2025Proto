package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends Command {

    private final ArmSubsystem m_Arm;
    private final CommandXboxController m_controller;

    /** Driver control */
    public MoveArmCommand(ArmSubsystem arm, CommandXboxController controller) {
        m_Arm = arm;
        m_controller = controller;

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_Arm.driveRotateArm(m_controller.getRightY());

        double trigger = m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis();
        m_Arm.driveLinearArm(trigger);
        
        m_Arm.driveWrist(m_controller.getLeftY());
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
