package frc.robot.commands.arms;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BallScrewSubsystem;
import frc.robot.subsystems.LinearArmSubsystem;
import frc.robot.subsystems.RotateArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class MoveToL2 extends Command {

    private final BallScrewSubsystem m_ballScrew;
    private final RotateArmSubsystem m_rotateArm;
    private final LinearArmSubsystem m_linearArm;
    private final WristSubsystem m_wrist;

    /** Driver control */
    public MoveToL2(BallScrewSubsystem ballScrew, 
                    RotateArmSubsystem rotateArm, 
                    LinearArmSubsystem linearArm, 
                    WristSubsystem wrist) {
        m_ballScrew = ballScrew;
        m_rotateArm = rotateArm;
        m_linearArm = linearArm;
        m_wrist = wrist;

        addRequirements(ballScrew, rotateArm, linearArm, wrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_ballScrew.setReferenceValue(m_ballScrew.getMinDistance());
        m_rotateArm.setReferenceValue(72);
        m_linearArm.setReferenceValue(Units.radiansToDegrees(m_linearArm.getMaxRotations()));
        m_wrist.setReferenceValue(0);
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
