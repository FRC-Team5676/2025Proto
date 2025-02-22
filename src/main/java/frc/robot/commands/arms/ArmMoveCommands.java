package frc.robot.commands.arms;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BallScrewSubsystem;
import frc.robot.subsystems.LinearArmSubsystem;
import frc.robot.subsystems.RotateArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmMoveCommands extends Command {

    private final BallScrewSubsystem m_ballScrew;
    private final RotateArmSubsystem m_rotateArm;
    private final LinearArmSubsystem m_linearArm;
    private final WristSubsystem m_wrist;

    public ArmMoveCommands(BallScrewSubsystem ballScrew,
            RotateArmSubsystem rotateArm,
            LinearArmSubsystem linearArm,
            WristSubsystem wrist) {

        m_ballScrew = ballScrew;
        m_rotateArm = rotateArm;
        m_linearArm = linearArm;
        m_wrist = wrist;

        addRequirements(ballScrew, rotateArm, linearArm, wrist);
    }

    public Command MoveAllHome() {
        return Commands.sequence(
                new InstantCommand(() -> m_ballScrew.moveToDownPosition()),
                new InstantCommand(() -> m_wrist.setReferenceValue(0)),
                new InstantCommand(() -> m_linearArm.setReferenceValue(Units.radiansToDegrees(m_linearArm.getMaxRotations()))),
                new InstantCommand(() -> m_rotateArm.setReferenceValue(0)));
    }

    public Command MoveToL2() {
        return Commands.sequence(
                new InstantCommand(() -> m_ballScrew.moveToUpPosition()),
                new InstantCommand(() -> m_rotateArm.setReferenceValue(72)),
                new InstantCommand(() -> m_linearArm.setReferenceValue(Units.radiansToDegrees(m_linearArm.getMaxRotations()))),
                new InstantCommand(() -> m_wrist.setReferenceValue(0))
            );
    }

    /*
     * public Command MoveToL2() {
     * return run(() -> m_ballScrew.setReferenceValue(m_ballScrew.getMinDistance())
     * .until(m_ballScrew::atSetpoint)
     * .thenrun(() -> runOnce(PositionL2)));
     * }
     * 
     * private Command PositionL2() {
     * return Commands.sequence(
     * new InstantCommand(() -> m_rotateArm.setReferenceValue(72)),
     * new InstantCommand(() ->
     * m_linearArm.setReferenceValue(Units.radiansToDegrees(m_linearArm.
     * getMaxRotations()))),
     * new InstantCommand(() -> m_wrist.setReferenceValue(0))
     * );
     * 
     * }
     */
}
