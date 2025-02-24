package frc.robot.commands.arms;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BallScrewSubsystem;

public class ArmMoveCommands extends Command {

    private final BallScrewSubsystem m_BallScrew;
    private final ArmSubsystem m_Arm;

    public ArmMoveCommands(BallScrewSubsystem ballScrew,
            ArmSubsystem arm) {

        m_BallScrew = ballScrew;
        m_Arm = arm;

        addRequirements(ballScrew, arm);
    }

    public Command MoveAllHome() {
        return Commands.sequence(
                new InstantCommand(() -> m_BallScrew.moveToDownPosition()),
                new InstantCommand(() -> m_Arm.moveWrist(0)),
                new InstantCommand(() -> m_Arm.moveLinearArm(m_Arm.getMaxLinearArmDegrees())),
                new InstantCommand(() -> m_Arm.moveRotateArm(0)));
    }

    public Command MoveToL2() {
        return Commands.sequence(
                new InstantCommand(() -> m_BallScrew.moveToUpPosition()),
                new InstantCommand(() -> m_Arm.moveRotateArm(72)),
                new InstantCommand(() -> m_Arm.moveLinearArm(m_Arm.getMaxLinearArmDegrees())),
                new InstantCommand(() -> m_Arm.moveWrist(0))
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
