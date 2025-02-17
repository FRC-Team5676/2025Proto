package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class RotateArmSubsystem extends SubsystemBase {

  public double rotations;

  private final int m_lowerArmCanId = 51;

  private final RelativeEncoder m_driveEncoder;
  private final SparkMax m_driveMotor;
  private final SparkClosedLoopController m_driveController;

  private final double minRotations = -20;
  private final double maxRotations = 90;

  public RotateArmSubsystem() {
    // Drive Motor setup
    m_driveMotor = new SparkMax(m_lowerArmCanId, MotorType.kBrushless);

    // drive encoder setup
    m_driveEncoder = m_driveMotor.getEncoder();

    m_driveController = m_driveMotor.getClosedLoopController();
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
    .p(0.01)
    .i(0)
    .d(0)
    .outputRange(-1, 1);

    ShuffleboardContent.initLowerArm(this);
  }

  @Override
  public void periodic() {
  }

  public void moveToPosition(double position) {
    setReferenceValue(position);
    setReferencePeriodic();
  }

  public void moveToFarPosition() {
    setReferenceValue(maxRotations);
    setReferencePeriodic();
  }

  public void moveToMidPosition() {
    setReferenceValue(22);
    setReferencePeriodic();
  }

  public void moveToBackPosition() {
    setReferenceValue(-20);
    setReferencePeriodic();
  }

  public double getMinRotations() {
    return minRotations;
  }

  public double getMaxRotations() {
    return maxRotations;
  }

  public double getPosition() {
    return m_driveEncoder.getPosition();
  }

  public void driveArm(double throttle) {
    throttle = -throttle;
    rotations += throttle;
    setReferencePeriodic();
  }

  public void stop() {
    m_driveMotor.set(0);
  }

  public void setReferenceValue(double rotation) {
    rotations = rotation;
  }

  public void setReferencePeriodic() {
    rotations = MathUtil.clamp(rotations, minRotations, maxRotations);
    m_driveController.setReference(rotations, SparkMax.ControlType.kPosition);
  }
}