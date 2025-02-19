package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class BallScrewSubsystem extends SubsystemBase {

  public double m_positionInches;
  public static final double kGearRatio = 5 / 1;
  public static final double kPositionFactor = (2 * Math.PI) * 0.1 / kGearRatio;

  private final int m_canId = 50;

  private final RelativeEncoder m_driveEncoder;
  private final SparkMax m_driveMotor;
  private final SparkClosedLoopController m_driveController;

  private final double minDistance = Units.degreesToRadians(-180);
  private final double maxDistance = Units.degreesToRadians(180);

  public BallScrewSubsystem() {
    // Drive Motor setup
    m_driveMotor = new SparkMax(m_canId, MotorType.kBrushless);

    // drive encoder setup
    m_driveEncoder = m_driveMotor.getEncoder();

    m_driveController = m_driveMotor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
    .p(1.5)
    .i(0)
    .d(0)
    .outputRange(-1, 1);
    config.encoder.positionConversionFactor(kPositionFactor);

    m_driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_positionInches = m_driveEncoder.getPosition();

    ShuffleboardContent.initBallScrew(this);
  }

  @Override
  public void periodic() {
  }

  public void moveToPosition(double position) {
    setReferenceValue(position);
    setReferencePeriodic();
  }

  public void moveToFarPosition() {
    setReferenceValue(maxDistance);
    setReferencePeriodic();
  }

  public void moveToMidPosition() {
    setReferenceValue(22);
    setReferencePeriodic();
  }

  public void moveToBackPosition() {
    setReferenceValue(0);
    setReferencePeriodic();
  }

  public double getMinDistance() {
    return minDistance;
  }

  public double getMaxDistance() {
    return maxDistance;
  }

  public double getPosition() {
    return m_driveEncoder.getPosition();
  }

  public void driveArm(double throttle) {
    if (Math.abs(throttle) > 0.05) {
      m_positionInches += throttle * 0.125;
    }
    setReferencePeriodic();
  }

  public void stop() {
    m_driveMotor.set(0);
  }

  public void setReferenceValue(double distance) {
    m_positionInches = distance;
  }

  public void setReferencePeriodic() {
    m_positionInches = MathUtil.clamp(m_positionInches, minDistance, maxDistance);
    m_driveController.setReference(m_positionInches, ControlType.kPosition);
  }
}