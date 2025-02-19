package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class BallScrewSubsystem extends SubsystemBase {

  public double m_positionInches;
  public static final double kGearRatio = 5 / 1;
  public static final double kPositionFactor = 0.1 / 4096 / kGearRatio;

  private final int m_canId = 50;

  private final WPI_TalonSRX m_driveMotor;

  private final double minDistance = -2000;
  private final double maxDistance = 2000;

  public BallScrewSubsystem() {
    // Drive Motor setup
    m_driveMotor = new WPI_TalonSRX(m_canId);
    m_driveMotor.configFactoryDefault();
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    m_driveMotor.setSensorPhase(true);

    m_driveMotor.config_kP(0, 0.15);
    m_driveMotor.config_kI(0, 0);
    m_driveMotor.config_kD(0, 1);
    m_driveMotor.config_kF(0, 0);

		/* Config the peak and nominal outputs, 12V means full */
		m_driveMotor.configNominalOutputForward(0);
		m_driveMotor.configNominalOutputReverse(0);
		m_driveMotor.configPeakOutputForward(1);
		m_driveMotor.configPeakOutputReverse(-1);
    
    m_positionInches = m_driveMotor.getSelectedSensorPosition() * kPositionFactor;

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
    return m_driveMotor.getSelectedSensorPosition() * kPositionFactor;
  }

  public void driveArm(double throttle) {
    if (Math.abs(throttle) > 0.05) {
      m_positionInches += throttle * 10;
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
    //m_positionInches = MathUtil.clamp(m_positionInches, minDistance, maxDistance);
    m_driveMotor.set(ControlMode.Position, m_positionInches / kPositionFactor);
  }
}