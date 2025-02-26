package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class BallScrewSubsystem extends SubsystemBase {

  public double m_positionUnits;

  private final int m_canId = 50;

  private final WPI_TalonSRX m_driveMotor;

  private final double ballScrewUpPosition = -540000; //-540000;
  private final double ballScrewDownPosition = 3938; //3938;

  public BallScrewSubsystem() {
    // Drive Motor setup
    m_driveMotor = new WPI_TalonSRX(m_canId);
    m_driveMotor.configFactoryDefault();
    m_driveMotor.setNeutralMode(NeutralMode.Brake);

    // PID Setup
    m_driveMotor.config_kP(0, 0.04);
    m_driveMotor.config_kI(0, 0);
    m_driveMotor.config_kD(0, 0);
    m_driveMotor.config_kF(0, 0);

    // Encoder setup
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    m_driveMotor.setInverted(true);
    m_driveMotor.setSensorPhase(true);
    m_driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 30);
    m_driveMotor.configPeakOutputForward(1);
    m_driveMotor.configPeakOutputReverse(-1);
    
    m_positionUnits = m_driveMotor.getSelectedSensorPosition();

    ShuffleboardContent.initBallScrew(this);
  }

  @Override
  public void periodic() {
  }

  public void moveToUpPosition() {
    m_positionUnits = ballScrewUpPosition;
    setReferencePeriodic();
  }

  public void moveToDownPosition() {
    m_positionUnits = ballScrewDownPosition;
    setReferencePeriodic();
  }

  public boolean atDownPosition() {
    double min = ballScrewDownPosition - ballScrewDownPosition * 0.1;
    double max = ballScrewDownPosition + ballScrewDownPosition * 0.1;
    if (getActualUnits() >= min && getActualUnits() <= max) {
      return true;
    } else {
      return false;
    }
  }

  public double getUpPosition() {
    return ballScrewUpPosition;
  }

  public double getDownPosition() {
    return ballScrewDownPosition;
  }

  public boolean belowMidPosition() {
    double midPosition = (ballScrewUpPosition + ballScrewDownPosition) / 2;
    if (getActualUnits() > midPosition) {
      return true;
    }
    return false;
  }

  public double getActualUnits() {
    return m_driveMotor.getSelectedSensorPosition();
  }

  public double getTargetUnits() {
    return m_positionUnits;
  }

  public void driveArm(double units) {
    if (Math.abs(units) > 0.05) {
      m_positionUnits += units * 10000;
    }
    setReferencePeriodic();
  }

  private void setReferencePeriodic() {
    m_positionUnits = MathUtil.clamp(m_positionUnits, ballScrewUpPosition, ballScrewDownPosition);
    m_driveMotor.set(ControlMode.Position, m_positionUnits);
  }
}