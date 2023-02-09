// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.security.auth.login.FailedLoginException;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class SwerveModule {
  private static final double kWheelRadius = 1.9685;
  private static final int kEncoderResolution = 42;
  private static final float kGearRatio = 21.4285714286f;

  private static final float kDriveGearRatio = 6.75f;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;
    // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0, 0, 0);

  private final String m_name;

  private final double m_correction;

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          0,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 0);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      
      int turningEncoderChannel,
      String moduleName,
      double zeroPoint
      ) {
    
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveMotor.setSmartCurrentLimit(40);
    m_turningMotor.setSmartCurrentLimit(40);
   
    m_turningEncoder = new CANCoder(turningEncoderChannel);
    m_name = moduleName;
    m_correction = zeroPoint;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setVelocityConversionFactor(1/6.75);
    m_driveEncoder.setPositionConversionFactor( 1/6.75);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    // m_driveEncoder.setVelocityConversionFactor(2 * Math.PI / kEncoderResolution);
    
    //m_driveEncoder.configFeedbackCoefficient(0.087890625 * 360 * (1/kDriveGearRatio) * (Units.inchesToMeters(2 * Math.PI * kWheelRadius)), "meters", SensorTimeBase.PerSecond);
    m_turningEncoder.configFeedbackCoefficient(0.087890625, "deg", SensorTimeBase.PerSecond);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }


  private double fixEncoder() {
    return m_turningEncoder.getPosition() - m_correction;
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   * 
  
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(fixEncoder()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(fixEncoder()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SmartDashboard.putNumber(m_name + "'s angle", m_turningEncoder.getPosition());
    SmartDashboard.putNumber(m_name + "'s speed", m_driveEncoder.getVelocity());
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(fixEncoder()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getPosition(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    //m_driveMotor.setVoltage(driveOutput + driveFeedforward); this should be it but I am testing things rn
    m_driveMotor.setVoltage(driveOutput +driveFeedforward);
    //m_turningMotor.setVoltage(turnOutput + turnFeedforward); real code, commented out for testing purposes
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}