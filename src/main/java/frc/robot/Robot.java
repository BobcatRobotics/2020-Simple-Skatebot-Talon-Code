/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Solenoid;

//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.util.Color;
//import com.revrobotics.ColorSensorV3;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private WPI_TalonSRX leftTalon = new WPI_TalonSRX(7);
  private WPI_TalonSRX rightTalon = new WPI_TalonSRX(8);
  private WPI_TalonFX testFalcon = new WPI_TalonFX(0);
  //private WPI_TalonSRX pingPongTalon = new WPI_TalonSRX(8);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftTalon, rightTalon);
  private final Joystick m_stick = new Joystick(0);
  //public static Solenoid solenoid1 = new Solenoid(7);
  private double leftStick = 0.0;
  private double rightStick = 0.0;
  private double leftVelocity = 0.0;
  private double leftDistance = 0.0;
  private double rightDistance = 0.0;
  private double rightVelocity = 0.0;
  private double testFalconDistance = 0.0;
  private double testFalconVelocity = 0.0;

  private AHRS ahrs = new AHRS(SPI.Port.kMXP);



  //private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Flip the phase of the encoder for use with SRX motion magic, etc.
    // and set current position to 0.0;
      leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,0);
      leftTalon.setSelectedSensorPosition(0,0,0);
      leftTalon.setSensorPhase(true);

      rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,0);
      rightTalon.setSelectedSensorPosition(0,0,0);
      rightTalon.setSensorPhase(false);

      testFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
      testFalcon.setSelectedSensorPosition(0,0,0);
      testFalcon.setSensorPhase(false);
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    leftStick = m_stick.getRawAxis(Joystick.AxisType.kY.value);
    rightStick = m_stick.getRawAxis(Joystick.AxisType.kTwist.value);
    SmartDashboard.putNumber("left stick:", leftStick);
    SmartDashboard.putNumber("right stick:", rightStick);

    // Set the testFalcon to percent output based on right stick
    // Read right bumper, if pressed pass right stick to talonFX.
    boolean test_falcon_pressed = m_stick.getRawButton(6);
    if (test_falcon_pressed) {
      testFalcon.set(ControlMode.PercentOutput, rightStick);
      m_robotDrive.tankDrive(0.0, 0.0);
    } else {
      testFalcon.set(ControlMode.PercentOutput, 0.0);
      m_robotDrive.tankDrive(leftStick, rightStick);
    }

    // Try to drive the solenoids
    //solenoid1.set(m_stick.getRawButton(2));

    // Read Talon Sensors and display values
    readTalonsAndShowValues();

    // Read left bumper, if pressed reset NavX yaw value.
    boolean zero_yaw_pressed = m_stick.getRawButton(5);
    if (zero_yaw_pressed) {
      ahrs.zeroYaw();
    }
    // Read NavX and display values
    readNavxAndShowValues();

    // Drive ping pong ball shooter when button 8 is pressed
    //if (m_stick.getRawButton(8)) {
    //  pingPongTalon.set(-1.0);
    //}
    //else {
    //  pingPongTalon.set(0.0);
    //}
  }

   /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void disabledPeriodic() {
    leftStick = m_stick.getRawAxis(Joystick.AxisType.kY.value);
    rightStick = m_stick.getRawAxis(Joystick.AxisType.kTwist.value);
    SmartDashboard.putNumber("left stick:", leftStick);
    SmartDashboard.putNumber("right stick:", rightStick);

    // Read Talon Sensors and display values
    readTalonsAndShowValues();

    // Read left bumper, if pressed reset NavX yaw value.
    boolean zero_yaw_pressed = m_stick.getRawButton(5);
    if (zero_yaw_pressed) {
      ahrs.zeroYaw();
    }
    // Read NavX and display values
    readNavxAndShowValues();
    
    // Color color = m_colorSensor.getColor();
    // double IR = color.getIR();

    // SmartDashboard.putNumber("Red", color.red);
    // SmartDashboard.putNumber("Blue", color.blue);
    // SmartDashboard.putNumber("Green", color.green);
    // SmartDashboard.putNumber("IR", IR);
    // SmartDashboard.putNumber("Dist",m_colorSensor.getProximity());
}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void readTalonsAndShowValues() {
    leftDistance = leftTalon.getSelectedSensorPosition(0);
    rightDistance = rightTalon.getSelectedSensorPosition(0);
    leftVelocity = leftTalon.getSelectedSensorVelocity(0);
    rightVelocity = rightTalon.getSelectedSensorVelocity(0);
    testFalconDistance = testFalcon.getSelectedSensorPosition(0);
    testFalconVelocity = testFalcon.getSelectedSensorVelocity(0);
    SmartDashboard.putNumber("left distance:", leftDistance);
    SmartDashboard.putNumber("left velocity:", leftVelocity);
    SmartDashboard.putNumber("right distance:", rightDistance);
    SmartDashboard.putNumber("right velocity:", rightVelocity);
    SmartDashboard.putNumber("testFalcon distance:", testFalconDistance);
    SmartDashboard.putNumber("testFalcon velocity:", testFalconVelocity);
  }

  public void readNavxAndShowValues() {
    /* Display 6-axis Processed Angle Data */
    SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

    /* Display tilt-corrected, Magnetometer-based heading (requires */
    /* magnetometer calibration to be useful) */

    SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

    SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
    SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

    SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
    SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
    SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
    SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

    /* Display estimates of velocity/displacement. Note that these values are */
    /* not expected to be accurate enough for estimating robot position on a */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially */
    /* double (displacement) integration. */

    SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
    SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
    SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
    SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());

    /* Display Raw Gyro/Accelerometer/Magnetometer Values */
    /* NOTE: These values are not normally necessary, but are made available */
    /* for advanced users. Before using this data, please consider whether */
    /* the processed data (see above) will suit your needs. */

    SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
    SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
    SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
    SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
    SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
    SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
    SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
    SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
    SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
    SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());
    SmartDashboard.putNumber("IMU_Timestamp", ahrs.getLastSensorTimestamp());

    /* Omnimount Yaw Axis Information */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
    AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

    /* Sensor Board Information */
    SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());

    /* Quaternion Data */
    /* Quaternions are fascinating, and are the most compact representation of */
    /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
    /* from the Quaternions. If interested in motion processing, knowledge of */
    /* Quaternions is highly recommended. */
    SmartDashboard.putNumber("QuaternionW", ahrs.getQuaternionW());
    SmartDashboard.putNumber("QuaternionX", ahrs.getQuaternionX());
    SmartDashboard.putNumber("QuaternionY", ahrs.getQuaternionY());
    SmartDashboard.putNumber("QuaternionZ", ahrs.getQuaternionZ());    
  }
}
