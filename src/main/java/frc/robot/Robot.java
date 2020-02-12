/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private final Joystick m_stick = new Joystick(0);  // Code setup to use gamepad with botton on back
                                                     // set in the D position (not X position)
  private double leftStickRaw = 0.0; // Raw value from left stick
  private double rightStickRaw = 0.0; // Raw value from right stick
  private double leftStickBias = -0.007813;  // Left stick normal bias from zero
  private double rightStickBias = -0.007813; // Right stick normal bias from zero
  private double flipStick = -1.0;   // Use -1.0 to flip raw value from stick
  private double scaleStick = 1.0;   // Use 1.0 for max speed, lower values to impose slower driving
  private double leftStick = 0.0;    // Flipped and scaled value of left stick
  private double rightStick = 0.0;   // Flipped and scaled value of right stick

  private WPI_TalonSRX leftMotor = new WPI_TalonSRX(7);   // Instantiate left motor controller
  private WPI_TalonSRX rightMotor = new WPI_TalonSRX(8);  // Instantiate right motor controller
  private WPI_TalonSRX turretMotor = new WPI_TalonSRX(10);  // Instantiate turret motor controller
  private double encoderEPR = 4096.0;                     // Talon mag encoder edges per rev of encoder
  
  private double maxMotorVoltage = 11.0;          // Voltage to be used as max for motor controllers
  private boolean enableMaxMotorVoltage = true;   // Flag to enable using the max voltage setting

  private double leftDiameter = 0.3229;  // left wheel diameter
  private double leftRatio = 3.0;        // left gear ratio
  private double leftDistanceRaw = 0.0;  // left wheel distance in raw encoder units
  private double leftVelocityRaw = 0.0;  // left wheel velocity in raw encoder units per 100msec
  private double leftDistance = 0.0;     // left wheel distance in feet
  private double leftVelocity = 0.0;     // left wheel velocity in feet/sec
  private double leftRPM = 0.0;          // left wheel RPM

  private double rightDiameter = 0.3229; // right wheel diameter
  private double rightRatio = 5.0;       // right gear ratio
  private double rightDistanceRaw = 0.0; // right wheel distance in raw encoder units
  private double rightVelocityRaw = 0.0; // right wheel velocity in raw encoder units per 100msec
  private double rightDistance = 0.0;    // right wheel distance in feet
  private double rightVelocity = 0.0;    // right wheel velocity in feet/sec
  private double rightRPM = 0.0;         // right wheel RPM

  private double turretPosition = 0.0;
  private double turretVelocity = 0.0;

  private double PI = 3.141592;

  private double tv;
  private double tx;
  private double ty;
  private double ta;

  private boolean toggleLED;

  private double thresh=0.5;
  private double turretKp = 0.03; // Initial guess, 0.03 ~ 1.0PctOutput/30degrees
  private double minTurretCmd = 0.06;  // Min cmd to make turret move at all
  private double turretCmd;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    // Initialize Falcons to Factory Default
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();
    turretMotor.configFactoryDefault();

    // Set sense of master motor output
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
    turretMotor.setInverted(false);

    // Set motors controllers to coast mode
    leftMotor.setNeutralMode(NeutralMode.Coast);
    rightMotor.setNeutralMode(NeutralMode.Coast);
    turretMotor.setNeutralMode(NeutralMode.Coast);

    // Set voltage compensation to keep things consistent as battery discharges
    leftMotor.configVoltageCompSaturation(maxMotorVoltage);
    leftMotor.enableVoltageCompensation(enableMaxMotorVoltage);
    rightMotor.configVoltageCompSaturation(maxMotorVoltage);
    rightMotor.enableVoltageCompensation(enableMaxMotorVoltage);
    turretMotor.configVoltageCompSaturation(maxMotorVoltage);
    turretMotor.enableVoltageCompensation(enableMaxMotorVoltage);

    // Setup sensors, set current positoin to 0.0 and choose phase if needed.
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    leftMotor.setSelectedSensorPosition(0,0,0);
    leftMotor.setSensorPhase(false);
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
    rightMotor.setSelectedSensorPosition(0,0,0);
    rightMotor.setSensorPhase(false);
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,0);
    turretMotor.setSelectedSensorPosition(0,0,0);
    turretMotor.setSensorPhase(false);

    // Config the peak and nominal outputs
    leftMotor.configNominalOutputForward(0,0);
    leftMotor.configNominalOutputReverse(0,0);
    leftMotor.configPeakOutputForward(1,0);
    leftMotor.configPeakOutputReverse(-1,0);
    rightMotor.configNominalOutputForward(0,0);
    rightMotor.configNominalOutputReverse(0,0);
    rightMotor.configPeakOutputForward(1,0);
    rightMotor.configPeakOutputReverse(-1,0);
    turretMotor.configNominalOutputForward(0,0);
    turretMotor.configNominalOutputReverse(0,0);
    turretMotor.configPeakOutputForward(1,0);
    turretMotor.configPeakOutputReverse(-1,0);

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
    leftStickRaw = m_stick.getRawAxis(Joystick.AxisType.kY.value);
    rightStickRaw = m_stick.getRawAxis(Joystick.AxisType.kTwist.value);
    leftStick = flipStick*scaleStick*(leftStickRaw-leftStickBias);
    rightStick = flipStick*scaleStick*(rightStickRaw-rightStickBias);

    // Drive the motor controllers with the left and right stick commands
    if (m_stick.getRawButton(2)) {
      turretMotor.set(ControlMode.PercentOutput, leftStick);
      leftMotor.set(ControlMode.PercentOutput, 0.0);
      rightMotor.set(ControlMode.PercentOutput, 0.0);
    } else {
      turretMotor.set(ControlMode.PercentOutput, 0.0);
      leftMotor.set(ControlMode.PercentOutput, leftStick);
      rightMotor.set(ControlMode.PercentOutput, rightStick);
    }

    // If the right bumper is pressed reset the encoder values
    if (m_stick.getRawButton(6)) {
      leftMotor.setSelectedSensorPosition(0,0,0);
      rightMotor.setSelectedSensorPosition(0,0,0);
    }

    // Track target
    if (m_stick.getRawButton(1)){
      // If we see the target get the angular offset tx (which we want to be zero)
      tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
      if (tv > 0.5) {
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      } else {
        tx = 0.0;
      }
      // if tx is between -0.5 and +0.5 then don't move, otherwise try and get there
      if ((tx < -thresh) || (tx > thresh)) {
        // If tx is positive the target is to the right in the limelight frame, and need to
        // turn to the right to bring target toward center,
        // on the skatebot this is a negative command to the turret motor control
        if (tx > thresh) {
          turretCmd = -1.0*turretKp*(tx-thresh) - minTurretCmd;
        }
        if (tx < -thresh) {
          turretCmd = -1.0*turretKp*(tx+thresh) + minTurretCmd;
        }
      } else {
        turretCmd = 0.0;  // Inside threshold, command is zero
      }
      turretMotor.set(ControlMode.PercentOutput, turretCmd);
    }


    // Read Talon Sensors and display values
    readTalonsAndShowValues();
  }

   /**
   * This function is called periodically during disabled mode.
   */
  @Override
  public void disabledPeriodic() {
    leftStickRaw = m_stick.getRawAxis(Joystick.AxisType.kY.value);
    rightStickRaw = m_stick.getRawAxis(Joystick.AxisType.kTwist.value);
    leftStick = flipStick*scaleStick*(leftStickRaw-leftStickBias);
    rightStick = flipStick*scaleStick*(rightStickRaw-rightStickBias);

    // If the right bumper is pressed reset the encoder values
    if (m_stick.getRawButton(6)) {
      leftMotor.setSelectedSensorPosition(0,0,0);
      rightMotor.setSelectedSensorPosition(0,0,0);
    }
    
    if (m_stick.getRawButton(5)) toggleLED = ! toggleLED;
    if (toggleLED) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    // Read Talon Sensors and display values
    readTalonsAndShowValues();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  // Simple method to put values on the SmartDashboard
  public void readTalonsAndShowValues() {
    //SmartDashboard.putNumber("left stick raw:", leftStickRaw);
    //SmartDashboard.putNumber("right stick raw:", rightStickRaw);
    SmartDashboard.putNumber("left stick:", leftStick);
    SmartDashboard.putNumber("right stick:", rightStick);
    leftDistanceRaw = leftMotor.getSelectedSensorPosition();
    leftDistance = PI * leftDiameter * leftDistanceRaw / (leftRatio*encoderEPR);
    rightDistanceRaw = rightMotor.getSelectedSensorPosition();
    rightDistance = PI * rightDiameter * rightDistanceRaw / (rightRatio*encoderEPR);
    leftVelocityRaw = leftMotor.getSelectedSensorVelocity();
    leftRPM = leftVelocityRaw * 600.0/(leftRatio*encoderEPR);
    leftVelocity = PI*leftDiameter*leftRPM/60.0;
    rightVelocityRaw = rightMotor.getSelectedSensorVelocity();
    rightRPM = rightVelocityRaw * 600.0/(rightRatio*encoderEPR);
    rightVelocity = PI*rightDiameter*rightRPM/60.0;
    SmartDashboard.putNumber("left distance (ft):", leftDistance);
    SmartDashboard.putNumber("left velocity (ft/sec):", leftVelocity);
    SmartDashboard.putNumber("left RPM:", leftRPM);
    SmartDashboard.putNumber("right distance (ft):", rightDistance);
    SmartDashboard.putNumber("right velocity (ft/sec):", rightVelocity);
    SmartDashboard.putNumber("right RPM:", rightRPM);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    turretPosition = turretMotor.getSelectedSensorPosition();
    turretVelocity = turretMotor.getSelectedSensorVelocity();
    SmartDashboard.putNumber("turret_pos", turretPosition);
    SmartDashboard.putNumber("turret_vel", turretVelocity);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("turretCmd", turretCmd);
  }
}