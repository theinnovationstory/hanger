// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import java.util.spi.CurrencyNameProvider;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  private Joystick m_leftStick;
  private WPI_TalonSRX Cim1_Motor;// CIM1
  private WPI_TalonSRX Cim2_Motor;// CIM2
  private CANSparkMax Neo1_motor;// Neo1
  private CANSparkMax Neo2_motor;// Neo2
  private WPI_TalonSRX A_pg; // ander vala pg
  private WPI_TalonSRX B_pg; // bahar vala pg
  // private final Encoder m_encoder = new Encoder(6, 7, false,
  // CounterBase.EncodingType.k4X);
  double Apg_encoder;
  double Bpg_encoder;
  double counterA;
  double counterB;
  double errorA;
  double errorB;
  private double A_pgSpeed;
  private double B_pgSpeed;
  double positionA;
  double positionB;
  double Cim_speed = 0.85;
  double Neo_speed = 0.85;

  boolean Ander = false;
  boolean Bahar = false;

  // double current;

  @Override
  public void robotInit() {
    // PowerDistribution TIS = new PowerDistribution(0, ModuleType.kCTRE);

    m_leftStick = new Joystick(0);
    // m_encoder.setSamplesToAverage(100);
    Cim1_Motor = new WPI_TalonSRX(7);// bahar wala
    Cim2_Motor = new WPI_TalonSRX(5);// Bahar wala
    Cim1_Motor.setNeutralMode(NeutralMode.Brake);
    Cim2_Motor.setNeutralMode(NeutralMode.Brake);
    Neo1_motor = new CANSparkMax(31, MotorType.kBrushless);// andar wala motor
    Neo2_motor = new CANSparkMax(32, MotorType.kBrushless);// ander wala motor
    Neo1_motor.setIdleMode(IdleMode.kBrake);
    Neo2_motor.setIdleMode(IdleMode.kBrake);
    A_pg = new WPI_TalonSRX(4);
    B_pg = new WPI_TalonSRX(6);

    // s_encoder = m_motor.getEncoder();
    Apg_encoder = A_pg.getSelectedSensorPosition();
    positionA = Apg_encoder / 5.5;
    counterA = positionA;

    Bpg_encoder = B_pg.getSelectedSensorPosition();
    positionB = Bpg_encoder / 5.5;
    counterB = positionB;

    // A_encoder = A_motor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature,
    // 255);
    // positionA = A_encoder.getPosition() *10.1;
    // counterA = positionA;

    // B_encoder = B_motor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature,
    // 255);
    // positionB = B_encoder.getPosition() *10.1;
    // counterB = positionB;
    /*
     * Defines how far the mechanism attached to the encoder moves per pulse. In
     * this case, we assume that a 360 count encoder is directly
     * attached to a 3 inch diameter (1.5inch radius) wheel,
     * and that we want to measure distance in inches.
     */
    // m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 10.1);

    /*
     * Defines the lowest rate at which the encoder will
     * not be considered stopped, for the purposes of
     * the GetStopped() method. Units are in distance / second,
     * where distance refers to the units of distance
     * that you are using, in this case inches.
     */
    // m_encoder.setMinRate(1.0);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    if (m_leftStick.getRawAxis(1) > 0.5) {
      Ander = true;
    } else if (m_leftStick.getRawAxis(1) < (-0.5)) {
      Ander = false;
    }

    if (m_leftStick.getRawAxis(5) > 0.5) {
      Bahar = true;
    } else if (m_leftStick.getRawAxis(5) < (-0.5)) {
      Bahar = false;
    }
    // Ander side
    if (m_leftStick.getRawButton(2) && !m_leftStick.getRawButton(1)) {
      counterA = counterA + 1;
    } else if (!m_leftStick.getRawButton(2) && m_leftStick.getRawButton(1)) {
      counterA = counterA - 1;
    }

    positionA = A_pg.getSelectedSensorPosition();// *10.1
    errorA = ((positionA - counterA));
    A_pgSpeed = 0.02 * errorA;

    if (A_pgSpeed > 0.9) {
      A_pgSpeed = 0.9;
    } else if (A_pgSpeed < -0.9) {
      A_pgSpeed = -0.9;
    }

    // B_TDMotor.set(-AnderSpeed);
    if (Ander) {
      A_pg.set(A_pgSpeed);
    } else {
      A_pg.set(0);
      counterA = positionA;
    }

    // Bahar side

    if (m_leftStick.getRawButton(3) && !m_leftStick.getRawButton(4)) {
      counterB = counterB + 1;
    } else if (!m_leftStick.getRawButton(3) && m_leftStick.getRawButton(4)) {
      counterB = counterB - 1;
    }

    positionB = B_pg.getSelectedSensorPosition();// *10.1
    errorB = ((positionB - counterB));
    B_pgSpeed = 0.02 * errorB;

    if (B_pgSpeed > 0.9) {
      B_pgSpeed = 0.9;
    } else if (B_pgSpeed < -0.9) {
      B_pgSpeed = -0.9;
    }

    // B_TDMotor.set(-AnderSpeed);
    if (Bahar) {
      B_pg.set(B_pgSpeed);
    } else {
      B_pg.set(0);
      counterB = positionB;
    }

    // if (m_leftStick.getRawButton(3) && !m_leftStick.getRawButton(4)) {
    // counterB= counterB+1;
    // } else if (!m_leftStick.getRawButton(3) && m_leftStick.getRawButton(4)) {
    // counterB= counterB-1;
    // }

    // positionB = B_encoder.getPosition() *10.1;
    // errorB = ((positionB - counterB)/10);
    // BaharSpeed= 0.2*errorB ;

    // if(BaharSpeed>0.9){
    // BaharSpeed=0.9;
    // }
    // else if(BaharSpeed<-0.9){
    // BaharSpeed=-0.9;
    // }

    // if (Bahar){
    // B_motor.set(BaharSpeed);
    // }
    // else{
    // B_motor.set(0);
    // counterB = positionB;
    // }

    // Top Down Motion CIM Motors

    if (m_leftStick.getRawButton(6) && !m_leftStick.getRawButton(5)) {
      Cim1_Motor.set(Cim_speed);
      Cim2_Motor.set(-Cim_speed);
    } else if (!m_leftStick.getRawButton(6) && m_leftStick.getRawButton(5)) {
      Cim1_Motor.set(-Cim_speed);
      Cim2_Motor.set(Cim_speed);
    } else {
      Cim1_Motor.set(0);
      Cim2_Motor.set(0);

      Cim1_Motor.setNeutralMode(NeutralMode.Brake);
      Cim2_Motor.setNeutralMode(NeutralMode.Brake);
    }

    // if (m_leftStick.getRawButton(7) && !m_leftStick.getRawButton(8)) {
    // A_TDMotor.set(A_TDspeed);
    // } else if (!m_leftStick.getRawButton(7) && m_leftStick.getRawButton(8)) {
    // A_TDMotor.set(-A_TDspeed);
    // } else {
    // A_TDMotor.set(0);
    // A_TDMotor.setNeutralMode(NeutralMode.Brake);
    // }

    // if (error>0) {
    // B_TDMotor.set(0.2);
    // } else if (error<0) {
    // B_TDMotor.set(AnderSpeed);
    // } else {
    // B_TDMotor.set(0);
    // B_TDMotor.setNeutralMode(NeutralMode.Brake);

    // }
    if (m_leftStick.getRawButton(8) && !m_leftStick.getRawButton(7)) {
      Neo1_motor.set(Neo_speed);
      Neo2_motor.set(-Neo_speed);
    } else if (!m_leftStick.getRawButton(8) && m_leftStick.getRawButton(7)) {
      Neo1_motor.set(-Neo_speed);
      Neo2_motor.set(Neo_speed);
    } else {
      Neo1_motor.set(0);
      Neo2_motor.set(0);

      Neo1_motor.setIdleMode(IdleMode.kBrake);
      Neo2_motor.setIdleMode(IdleMode.kBrake);
    }

    // SmartDashboard.putNumber("Encoder Distance", m_encoder.getDistance());
    // SmartDashboard.putNumber("Encoder Rate", m_encoder.getRate());
    SmartDashboard.putNumber("counterA", counterA);
    SmartDashboard.putNumber("counterB", counterB);

    SmartDashboard.putNumber("errorA", errorA);
    SmartDashboard.putNumber("errorB", errorB);

    SmartDashboard.putNumber("AnderPgspeed", A_pgSpeed);
    SmartDashboard.putNumber("BaharPgspeed", B_pgSpeed);

    SmartDashboard.putNumber("positionA", positionA);
    SmartDashboard.putNumber("positionB", positionB);

    SmartDashboard.putBoolean("Ander", Ander);
    SmartDashboard.putBoolean("Bahar", Bahar);

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
