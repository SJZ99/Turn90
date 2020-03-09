/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends SubsystemBase {
  public static int turn = 0;
  double lastYaw = 0;
  WPI_TalonFX rightmotor = new WPI_TalonFX(18);
  WPI_TalonFX rightmotorS = new WPI_TalonFX(20);

  WPI_TalonFX leftmotor = new WPI_TalonFX(19);
  WPI_TalonFX leftmotorS = new WPI_TalonFX(21);

  AHRS ahrs = new AHRS(SPI.Port.kMXP);
  public static DifferentialDrive drive = new DifferentialDrive(leftmotor, rightmotor);
  // ADIS16448_IMU gyro;


  public Drivetrain() {
    // try{
    //   gyro = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, 0);
      
    // }catch(RuntimeException e){
    //   System.out.println(e.getMessage());
    // }
    ahrs.reset();
    rightmotor.configFactoryDefault();
    rightmotorS.configFactoryDefault();
    leftmotor.configFactoryDefault();
    leftmotorS.configFactoryDefault();

    rightmotor.setSelectedSensorPosition(0);
    leftmotor.setSelectedSensorPosition(0);

    rightmotorS.follow(rightmotor);
    leftmotorS.follow(leftmotor);

    // rightmotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0 ,0);
    // leftmotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0 ,0);

    rightmotor.setInverted(true);
    leftmotor.setInverted(false);

    rightmotorS.setInverted(InvertType.FollowMaster);
    leftmotorS.setInverted(InvertType.FollowMaster);

    // rightmotor.setSensorPhase(true);
    // leftmotor.setSensorPhase(false);

    rightmotor.configVoltageCompSaturation(10);
    leftmotor.configVoltageCompSaturation(10);
    rightmotor.enableVoltageCompensation(true);
    leftmotor.enableVoltageCompensation(true);
  }
  public double getYaw(){
    double a = ahrs.getAngle();
    return a;
  }
  public void setlastYaw(){
    lastYaw = ahrs.getAngle();
  }
  public double getlastYaw(){
    return lastYaw;
  }
  public void setOutput(double left, double right){
    rightmotor.set(ControlMode.PercentOutput, right);
    leftmotor.set(ControlMode.PercentOutput, left);
  }
  public void turn(int target){
    double curr = getYaw();
    target += lastYaw;
    double error = (target - curr);
    // double magnification = 0;
    // if(error < 0){
    //   magnification = -1;
    // }else{
    //   magnification = 1;
    // }
    SmartDashboard.putNumber("curr", curr);
    SmartDashboard.putNumber("target", target);
    setOutput(0.005 * error, -0.01 * error);
  }
  public void setTurn(int setpoint){
    turn = setpoint;
    }

  @Override
  public void periodic() {
 
  }
}
