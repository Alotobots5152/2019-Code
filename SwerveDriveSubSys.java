// version 1.5    2/16/2019  CCD  Changed DriveTrainMove method to Move and added default command 
// version 1.4    2/14/2019  APD  Changed all inverts and set encoder postions: Working drive 
// version 1.3    2/13/2019  APD  Changed Chassis Width and Length   
// Version 1.2    2/12/2019  CCD  Changed _TalonSRX to _MtrDrive to be generic
// Version 1.1    2/12/2019  CCD  Added getGyroAngle() and resetGyroAngle()
// Version 1.0    2/12/2019  CCD  Initial 


/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

// Dashboards
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

// Talon SRX
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

// Swerve
import org.usfirst.frc4048.swerve.drive.*;

// NavX
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

// Commands
import frc.robot.commands.SwerveDrive_JoystickControlCmd;

/**
 * Implements Swerve Drive based on Team 4048 code
 */

public class SwerveDriveSubSys extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Talon SRX
  public CanTalonSwerveEnclosure SwerveEnclosure_FR;
  public CanTalonSwerveEnclosure SwerveEnclosure_FL;
  public CanTalonSwerveEnclosure SwerveEnclosure_RL;
  public CanTalonSwerveEnclosure SwerveEnclosure_RR;

  // Swerve
  private static final double GEAR_RATIO = 1024;
  private static final double L = 21;  // inches
  private static final double W = 20;    // inches
  public SwerveDrive m_SwerveDrive;

  // NavX
  public AHRS navXGyro;

public SwerveDriveSubSys(){

  // Swerve Init
  SwerveEnclosure_FR = new CanTalonSwerveEnclosure("FR", RobotMap.FR_Drive_MtrDrive, RobotMap.FR_Steer_MtrDrive, GEAR_RATIO);
  SwerveEnclosure_FR.steerMotor.setInverted(true);
  SwerveEnclosure_FR.steerMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
  SwerveEnclosure_FR.steerMotor.configFeedbackNotContinuous(false, 20);
  SwerveEnclosure_FR.steerMotor.setSensorPhase(false);
  SwerveEnclosure_FR.steerMotor.config_kP(0, 10);
  SwerveEnclosure_FR.steerMotor.config_kI(0, 0);
  SwerveEnclosure_FR.steerMotor.config_kD(0, 0);

  SwerveEnclosure_FR.driveMotor.setInverted(true);

  SwerveEnclosure_FL = new CanTalonSwerveEnclosure("FL", RobotMap.FL_Drive_MtrDrive, RobotMap.FL_Steer_MtrDrive, GEAR_RATIO);
  SwerveEnclosure_FL.steerMotor.setInverted(true);
  SwerveEnclosure_FL.steerMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
  SwerveEnclosure_FL.steerMotor.configFeedbackNotContinuous(false, 20);
  SwerveEnclosure_FL.steerMotor.setSensorPhase(false);
  SwerveEnclosure_FL.steerMotor.config_kP(0, 10);
  SwerveEnclosure_FL.steerMotor.config_kI(0, 0);
  SwerveEnclosure_FL.steerMotor.config_kD(0, 0);

  SwerveEnclosure_FL.driveMotor.setInverted(true);

  SwerveEnclosure_RL = new CanTalonSwerveEnclosure("RL", RobotMap.RL_Drive_MtrDrive, RobotMap.RL_Steer_MtrDrive, GEAR_RATIO);
  SwerveEnclosure_RL.steerMotor.setInverted(true);
  SwerveEnclosure_RL.steerMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
  SwerveEnclosure_RL.steerMotor.configFeedbackNotContinuous(false, 20);
  SwerveEnclosure_RL.steerMotor.setSensorPhase(false);
  SwerveEnclosure_RL.steerMotor.config_kP(0, 10);
  SwerveEnclosure_RL.steerMotor.config_kI(0, 0);
  SwerveEnclosure_RL.steerMotor.config_kD(0, 0);

  SwerveEnclosure_RL.driveMotor.setInverted(false);

  SwerveEnclosure_RR = new CanTalonSwerveEnclosure("RR", RobotMap.RR_Drive_MtrDrive, RobotMap.RR_Steer_MtrDrive, GEAR_RATIO);
  SwerveEnclosure_RR.steerMotor.setInverted(false);
  SwerveEnclosure_RR.steerMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
  SwerveEnclosure_RR.steerMotor.configFeedbackNotContinuous(false, 20);
  SwerveEnclosure_RR.steerMotor.setSensorPhase(true);
  SwerveEnclosure_RR.steerMotor.config_kP(0, 10);
  SwerveEnclosure_RR.steerMotor.config_kI(0, 0);
  SwerveEnclosure_RR.steerMotor.config_kD(0, 0);
  
  SwerveEnclosure_RR.driveMotor.setInverted(false);

  m_SwerveDrive = new SwerveDrive(SwerveEnclosure_FR, SwerveEnclosure_FL, SwerveEnclosure_RL, SwerveEnclosure_RR, W, L);
 // m_SwerveDrive.setModeField();

  // NavX Init
  try 
  {
          /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
          /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
          /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
          navXGyro = new AHRS(SPI.Port.kMXP); 
          //navXGyro.reset();
    } 
  catch (RuntimeException ex ) 
  {
          DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

}

public void Move(double fwd, double str, double rcw){
  SmartDashboard.putNumber("Swerve Fwd", fwd);
  SmartDashboard.putNumber("Swerve Str", str);
  SmartDashboard.putNumber("Swerve Rcw", rcw);
  m_SwerveDrive.move(fwd, str, rcw, navXGyro.getAngle());
  //SwerveEnclosure_FR.driveMotor.set(.25);
  //SwerveEnclosure_FL.driveMotor.set(.25);
  //SwerveEnclosure_RR.driveMotor.set(.25);
 // SwerveEnclosure_RL.driveMotor.set(.25);

  //SwerveEnclosure_FR.steerMotor.set(.25);
  //SwerveEnclosure_FL.steerMotor.set(.25);
  //SwerveEnclosure_RR.steerMotor.set(.25);
  //SwerveEnclosure_RL.steerMotor.set(.25);

  SmartDashboard.putNumber("FR_Dir", SwerveEnclosure_FR.getEncPosition());
  SmartDashboard.putNumber("FL_Dir", SwerveEnclosure_FL.getEncPosition());
  SmartDashboard.putNumber("RL_Dir", SwerveEnclosure_RL.getEncPosition());
  SmartDashboard.putNumber("RR_Dir", SwerveEnclosure_RR.getEncPosition());
  SmartDashboard.putNumber("Gyro", navXGyro.getAngle());

  

}

public double getGyroAngle(){
  return navXGyro.getAngle();
}

public void resetGyroAngle(){
  navXGyro.reset();
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new SwerveDrive_JoystickControlCmd());
  }
}
