// Version 1.3    2/13/2019   CCD   Added Arm, Wrist, Claw, Intake drives
// Version 1.2    2/12/2019   CCD   Added Lift_1_MtrDrive and Lift_2_MtrDrive
// Version 1.1    2/12/2019   CCD   Changed _TalonSRX to _MtrDrive to be generic
// Version 1.0    2/12/2019   CCD   Initial Release

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  // *******************************************************
  // Swerve Drive Motor Drives

  public static WPI_TalonSRX FR_Steer_MtrDrive = new WPI_TalonSRX(1);
  public static WPI_TalonSRX FR_Drive_MtrDrive = new WPI_TalonSRX(2);
  
  public static WPI_TalonSRX FL_Steer_MtrDrive = new WPI_TalonSRX(3);
  public static WPI_TalonSRX FL_Drive_MtrDrive = new WPI_TalonSRX(4);
  
  public static WPI_TalonSRX RL_Steer_MtrDrive = new WPI_TalonSRX(5);
  public static WPI_TalonSRX RL_Drive_MtrDrive = new WPI_TalonSRX(6);
  
  public static WPI_TalonSRX RR_Steer_MtrDrive = new WPI_TalonSRX(7);
  public static WPI_TalonSRX RR_Drive_MtrDrive = new WPI_TalonSRX(8);

  public static WPI_TalonSRX Lift_1_MtrDrive = new WPI_TalonSRX(9);
  public static WPI_TalonSRX Lift_2_MtrDrive = new WPI_TalonSRX(10);

  public static CANSparkMax Arm_1_MtrDrive = new CANSparkMax(11, MotorType.kBrushless);
  public static CANSparkMax Arm_2_MtrDrive = new CANSparkMax(12, MotorType.kBrushless);

  public static CANSparkMax Wrist_MtrDrive = new CANSparkMax(13, MotorType.kBrushless);
  
  public static WPI_TalonSRX Claw_MtrDrive = new WPI_TalonSRX(14);
  
  public static WPI_VictorSPX Intake_MtrDrive = new WPI_VictorSPX(15);

  public static WPI_VictorSPX Crawl_MtrDrive = new WPI_VictorSPX(16);
}