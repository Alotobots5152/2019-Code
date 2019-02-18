// Version 1.1    2/16/2019   CCD   Added Claw Motor 
// Version 1.0    2/12/2019   CCD   Initial Release

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

// Dashboards
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

// Talon SRX
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class ClawSubSys extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  public ClawSubSys(){
    //RobotMap.Claw_MtrDrive.setInverted(true);
    //Claw_MtrDrive.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    //Claw_MtrDrive.configFeedbackNotContinuous(false, 20);
    //Claw_MtrDrive.setSensorPhase(false);
    //Claw_MtrDrive.config_kP(0, 10);
    //Claw_MtrDrive.config_kI(0, 0);
    //Claw_MtrDrive.config_kD(0, 0);
  }

  public void Move(double spd){
    RobotMap.Claw_MtrDrive.set(spd);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
