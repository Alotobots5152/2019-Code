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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Add your docs here.
 */
public class WristSubSys extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WristSubSys(){
    RobotMap.Wrist_MtrDrive.setInverted(true);
    //RobotMap.Wrist_MtrDrive.setParameter(ConfigParameter.kSensorType, 1);
    RobotMap.Wrist_MtrDrive.setIdleMode(IdleMode.kCoast);
    RobotMap.Wrist_MtrDrive.setParameter(ConfigParameter.kHardLimitFwdEn, false);
    RobotMap.Wrist_MtrDrive.setParameter(ConfigParameter.kHardLimitRevEn, false);
  }

  public void Move(double spd){
    SmartDashboard.putNumber("WristMove",spd);
    RobotMap.Wrist_MtrDrive.set(spd);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
