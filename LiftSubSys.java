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

/**
 * Add your docs here.
 */
public class LiftSubSys extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public LiftSubSys(){

    // Lift_1_Mtr Drive Init
    RobotMap.Lift_1_MtrDrive.setInverted(true);
    //Claw_MtrDrive.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    //Claw_MtrDrive.configFeedbackNotContinuous(false, 20);
    //Claw_MtrDrive.setSensorPhase(false);
    //Claw_MtrDrive.config_kP(0, 10);
    //Claw_MtrDrive.config_kI(0, 0);
    //Claw_MtrDrive.config_kD(0, 0);

    // Lift_2_Mtr Drive Init
    RobotMap.Lift_2_MtrDrive.setInverted(true);
    RobotMap.Lift_2_MtrDrive.follow(RobotMap.Lift_1_MtrDrive);
    RobotMap.Lift_2_MtrDrive.overrideLimitSwitchesEnable(true);
  }

  public void Move(double spd){
    RobotMap.Lift_1_MtrDrive.set(spd);
    //RobotMap.Lift_2_MtrDrive.set(spd);
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
