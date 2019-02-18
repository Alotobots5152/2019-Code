
// Version 1.0    2/12/2019   CCD   Initial Release

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
//import frc.robot.commands.ArmInit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.ConfigParameter;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SensorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;


// /**
//  * Add your docs here.
//  */
// public class ArmSubSys extends Subsystem {
//   // Put methods for controlling this subsystem
//   // here. Call these from Commands.

//   // Put methods for controlling this subsystem
//   // here. Call these from Commands.
//   private CANDigitalInput arm_forwardLimit;
//   private CANDigitalInput arm_reverseLimit;
//   private CANEncoder arm_CanEncoder = RobotMap.Arm_1_MtrDrive.getEncoder();
//   private Double deadzone = 1.0;
//   private CANPIDController arm_pidController;


//   public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
//   public String kEnable;
//   public String kDisable;
//   public Double arm_Offset;
//   public static Boolean arm_positionReached = false;
//   public double currentPosition;


//   public void ArmSubSys() {
//     // PID coefficients
//    kP = 0.1; 
//    kI = 0;
//    kD = 0; 
//    kIz = 0; 
//    kFF = 0; 
//    kMaxOutput = .5; 
//    kMinOutput = -.5;

//    // set PID coefficients
//    arm_pidController.setP(kP);
//    arm_pidController.setI(kI);
//    arm_pidController.setD(kD);
//    arm_pidController.setIZone(kIz);
//    arm_pidController.setFF(kFF);
//    arm_pidController.setOutputRange(kMinOutput, kMaxOutput);
//  }

//  @Override
//  public void initDefaultCommand() {
//    // Set the default command for a subsystem here.
//    // setDefaultCommand(new MySpecialCommand());

//   }
  
//   public void arm_UP() {
//     RobotMap.Arm_1_MtrDrive.set(.5);
//   }


//   public void arm_DOWN() {
//     RobotMap.Arm_1_MtrDrive.set(-.5);
//   }


//   public void arm_STOP() {
//     RobotMap.Arm_1_MtrDrive.stopMotor();
//   }

//   public void arm_Run2Position(Double position) {
//     currentPosition = (arm_CanEncoder.getPosition() * -1);  //  get current position of arm
//     arm_pidController = RobotMap.Arm_1_MtrDrive.getPIDController();
//     arm_CanEncoder = RobotMap.Arm_1_MtrDrive.getEncoder();


//     // display PID coefficients on SmartDashboard
//     SmartDashboard.putNumber("P Gain", kP);
//     SmartDashboard.putNumber("I Gain", kI);
//     SmartDashboard.putNumber("D Gain", kD);
//     SmartDashboard.putNumber("I Zone", kIz);
//     SmartDashboard.putNumber("Feed Forward", kFF);
//     SmartDashboard.putNumber("Max Output", kMaxOutput);
//     SmartDashboard.putNumber("Min Output", kMinOutput);
//     SmartDashboard.putNumber("Set Rotations", 0);

//     // read PID coefficients from SmartDashboard
//     double p = SmartDashboard.getNumber("P Gain", 0);
//     double i = SmartDashboard.getNumber("I Gain", 0);
//     double d = SmartDashboard.getNumber("D Gain", 0);
//     double iz = SmartDashboard.getNumber("I Zone", 0);
//     double ff = SmartDashboard.getNumber("Feed Forward", 0);
//     double max = SmartDashboard.getNumber("Max Output", 0);
//     double min = SmartDashboard.getNumber("Min Output", 0);
//     double rotations = SmartDashboard.getNumber("Set Rotations", 0);

//     // if PID coefficients on SmartDashboard have changed, write new values to controller
//     if((p != kP)) { arm_pidController.setP(p); kP = p; }
//     if((i != kI)) { arm_pidController.setI(i); kI = i; }
//     if((d != kD)) { arm_pidController.setD(d); kD = d; }
//     if((iz != kIz)) { arm_pidController.setIZone(iz); kIz = iz; }
//     if((ff != kFF)) { arm_pidController.setFF(ff); kFF = ff; }
//     if((max != kMaxOutput) || (min != kMinOutput)) { 
//       arm_pidController.setOutputRange(min, max); 
//       kMinOutput = min; kMaxOutput = max; 
      
//     }

//     arm_pidController.setReference((position + ArmInit.arm_Position_Offset)*-1, ControlType.kPosition);

//     SmartDashboard.putNumber("SetPoint", rotations);
//     SmartDashboard.putNumber("ProcessVariable", arm_CanEncoder.getPosition());
//     SmartDashboard.putNumber("requested Position", position);
//     /* currentPosition = (arm_CanEncoder.getPosition() * -1);  //  get current position of arm
//     SmartDashboard.putNumber("current encoder Position", currentPosition);  
//     SmartDashboard.putNumber("requested Position", position);
//     SmartDashboard.putNumber("arm_offset sent", ArmInit.arm_Position_Offset); */
    
//     if (currentPosition < (position + ArmInit.arm_Position_Offset - deadzone))
//     {
//       //  if arm is up too far from desired position, run it back
//       SmartDashboard.putNumber("current Position", currentPosition);
//       SmartDashboard.putNumber("current offset position", (currentPosition - ArmInit.arm_Position_Offset));
//       //arm_DOWN();
//       arm_positionReached = false;
//     }  
//     else if (currentPosition > (position + ArmInit.arm_Position_Offset + deadzone))
//     {
//       //  if arm is down too far from desired postion, run it up
//       SmartDashboard.putNumber("current offset position", (currentPosition - ArmInit.arm_Position_Offset));
//       SmartDashboard.putNumber("current Position", currentPosition);
//       //arm_UP();
//       arm_positionReached = false;
//     }
//     else 
//     {
//       //  current position is within the deadzone of the desired position
//       arm_STOP();
//       arm_positionReached = true;
//       SmartDashboard.putBoolean("PostionReached?", arm_positionReached);  // return true to command to stop processing


//     }

//   }

//   public void arm_Joystick(Double JoyY) {
//     RobotMap.Arm_1_MtrDrive.set(JoyY);
//     SmartDashboard.putNumber("current encoder Position", currentPosition);
//     //SmartDashboard.putNumber("current offset position", (currentPosition - ArmInit.arm_Position_Offset));
//   }

// /**
//  * Add your docs here.
//  */
public class ArmSubSys extends Subsystem {
//   // Put methods for controlling this subsystem
//   // here. Call these from Commands.

//   // Put methods for controlling this subsystem
//   // here. Call these from Commands.
//   private CANDigitalInput arm_forwardLimit;
//   private CANDigitalInput arm_reverseLimit;
//   private CANEncoder arm_CanEncoder = RobotMap.Arm_1_MtrDrive.getEncoder();
//   private Double deadzone = 1.0;
//   private CANPIDController arm_pidController;


//   public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
//   public String kEnable;
//   public String kDisable;
//   public Double arm_Offset;
//   public static Boolean arm_positionReached = false;
//   public double currentPosition;

  public ArmSubSys() {
    // PID coefficients
   //kP = 0.1; 
  //  kI = 0;
  //  kD = 0; 
  //  kIz = 0; 
  //  kFF = 0; 
  //  kMaxOutput = .5; 
  //  kMinOutput = -.5;

  //  // set PID coefficients
  //  arm_pidController.setP(kP);
  //  arm_pidController.setI(kI);
  //  arm_pidController.setD(kD);
  //  arm_pidController.setIZone(kIz);
  //  arm_pidController.setFF(kFF);
  //  arm_pidController.setOutputRange(kMinOutput, kMaxOutput);
  RobotMap.Arm_1_MtrDrive.setInverted(true);
  //RobotMap.Arm_1_MtrDrive.setParameter(ConfigParameter.kSensorType, 1);
  RobotMap.Arm_1_MtrDrive.setIdleMode(IdleMode.kCoast);
  RobotMap.Arm_1_MtrDrive.setParameter(ConfigParameter.kHardLimitFwdEn, false);
  RobotMap.Arm_1_MtrDrive.setParameter(ConfigParameter.kHardLimitRevEn, false);

  RobotMap.Arm_2_MtrDrive.setInverted(false);
  //RobotMap.Arm_2_MtrDrive.setParameter(ConfigParameter.kSensorType, 1);
  RobotMap.Arm_2_MtrDrive.setIdleMode(IdleMode.kCoast);
  RobotMap.Arm_2_MtrDrive.setParameter(ConfigParameter.kHardLimitFwdEn, false);
  RobotMap.Arm_2_MtrDrive.setParameter(ConfigParameter.kHardLimitRevEn, false);
  //RobotMap.Arm_2_MtrDrive.follow(RobotMap.Arm_1_MtrDrive);
 }

 public void Move(double spd){
  RobotMap.Arm_1_MtrDrive.set(spd);
  RobotMap.Arm_2_MtrDrive.set(spd);
}

 @Override
 public void initDefaultCommand() {
   // Set the default command for a subsystem here.
   // setDefaultCommand(new MySpecialCommand());

  }
}
  