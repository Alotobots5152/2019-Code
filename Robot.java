// Version 2.0    2/16/2019  CCD  Changed Names of m_drivetrain to DriveTrain, m_oi to oi
// Version 1.0    2/12/2019  CCD  Initial 

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// Base Classes
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotMap;
import frc.robot.OI;

// Subsystems
import frc.robot.subsystems.ArmSubSys;
import frc.robot.subsystems.ClawSubSys;
import frc.robot.subsystems.CrawlSubSys;
import frc.robot.subsystems.IntakeSubSys;
import frc.robot.subsystems.LiftSubSys;
import frc.robot.subsystems.LimelightSubSys;
import frc.robot.subsystems.SwerveDriveSubSys;
import frc.robot.subsystems.WristSubSys;

// Components
import org.usfirst.frc4048.swerve.drive.SwerveDrive;

import java.nio.channels.WritableByteChannel;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  public static OI oi;

  // Subsystems
  public static SwerveDriveSubSys DriveTrain = new SwerveDriveSubSys();
  //public static ArmSubSys ArmMotors = new ArmSubSys();
  public static ArmSubSys Arm = new ArmSubSys();
  public static ClawSubSys Claw = new ClawSubSys();
  public static WristSubSys Wrist = new WristSubSys();
  public static LiftSubSys Lift = new LiftSubSys();
  public static IntakeSubSys Intake = new IntakeSubSys();

  // NetworkTable
  public NetworkTable table;
  public NetworkTableEntry tx;
  public NetworkTableEntry ty;
  public NetworkTableEntry ta;


  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI();

    //m_chooser.setDefaultOption("Default Auto", new LimelightTargeting());
    // chooser.addOption("My Auto", new MyAutoCommand());
    //SmartDashboard.putData("Auto mode", m_chooser); 


    DriveTrain.navXGyro.reset();    
    
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    Arm.Move(0);
    Wrist.Move(0);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putNumber("Joystick Fwd", oi.getDriveFwdAxis());
    SmartDashboard.putNumber("Joystick Strafe", oi.getDriveStrAxis());
    SmartDashboard.putNumber("Joystick Direction", oi.getDriveDirection());
    SmartDashboard.putNumber("Joystick Mag", oi.getDriveMag());
    SmartDashboard.putNumber("Joystick Rot", oi.getDriveRot());
    //SmartDashboard.putNumber("Gyro", Drive.navXGyro.getAngle());
   // Drive.drivetrainMove(oi.getDriveFwdAxis(), oi.getDriveStrAxis(), oi.getDriveRot());  
    
   // double x = tx.getDouble(0.0);
    //double y = ty.getDouble(0.0);
    //double area = ta.getDouble(0.0);

   // SmartDashboard.putNumber("tx", x);
   // SmartDashboard.putNumber("ty", y);
//SmartDashboard.putNumber("area", area);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    // Joystick Inputs
    SmartDashboard.putNumber("Joystick Fwd", oi.getDriveFwdAxis());
    SmartDashboard.putNumber("Joystick Strafe", oi.getDriveStrAxis());
    SmartDashboard.putNumber("Joystick Direction", oi.getDriveDirection());
    SmartDashboard.putNumber("Joystick Mag", oi.getDriveMag());
    SmartDashboard.putNumber("Joystick Rot", oi.getDriveRot());
    //SmartDashboard.putNumber("Gyro", Drive.navXGyro.getAngle());


    // DriveTrain
    DriveTrain.Move(oi.getDriveFwdAxis(), oi.getDriveStrAxis(), oi.getDriveRot());  

    // Lift
    Lift.Move(-oi.CoDriverGamePad.getY());

    // Arm
    Arm.Move(oi.CoDriverGamePad.getX());
  
    // Wrist
    //Wrist.Move(-oi.CoDriverGamePad.getRawAxis(5));
    RobotMap.Wrist_MtrDrive.set(-oi.CoDriverGamePad.getRawAxis(5));
    SmartDashboard.putNumber("Wristcmd",-oi.CoDriverGamePad.getRawAxis(5));

    // Claw
    if (oi.CoDriverGamePad.getRawButton(3)){
      Claw.Move(.25);
    }else if(oi.CoDriverGamePad.getRawButton(2)){
      Claw.Move(-.25);
    }else{
      Claw.Move(0);
    }

    // Intake
    if (oi.CoDriverGamePad.getRawButton(6)){
      Intake.Move(1);
    } else if (oi.CoDriverGamePad.getRawButton(5)){
      Intake.Move(-1);
    } else {
      Intake.Move(0);
    }

    //RobotMap.Arm_1_MtrDrive.set(oi.CoDriverGamePad.getX());   

  }
}
