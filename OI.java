// Version 1.0      2/12/2019

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import frc.robot.commands.ArmInit;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public Joystick DriverAttackJoystick;
  public Joystick CoDriverGamePad;
  public Joystick ButtonBox;

  
  public OI() {

    // Driver Attack Joystick
    DriverAttackJoystick = new Joystick(0);
    DriverAttackJoystick.setXChannel(0);
    DriverAttackJoystick.setYChannel(1);
    DriverAttackJoystick.setZChannel(2);

    // CoDriver GamePad 
    CoDriverGamePad = new Joystick(1);
    CoDriverGamePad.setXChannel(0);
    CoDriverGamePad.setYChannel(1);



    /*
    joystick_2 = new Joystick(1);
    JoystickButton button1 = new JoystickButton(joystick_2, 1);
    JoystickButton button2 = new JoystickButton(joystick_2, 2);
    JoystickButton button3 = new JoystickButton(joystick_2, 3);
    JoystickButton button4 = new JoystickButton(joystick_2, 4);*/

    //button1.whenPressed(new ArmInit());
	}

  public double getDriveFwdAxis() {
    // Get Drive Forward Axis Cmd (-Y)
    return -DriverAttackJoystick.getY();
  }
  
  public double getDriveStrAxis() {
    // Get Drive Strafe Axis Cmd
    return DriverAttackJoystick.getX();
  }
  
  public double getDriveRot() {
    // Get Drive Rotate Cmd
    return DriverAttackJoystick.getZ();
  }
  
  public double getDriveMag() {
    return DriverAttackJoystick.getMagnitude();
  }

  public double getDriveDirection() {
    return DriverAttackJoystick.getDirectionDegrees();
  }
    
  public double getArmFwdAxis() {
    // Get Drive Forward Axis Cmd (-Y)
    return 0;
    //return joystick_2.getY();
  } 



  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
//JoystickButton(stick, 1).whenActive(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.

 //button2.whenPressed(new ArmMove(40.0));
 //button3.whenPressed(new ArmMove(60.0));
 //button4.whenPressed(new ArmMove(0.0));

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  

  
}

  



