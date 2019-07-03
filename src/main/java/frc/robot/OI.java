/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class OI {
  public XboxController driveJoyStick = new XboxController(0);

  public int getDpad() {
    return (driveJoyStick.getPOV());
  }

  public double getRightXDrive() {
    return (driveJoyStick.getX(Hand.kRight));
  }
  public double getRightYDrive() {
    return (driveJoyStick.getY(Hand.kRight));
  }
  public double getLeftYDrive() {
    return (driveJoyStick.getY(Hand.kLeft));
  }
  public double getLeftXDrive() {
    return (driveJoyStick.getX(Hand.kLeft));
  }

  public double getLeftTrigger() {
    return (driveJoyStick.getTriggerAxis(Hand.kLeft));
  }
  public double getRightTrigger() {
    return (driveJoyStick.getTriggerAxis(Hand.kRight));
  }
}
