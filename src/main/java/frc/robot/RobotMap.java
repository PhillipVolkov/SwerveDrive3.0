/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static int pcmcan = 5; //This must match the CAN id of the PCM. If it does not, pneumatics breaks
  public static int pdp = 0;
  public static int leftFrontDrive = 2;
  public static int leftFrontSteer = 1;
  public static int rightFrontDrive = 3;
  public static int rightFrontSteer = 4;
  public static int leftRearDrive = 5;
  public static int leftRearSteer = 6;
  public static int rightRearDrive = 7;
  public static int rightRearSteer = 8;
}
