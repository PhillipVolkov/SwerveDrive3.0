/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
package frc.robot.subsystems;

import frc.robot.OI;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.XboxController;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.RobotMap;
import frc.robot.commands.TurnAllByCommand;
import edu.wpi.first.wpilibj.command.Scheduler;

public class SwerveDriveSubsystem extends Subsystem {
  public static CANSparkMax leftFrontDrive;
  public static CANSparkMax leftFrontSteer;
  public static CANSparkMax rightFrontDrive;
  public static CANSparkMax rightFrontSteer;
  public static CANSparkMax leftRearDrive;
  public static CANSparkMax leftRearSteer;
  public static CANSparkMax rightRearDrive;
  public static CANSparkMax rightRearSteer;


  public static CANEncoder leftFrontAngleEncoder;
  public static CANEncoder rightFrontAngleEncoder;
  public static CANEncoder leftRearAngleEncoder;
  public static CANEncoder rightRearAngleEncoder;

  public XboxController m_mainJoyStick;
  public static boolean isBackward;
  public static String defaultDrive = "SwerveDriveCommand";

  public double startingAngle = 0;
  public double angle;
  public boolean turning;
  public boolean reversed;
  public double startingHeading;
  public double turnSpeed;
  public double accel;
  //private final AHRS navX;

  public SwerveDriveSubsystem(){
    leftFrontDrive = new CANSparkMax(RobotMap.leftFrontDrive, MotorType.kBrushless);
    leftFrontSteer = new CANSparkMax(RobotMap.leftFrontSteer, MotorType.kBrushless);
    rightFrontDrive = new CANSparkMax(RobotMap.rightFrontDrive, MotorType.kBrushless);
    rightFrontSteer = new CANSparkMax(RobotMap.rightFrontSteer, MotorType.kBrushless);
    leftRearDrive = new CANSparkMax(RobotMap.leftRearDrive, MotorType.kBrushless);
    leftRearSteer = new CANSparkMax(RobotMap.leftRearSteer, MotorType.kBrushless);
    rightRearDrive = new CANSparkMax(RobotMap.rightRearDrive, MotorType.kBrushless);
    rightRearSteer = new CANSparkMax(RobotMap.rightRearSteer, MotorType.kBrushless);

    leftFrontAngleEncoder = new CANEncoder(leftFrontSteer);
    leftRearAngleEncoder = new CANEncoder(leftRearSteer);
    rightFrontAngleEncoder = new CANEncoder(rightFrontSteer);
    rightRearAngleEncoder = new CANEncoder(rightRearSteer);
    leftFrontAngleEncoder.setPositionConversionFactor(19.969);
    leftRearAngleEncoder.setPositionConversionFactor(19.969);
    rightFrontAngleEncoder.setPositionConversionFactor(19.969);
    rightRearAngleEncoder.setPositionConversionFactor(19.969);

    startingHeading = 0;
}


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void periodic() {
    if(Robot.m_oi.getDpad() != -1) {
      this.turning = true;
      angle = Robot.m_oi.getDpad();
    }
    else {
      this.turnSpeed = 0;
      this.turning = false;
    }
    Scheduler.getInstance().add(new TurnAllByCommand());
  }

  public double getHeading(int module) {
    double heading;
    if(module == 0) {
      heading = 360-leftFrontAngleEncoder.getPosition();
    }
    else if(module == 1) {
      heading = 360-leftRearAngleEncoder.getPosition();
    }
    else if(module == 2) {
      heading = 360-rightFrontAngleEncoder.getPosition();
    }
    else if(module == 3) {
      heading = 360-rightRearAngleEncoder.getPosition();
    }
    else {
      heading = 0;
    }

    if (heading < 0) {
      //System.out.println(360 - (Math.abs(heading) % 360));
      return 360 - (Math.abs(heading) % 360);
    } else {
      //System.out.println(Math.abs(heading) % 360);
      return Math.abs(heading) % 360;
    }
  }

  public void stop() {
    leftFrontSteer.set(0);
    leftFrontDrive.set(0);
    leftRearSteer.set(0);
    leftRearDrive.set(0);
    rightFrontSteer.set(0);
    rightFrontDrive.set(0);
    rightRearSteer.set(0);
    rightRearDrive.set(0);
  }
}
*/
