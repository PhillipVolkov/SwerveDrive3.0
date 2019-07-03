/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class TurnByCommand extends Command {
  private double m_turnDegree = 0;
  private double m_targetDegree = 0;
  private double startingDegree = 0;
  private double echo = 2;
  public double m_speed = 0;
  private double m_startingAngle = Robot.driver.startingAngle;
  private boolean precisionTurn = false;
  boolean arcadeDrive;
  public double turnAmount;
  public int module;
  public double DPAngle;

  public TurnByCommand(int module1, double DPadAngle) {
    System.out.println(DPadAngle);
    module = module1;
    DPAngle = DPadAngle;
    //System.out.print(" " + module);

    if(turnAmount >= 0) {    
      m_turnDegree = turnAmount-echo;
    }
    else {
      m_turnDegree = turnAmount+echo;
    }

    if(Math.abs(m_turnDegree) < 10) {
      precisionTurn = true;
    }
    else {
      precisionTurn = false;
    }

  }

  protected void TurnTo(int module, double angle) {
    turnAmount = angle-Robot.driver.getHeading(module);
    Robot.driver.startingAngle = angle-Robot.driver.getHeading(module);
    if(turnAmount > 180) {
      turnAmount -= 360;
      Robot.driver.startingAngle -= 360;
    }
    else if (turnAmount < -180) {
      turnAmount += 360;
      Robot.driver.startingAngle += 360;
    }

    if(turnAmount > 90) {
      //System.out.print(turnAmount + ",+ ");
      turnAmount = turnAmount-180;
      //System.out.println(turnAmount);
      Robot.driver.reversed = true;
    }
    else if(turnAmount < -90) {
      //System.out.print(turnAmount + ",- ");
      turnAmount = 180+turnAmount;
      //System.out.println(turnAmount);
      Robot.driver.reversed = true;
    }

    if(!Robot.driver.turning) {
      Robot.driver.startingHeading = Robot.driver.getHeading(module);
    }

    //System.out.print(" " + Robot.swerveModule.m_module);
    if(Math.abs(turnAmount) > 5) {
      Robot.driver.turning = true;
      if(turnAmount < 90 && turnAmount > -90) {
        Robot.driver.reversed = false;
      }
    }
    else {
      Robot.driver.turnSpeed = 0;
      Robot.driver.turning = false;
    }
    //System.out.println(this.accel);
    //System.out.print(turnAmount + " "); System.out.print(Robot.driver.startingAngle + " "); System.out.println(Robot.driver.turnSpeed);
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    startingDegree = Robot.driver.getHeading(module);

    m_targetDegree = m_turnDegree + startingDegree;

    if (m_targetDegree > 360) {
      m_targetDegree -= 360;
    }
    else if (m_targetDegree < 0) {
      m_targetDegree += 360;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    TurnTo(module, DPAngle);
    double m_currentDegree = Robot.driver.getHeading(module);
    m_startingAngle = Robot.driver.startingAngle;
    if(Math.abs(m_turnDegree) > 20) {
      m_speed = ((m_turnDegree)/(m_startingAngle));
    }
    else {
      m_speed = ((m_turnDegree)/(m_startingAngle+(m_turnDegree/6)));
  }
  //sets speed bounds
  if (m_speed > 0.75 && m_speed > 0) {
    m_speed = 0.75;
  }
  else if (m_speed < 0.2 && m_speed > 0 && !precisionTurn) {
    m_speed = 0.2;
  }
  else if (m_speed < 0.1 && m_speed > 0 && precisionTurn) {
    m_speed = 0.1;
  }
  else if (m_speed < -0.75 && m_speed < 0) {
    m_speed = -0.75;
  }
  else if (m_speed > -0.2 && m_speed < 0 && !precisionTurn) {
    m_speed = -0.1;
  }
  else if (m_speed > -0.1 && m_speed < 0 && precisionTurn) {
    m_speed = -0.1;
  }

  if (m_startingAngle > 0) {
    m_speed = -m_speed;
    if(m_targetDegree-m_currentDegree > -echo && m_targetDegree-m_currentDegree < echo ) {
       m_speed = 0;
       Robot.driver.turning = false;
      }
    }
  else {
    if(m_targetDegree-m_currentDegree > -echo && m_targetDegree-m_currentDegree < echo ) {
      m_speed = 0;
      Robot.driver.turning = false;
    }
  } 
  if(Robot.driver.turning) {//Math.abs(angle-Robot.driver.getHeading()) > 5) {
    //Robot.swerveModule.m_module = 0;
    System.out.println(m_speed);
    
    if (module == 1){
    Robot.driver.leftFrontSteer.set(this.m_speed);
    }
    else if (module == 2){
    Robot.driver.leftRearSteer.set(this.m_speed);
    }
    //Robot.driver.rightFrontAngle.set(Robot.swerveModule.turnSpeed[2]);
    //Robot.driver.rightRearAngle.set(Robot.swerveModule.turnSpeed[3]);

    //System.out.println("Turning " + Robot.swerveModule.turnSpeed[0]);
   }
   else {
     if(Robot.driver.accel != 0) {
       if(Robot.driver.reversed && Robot.driver.accel > 0) {
          Robot.driver.accel = -Robot.driver.accel;
       }
      // Robot.driver.allSwerve(Robot.swerveModule.accel, 0);//(Math.sqrt(Math.pow(turn, 2)+Math.pow(forward,2))), 0);
       
      // System.out.println("Moving " + Robot.swerveModule.accel);
     }
     else {
       System.out.println("Stopping");
       Robot.driver.stop();
     }
   }
}

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //System.out.println(Robot.swerveModule.m_module);System.out.println(Robot.swerveModule.m_module);System.out.println(Robot.swerveModule.m_module);
    //System.out.println(" " + module);
    Robot.driver.turnSpeed = m_speed*0.2;
    //System.out.println(m_speed + " " + Robot.swerveModule.turnSpeed[Robot.swerveModule.m_module]);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
*/