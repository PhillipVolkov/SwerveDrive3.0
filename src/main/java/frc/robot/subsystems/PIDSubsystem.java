/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ResetEncodersCommand;

import frc.robot.OI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;


/**
 * Add your docs here.
 */
public class PIDSubsystem extends Subsystem {
  private CANSparkMax frontLeftAngle;
  private CANPIDController frontLeftPID;
  public CANEncoder frontLeftAngleEncoder;

  private CANSparkMax frontRightAngle;
  private CANPIDController frontRightPID;
  public CANEncoder frontRightAngleEncoder;

  private CANSparkMax rearLeftAngle;
  private CANPIDController rearLeftPID;
  public CANEncoder rearLeftAngleEncoder;

  private CANSparkMax rearRightAngle;
  private CANPIDController rearRightPID;
  public CANEncoder rearRightAngleEncoder;

  private CANSparkMax frontLeftDrive;
  private CANSparkMax frontRightDrive;
  private CANSparkMax rearLeftDrive;
  private CANSparkMax rearRightDrive;

  public PowerDistributionPanel pdp;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double rotations;
  public double rotations2;
  public double accel;
  public boolean turning;
  public double encoderConversionFactor =  19.969;
  public boolean reversed;
  private final AHRS navX;
  public double resetE = 0;
  public int passed360 = 0;

  public double p, i, d, iz, ff, max, min;
  //public OI m_oi;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public PIDSubsystem() {
    // initialize motor
    frontLeftAngle = new CANSparkMax(RobotMap.leftFrontSteer, MotorType.kBrushless);
    frontRightAngle = new CANSparkMax(RobotMap.rightFrontSteer, MotorType.kBrushless);
    rearLeftAngle = new CANSparkMax(RobotMap.leftRearSteer, MotorType.kBrushless);
    rearRightAngle = new CANSparkMax(RobotMap.rightRearSteer, MotorType.kBrushless);
    
    frontLeftDrive = new CANSparkMax(RobotMap.leftFrontDrive, MotorType.kBrushless);
    frontRightDrive = new CANSparkMax(RobotMap.rightFrontDrive, MotorType.kBrushless);
    rearLeftDrive = new CANSparkMax(RobotMap.leftRearDrive, MotorType.kBrushless);
    rearRightDrive = new CANSparkMax(RobotMap.rightRearDrive, MotorType.kBrushless);
    
    navX = new AHRS(SPI.Port.kMXP);

    //frontLeftAngle.restoreFactoryDefaults();
    //frontRightAngle.restoreFactoryDefaults();
    //rearLeftAngle.restoreFactoryDefaults();
    //rearRightAngle.restoreFactoryDefaults();

    frontLeftAngle.setSmartCurrentLimit(60);
    frontLeftDrive.setSmartCurrentLimit(60);
    frontLeftDrive.setClosedLoopRampRate(0.5);

    frontRightAngle.setSmartCurrentLimit(60);
    frontRightDrive.setSmartCurrentLimit(60);
    frontRightDrive.setClosedLoopRampRate(0.5);

    rearLeftAngle.setSmartCurrentLimit(60);
    rearLeftDrive.setSmartCurrentLimit(60);
    rearLeftDrive.setClosedLoopRampRate(0.5);

    rearRightAngle.setSmartCurrentLimit(60);
    rearRightDrive.setSmartCurrentLimit(60);
    rearRightDrive.setClosedLoopRampRate(0.5);
    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    frontLeftPID = frontLeftAngle.getPIDController();
    frontRightPID = frontRightAngle.getPIDController();
    rearLeftPID = rearLeftAngle.getPIDController();
    rearRightPID = rearRightAngle.getPIDController();

    // Encoder object created to display position values
    frontLeftAngleEncoder = frontLeftAngle.getEncoder();
    frontRightAngleEncoder = frontRightAngle.getEncoder();
    rearLeftAngleEncoder = rearLeftAngle.getEncoder();
    rearRightAngleEncoder = rearRightAngle.getEncoder();
    
    // PID coefficients
    kP = 0.5; 
    kI = 1e-4;
    kD = 0.0001; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.75;
    kMinOutput = -0.75;

    // set PID coefficients
    setPidControllers(frontLeftPID);
    setPidControllers(frontRightPID);
    setPidControllers(rearLeftPID);
    setPidControllers(rearRightPID);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Reset Encoder", 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void periodic() {
    // read PID coefficients from SmartDashboard\
    SmartDashboard.putData("Resetiing", new ResetEncodersCommand());
    
    p = SmartDashboard.getNumber("P Gain", 0);
    i = SmartDashboard.getNumber("I Gain", 0);
    d = SmartDashboard.getNumber("D Gain", 0);
    iz = SmartDashboard.getNumber("I Zone", 0);
    ff = SmartDashboard.getNumber("Feed Forward", 0);
    max = SmartDashboard.getNumber("Max Output", 0);
    min = SmartDashboard.getNumber("Min Output", 0);
    resetE = SmartDashboard.getNumber("Reset Encoder", 0);
    
    if (resetE == 1){
      Scheduler.getInstance().add(new ResetEncodersCommand());
    }
   // double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    SmartDashboard.putNumber("Set Rotations", rotations);   

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    
    changePidController(frontLeftPID);
    changePidController(frontRightPID);
    changePidController(rearLeftPID);
    changePidController(rearRightPID);

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.ControlType.kDutyCycle
     *  com.revrobotics.ControlType.kPosition
     *  com.revrobotics.ControlType.kVelocity
     *  com.revrobotics.ControlType.kVoltage
     */

    calculateAngle();
    double forward = -Robot.m_oi.getRightYDrive();
    double turn = -Robot.m_oi.getRightXDrive();

    if((Robot.m_oi.getLeftTrigger() > 0.1) || (Robot.m_oi.getRightTrigger() > 0.1)) {
      rotations2 = 45/encoderConversionFactor;
      rotations = 135/encoderConversionFactor;
    }
    else if ((forward > 0.1 || forward < -0.1) || (turn > 0.1 || turn < -0.1)){
      rotations2 = rotations;
    }
    
    frontLeftPID.setReference(rotations, ControlType.kPosition);
    frontRightPID.setReference(rotations2, ControlType.kPosition);
    rearLeftPID.setReference(rotations2, ControlType.kPosition);
    rearRightPID.setReference(rotations, ControlType.kPosition);

    /*if(Math.abs(frontLeftAngleEncoder.getPosition()-rotations) < 5 && 
    Math.abs(frontRightAngleEncoder.getPosition()-rotations) < 5 &&
    Math.abs(rearLeftAngleEncoder.getPosition()-rotations) < 5 &&
    Math.abs(rearLeftAngleEncoder.getPosition()-rotations) < 5) { */
      this.turning = false;
      if((forward > 0.1 || forward < -0.1) || (turn > 0.1 || turn < -0.1)) {
        calculateAngle();
        System.out.println("Moving: " + accel);
        moveSwerve(accel);
      }
      else if(Robot.m_oi.getLeftTrigger() > 0.1) {
        accel = -Robot.m_oi.getLeftTrigger();
        frontLeftDrive.set(accel);
        frontRightDrive.set(accel);
        rearLeftDrive.set(-accel);
        rearRightDrive.set(-accel);
      }
      else if(Robot.m_oi.getRightTrigger() > 0.1) {
        accel = Robot.m_oi.getRightTrigger();
        frontLeftDrive.set(accel);
        frontRightDrive.set(accel);
        rearLeftDrive.set(-accel);
        rearRightDrive.set(-accel);
      }
      else {
        moveSwerve(0);
      }
    //}
    //else {
     // this.turning = true;
     // moveSwerve(0);
    //}
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", frontLeftAngleEncoder.getPosition());
  }

  public void setPidControllers (CANPIDController pidController) {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void changePidController (CANPIDController pidController) {
    if((p != kP)) { pidController.setP(p); kP = p; }
    if((i != kI)) { pidController.setI(i); kI = i; }
    if((d != kD)) { pidController.setD(d); kD = d; }
    if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }

    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
  }

  public void moveSwerve (double forward) {
    frontLeftDrive.set(forward);
    frontRightDrive.set(forward);
    rearLeftDrive.set(forward);
    rearRightDrive.set(forward);
  }

  public double getHeading(int module) {
    double heading;
    if(module == 0) {
      heading = 360-frontLeftAngleEncoder.getPosition();
    }
    else if(module == 1) {
      heading = 360-frontRightAngleEncoder.getPosition();
    }
    else if(module == 2) {
      heading = 360-rearLeftAngleEncoder.getPosition();
    }
    else if(module == 3) {
      heading = 360-rearRightAngleEncoder.getPosition();
    }
    else {
      heading = navX.getAngle(); 
    }

    if(module >= 0 && module <= 3) {
      heading*=encoderConversionFactor;
    }

    if (heading < 0) {
      //System.out.println(360 - (Math.abs(heading) % 360));
      return 360 - (Math.abs(heading) % 360);
    } else {
      //System.out.println(Math.abs(heading) % 360);
      return Math.abs(heading) % 360;
    }
  }

  public void calculateAngle() {
    double forward = -Robot.m_oi.getRightYDrive();
    double turn = -Robot.m_oi.getRightXDrive();

    double forwardField = -Robot.m_oi.getLeftYDrive();
    double turnField = -Robot.m_oi.getLeftXDrive();
    double heading = getHeading(-1);

    if (Robot.m_oi.getDpad() != -1) {
      Timer.delay(0.5);

      if (Robot.m_oi.getDpad() != -1) {
        rotations = (360 - Robot.m_oi.getDpad()) / 19.969;
      }
    }

    if((forwardField > 0.1 || forwardField < -0.1) || (turnField > 0.1 || turnField < -0.1)) {
      if(turnField >= 0) {
        if(forwardField >= 0) {
          //System.out.print(4 + " ");
          rotations = 270+Math.atan(forwardField/turnField)*180/Math.PI;
        }
        if(forwardField < 0) {
          //System.out.print(3 + " ");
          rotations = 270+Math.atan(forwardField/turnField)*180/Math.PI;
        }
      }
      if(turnField < 0) {
        if(forwardField <= 0) {
          //System.out.print(2 + " ");
          rotations = 90+Math.atan(forwardField/turnField)*180/Math.PI;
        }
        if(forwardField > 0) {
          //System.out.print(1 + " ");
          rotations = -Math.atan(turnField/forwardField)*180/Math.PI;
        }
      }
      //rotations+=heading;

      if(rotations < 90) {
        rotations += 180;
        reversed = true;
      }
      else if(rotations > 90) {
        rotations -= 180;
        reversed = true;
      }
      else {
        reversed = false;
      }
      
      accel = (Math.sqrt(Math.pow(turnField, 2)+Math.pow(forwardField,2)));

      if(reversed) {
        accel *= -1;
      }

      rotations = (360 - rotations) / encoderConversionFactor;
    }
    
    //target angle calculations (trig to find joystick angle)
    if((forward > 0.1 || forward < -0.1) || (turn > 0.1 || turn < -0.1)) {
      if(turn >= 0) {
        if(forward >= 0) {
          //System.out.print(4 + " ");
          rotations = 270+Math.atan(forward/turn)*180/Math.PI;
        }
        if(forward < 0) {
          //System.out.print(3 + " ");
          rotations = 270+Math.atan(forward/turn)*180/Math.PI;
        }
      }
      if(turn < 0) {
        if(forward <= 0) {
          //System.out.print(2 + " ");
          rotations = 90+Math.atan(forward/turn)*180/Math.PI;
        }
        if(forward > 0) {
          //System.out.print(1 + " ");
          rotations = -Math.atan(turn/forward)*180/Math.PI;
        }
      }
      if((rotations-getHeading(0)) < -180) {
        rotations += 360;
        reversed = true;
      }
      else if((rotations-getHeading(0)) > 180) {
        rotations -= 360;
        reversed = true;
      }
      else {
        reversed = false;
      }
      
      accel = (Math.sqrt(Math.pow(turn, 2)+Math.pow(forward,2)));

      if(!reversed) {
        accel *= -1;
      }

      passed360 = Math.toIntExact(Math.round(getHeading(0)-(getHeading(0)%360))/360);

      rotations += 360*passed360;
      rotations = (360 - rotations) / encoderConversionFactor;
    }
  }
}
