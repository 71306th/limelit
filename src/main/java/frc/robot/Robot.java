// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.security.auth.login.FailedLoginException;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.ChenryLib.PID;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  TalonFX hori = new TalonFX(3);
  TalonFX verti = new TalonFX(4);

  CANSparkMax right = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax left = new CANSparkMax(2, MotorType.kBrushless);

  

  double theta = 76;
  double tanTheta;
  double cos2Theta;
  double cg = 9.84; //  m/ss
  double y=0; //  vertical
  double x; //  horizontal

  
  static final double kpdefault = 0.022;
  static final double kidefault = 0.5;
  static final double kddefault = 0.00000001;
  static final double limitdefault = 0.0009;
  static final double windupdefault = 0.00005; //current best
  static final double spiddefault = 0.3;
  static final double omdefault = 0.17;
  static final double offsetdefault = -0.15;

  static final String pkey = "kp";
  static final String ikey = "ki";
  static final String dkey = "kd";
  static final String limkey = "limit";
  static final String windkey = "wind up";
  static final String spidkey = "spid";
  static final String omkey = "om";
  static final String offsetkey = "offset";

  double kp = kpdefault;
  double ki = kidefault;
  double kd = kddefault;
  double limit = limitdefault;
  double windup = windupdefault;
  double spid = spiddefault; 
  double rpm;
  double vel; // average of both flywheels (rad/s) r1=0.0254 r2=0.0508 11.525 23.05 =17.2865
  double time;
  double offsetmultiplier = omdefault;
  double offset = offsetdefault;
  int count = 0;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry tpcs = table.getEntry("targetpose_cameraspace"); 

  

  
  XboxController controller = new XboxController(0);

  @Override
  public void teleopInit() {
    
    left.follow(right, true);
  }

  public void loadPreferances(){

  }

  

  @Override
  public void teleopPeriodic() {
    // // PIDController pid = new PIDController(0.015, 0.035, 0.00014); //new high score (0.015, 0.03, 0.0002)
    // // PID pidpro = new PID(0.015, 0.035, 0.00014, 0, 0); //uwu testing pending
    PID pidpro = new PID(kp, ki, kd, windup, limit);

    Preferences.initDouble(pkey, kp);
    Preferences.initDouble(ikey, ki);
    Preferences.initDouble(dkey, kd);
    Preferences.initDouble(limkey, limit);
    Preferences.initDouble(windkey, windup);
    Preferences.initDouble(dkey, kDefaultPeriod);
    Preferences.initDouble(spidkey, spiddefault);
    Preferences.initDouble(omkey, offsetmultiplier);
    Preferences.initDouble(offsetkey, offset);

    kp = Preferences.getDouble(pkey, kp);
    ki = Preferences.getDouble(ikey, ki);
    kd = Preferences.getDouble(dkey, kd);
    limit = Preferences.getDouble(limkey, limit);
    windup = Preferences.getDouble(windkey, windup);
    spid = Preferences.getDouble(spidkey, spid);
    offsetmultiplier = Preferences.getDouble(omkey, offsetmultiplier);
    offset = Preferences.getDouble(offsetkey, offset);

    tanTheta = Math.tan(theta);
    cos2Theta = Math.pow(Math.cos(theta), 2);
    // x = tpcs.getDoubleArray(new Double[6])[3];
    
    
   

    // pro775.set(TalonSRXControlMode.PercentOutput, -pid.calculate(tx.getDouble(0.0)));
    hori.set( -pidpro.calculate(tx.getDouble(0.0)));
    SmartDashboard.putNumber("x", tx.getDouble(0.0));
    SmartDashboard.putNumberArray("tp_cs", table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]));
    
    
    SmartDashboard.putNumber("tpcs_LRF", table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[2]);
    SmartDashboard.putNumber("pp", 0.015);
    vel = Math.sqrt(table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[2]*cg/Math.sin(theta*2));
    SmartDashboard.putNumber("raw speed", vel);
    SmartDashboard.putNumber("speed", vel*offsetmultiplier+offset);
    // SmartDashboard.putNumber("time", time);
    
    if(controller.getLeftBumperPressed()){
      if(count>1){count=0;}else{count+=1;} // gay pride
    }
    if(count==1){
      hori.set( controller.getLeftX()*0.5);
      // pro775.set(TalonSRXControlMode.PercentOutput, 0.1);// automove
      // pro775.set(TalonSRXControlMode.PercentOutput, output);
      
    }

    if (controller.getRightTriggerAxis()>0.2) {
      right.set(vel*offsetmultiplier+offset);
      
    }

    if (controller.getXButton()) {
      right.set(0);
    }
    


    
  }

  
  
  

}
