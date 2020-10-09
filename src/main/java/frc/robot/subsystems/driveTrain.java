/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import java.awt.event.KeyEvent;
import java.util.EventListener;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
//import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.RobotMap;
import frc.robot.OI;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;


/**
 * Add your docs here.
 */
public class driveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private double time = 0;
  private PWMVictorSPX left = new PWMVictorSPX(0);
  private PWMVictorSPX right = new PWMVictorSPX(1);

  public static driveTrain drive;
  public driveTrain() {
    time = 0;
    left.setInverted(true);
  }
  public static driveTrain getDrive() {
    if (drive == null) {
      drive = new driveTrain();
    }
    return drive;
  }
  public void tankDrive(double leftPow, double rightPow) {
    left.set(leftPow);
    right.set(rightPow);
  }
  public void periodic() {
    if(time<1000/20) {
      tankDrive(1,1);
    } else if(time<2000/20) {
      tankDrive(-1,1);
    } else if(time<3000/20) {
      tankDrive(-1,-1);
    } else {
      tankDrive(0,0);
    }
    time++;
  }
  public void keyPressed(KeyEvent e) {
    int key = e.getKeyCode();
    if(key == KeyEvent.VK_LEFT) {
      tankDrive(-1, 1);
    }
    if(key == KeyEvent.VK_RIGHT) {
      tankDrive(1, -1);
    }
    if(key == KeyEvent.VK_UP) {
      tankDrive(1, 1);
    }
    if(key == KeyEvent.VK_DOWN) {
      tankDrive(-1, -1);
    }
  }
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }
}
