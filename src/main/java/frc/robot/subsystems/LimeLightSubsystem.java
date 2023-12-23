// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLightSubsystem extends SubsystemBase {
  /** Creates a new LimeLightSubsystem. */
  private final NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry tx = limeLight.getEntry("tx");
  private final NetworkTableEntry ty = limeLight.getEntry("ty");
  private final NetworkTableEntry ta = limeLight.getEntry("ta"); 
  private final NetworkTableEntry _botPose = limeLight.getEntry("botpose");
  private final NetworkTableEntry _cameraPose = limeLight.getEntry("targetpose_cameraspace");
  private final PIDController yMovePID = new PIDController(0.018, 0, 0.001);
  private final PIDController xMovePID = new PIDController(0, 0, 0);
  private double txValue;
  private double tyValue;
  private double taValue;
  private double distance;
  private boolean turnRightState;
  private double yMovePIDOutput, xMovePIDOutput;
  private final double maxXMovepPIDOutput = 0.3; 
  private final double maxYMovePIDOutput = 0.3;

  public LimeLightSubsystem() {}

  public double xMove(){
    if(taValue != 0){
      return xMovePIDOutput;
    }
    else{
      return 0;
    }
  }

  public double yMove(){
    if(taValue != 0){
      return yMovePIDOutput;
    }
    else{
      return 0;
    }
  }

  public double turn(){
    if(turnRightState == true && taValue == 0){
      return -0.5;
    }
    else if(turnRightState == false && taValue == 0){
      return 0.5;
    }
    else{
      return 0;
    }

  }

  @Override
  public void periodic() {
    if(txValue > 10){
      turnRightState = true;
    }
    else if(txValue < -10){
      turnRightState = false;
    }
    double[] botPose = _botPose.getDoubleArray(new double[6]);
    double[] cameraPose = _cameraPose.getDoubleArray(new double[6]);

    if(cameraPose.length != 0){
      SmartDashboard.putNumber("x taget pose", cameraPose[3]);
      SmartDashboard.putNumber("y taget pose", cameraPose[4]);
      SmartDashboard.putNumber("z taget pose", cameraPose[5]);
    }
    if(botPose.length != 0){
      SmartDashboard.putNumber("x bot pose", botPose[0]);
      SmartDashboard.putNumber("y bot pose", botPose[1]);
      SmartDashboard.putNumber("z bot pose", botPose[2]);
    }
    txValue = tx.getDouble(0.0);
    tyValue = ty.getDouble(0.0);
    taValue = ta.getDouble(0.0);
    distance = (51-59)/Math.tan(tyValue);
    xMovePIDOutput = xMovePID.calculate(distance, 50);
    yMovePIDOutput = yMovePID.calculate(txValue, 0);
    xMovePIDOutput = Constants.setMaxOutput(xMovePIDOutput, maxXMovepPIDOutput);
    yMovePIDOutput = Constants.setMaxOutput(yMovePIDOutput, maxYMovePIDOutput);
    SmartDashboard.putNumber("tx", txValue);
    SmartDashboard.putNumber("ty", tyValue);
    SmartDashboard.putNumber("ta", taValue);
    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putBoolean("turnRight", turnRightState);
    // This method will be called once per scheduler run
  }
}