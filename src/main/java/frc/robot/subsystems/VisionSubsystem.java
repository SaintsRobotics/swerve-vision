// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Consumer;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

/**
 * <p>
 * In 3D poses and transforms:
 * <ul>
 * <li>+X is north/forward,
 * <li>+Y is west/left,
 * <li>+Z is up.
 * </ul>
 * 
 * <p>
 * On the field (0, 0, 0) in 3D space is the right corner of the blue alliance
 * driver station
 * Therefore, from the blue driver station: +X is forward, +Y is left, +Z is up
 * 
 * <p>
 * In 2D poses and transforms:
 * <ul>
 * <li>+X is away from the driver,
 * <li>+Y is toward the blue alliance driver's left and to the red alliance
 * driver's right
 * <li>+Rotation is clockwise
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {
  private Consumer<Measurement> m_VisionConsumer;
  private AHRS m_gyro;

  /** Creates a new Limelight. */
  public VisionSubsystem(AHRS gyro) {
    m_gyro = gyro;
  }

  @Override
  public void periodic() {
    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation("limelight", m_gyro.getAngle(), m_gyro.getRate(), VisionConstants.kLimelightCamPose[4], 0, VisionConstants.kLimelightCamPose[3], 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    {
      doRejectUpdate = true;
    }
    if(mt2.tagCount == 0)
    {
      doRejectUpdate = true;
    }
    if(!doRejectUpdate)
    {
      m_VisionConsumer.accept(new Measurement(mt2.timestampSeconds, mt2.pose, VecBuilder.fill(.7,.7,9999999)));
    }
  }

  public static class Measurement {
    public double timestamp;
    public Pose2d pose;
    public Matrix<N3, N1> stdDeviation;

    public Measurement(double timestamp, Pose2d pose, Matrix<N3, N1> stdDeviation) {
      this.timestamp = timestamp;
      this.pose = pose;
      this.stdDeviation = stdDeviation;
    }

    public Pose2d getPose2d(){
      return pose;
    }

    public double getTimestamp(){
      return timestamp;
    }

    public Matrix<N3, N1> getDeviation(){
      return stdDeviation;
    }
  }

  public void acceptConsumer(Consumer<Measurement> consume){
    m_VisionConsumer = consume;
  }
}
