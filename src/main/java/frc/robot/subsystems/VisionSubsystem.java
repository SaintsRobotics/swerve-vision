// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.saints.Measurement;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

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
  private Supplier<Pose2d> m_PoseSupplier;
  private AHRS m_gyro;

  /** Creates a new Limelight. */
  public VisionSubsystem(AHRS gyro) {
    m_gyro = gyro;
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", VisionConstants.validIDs);
  }

  @Override
  public void periodic() {
    boolean doRejectUpdate = false;

    if (!Robot.isReal()){
      return;
    }

    //Mtg1
    if (m_gyro.getRate() > VisionConstants.maxRotSpeedMgt2)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        m_VisionConsumer.accept(new Measurement(mt1.timestampSeconds, mt1.pose, VisionConstants.kVisionSTDDevs));
      }
    }
    else{
      //AHRS has positive-clockwise coords, robot coordinate is negative-clockwise coords
      LimelightHelpers.SetRobotOrientation("limelight", m_PoseSupplier.get().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_VisionConsumer.accept(new Measurement(mt2.timestampSeconds, mt2.pose, VisionConstants.kVisionSTDDevs));
      }
    }
  }

  public void acceptMeasurementConsumer(Consumer<Measurement> consume){
    m_VisionConsumer = consume;
  }

  public void acceptPoseSupplier(Supplier<Pose2d> supplier){
    m_PoseSupplier = supplier;
  }
}
