// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

/** 
 * Creates a gyrowrapper class with simulation support
 */
public class GyroWrapper {
    private AHRS gyro = new AHRS();
    private double m_simAngle;

    /**
     * @return Gyro angle
     */
    public double getAngle(){
        return Robot.isReal() ? gyro.getAngle() : 
    }

    public void resetGyro(){

    }
}
