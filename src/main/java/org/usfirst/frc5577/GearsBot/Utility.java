/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc5577.GearsBot;

/**
 * Add your docs here.
 */
public class Utility {
  public static double clamp(double value, double low, double high) {
    return Math.max(low, Math.min(value, high));
  }
}
