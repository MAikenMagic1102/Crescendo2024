// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class ScoringTarget {
    public enum Position{
        AMP,
        SUBWOOFER,
        RANGED,
      };
    
    private static Position currentTarget = Position.SUBWOOFER;

    public static Position getTarget(){
        return currentTarget;
    }

    public static void setTarget(Position targetPosition){
        currentTarget = targetPosition;
    }

}
