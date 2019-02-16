/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.custom_classes;

/**
 * Add your docs here.
 */
public class ForwardKin {
    public static double getX(double ang1, double ang2, double ang3, double[] lengths)
    {
        double a = Math.abs(ang1 + ang2 - 180);
        double b = Math.abs(ang1 + ang2 + ang3 - 360);

        double x1 = lengths[0]*Math.cos(ang1);
        double x2 = lengths[1]*Math.cos(a);
        double x3 = lengths[2]*Math.cos(b);

        return (x1+x2+x3);
    }
    
    public static double getY(double ang1, double ang2, double ang3, double[] lengths)
    {
        double a = Math.abs(ang1 + ang2 - 180);
        double b = Math.abs(ang1 + ang2 + ang3 - 360);

        double y1 = lengths[0]*Math.sin(ang1);
        double y2 = lengths[1]*Math.sin(a);
        double y3 = lengths[2]*Math.sin(b);

        return (y1+y2+y3);
    }
}
