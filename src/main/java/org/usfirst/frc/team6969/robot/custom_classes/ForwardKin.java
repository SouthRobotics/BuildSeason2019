/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.custom_classes;

/**
 * z is rotation
 * x is how far out
 * y is height
 */
/*lengths of joints (inches):
bottom: 32.5
middle: 32.25
top: 11
*/
public class ForwardKin {
    public static double[] getXY(double ang1, double ang2, double ang3, double[] lengths)
    {
        double x = getX(ang1,ang2,ang3,lengths);
        double y = getY(ang1,ang2,ang3,lengths);

        double[] xy = {x,y};
        return xy;
    }

    public static double getX(double ang1, double ang2, double ang3, double[] lengths)
    {
        double a = Math.abs(ang1 + ang2 - 180);
        double b = Math.abs(ang1 + ang2 + ang3 - 360);

        double realAng1 = 0;
        if (ang1>90)//if l1 is pointed backwards
            realAng1 = 180-ang1;
        else
            realAng1 = ang1;

        double x1 = lengths[0]*Math.cos(realAng1);
        double x2 = lengths[1]*Math.cos(a);
        double x3 = lengths[2]*Math.cos(b);

        double len = 0;

        if (ang1>90)//if l1 is pointed backwards
            len-=x1;
        else
            len+=x1;
        len+=x2;
        len+=x3;

        return len;
    }
    
    public static double getY(double ang1, double ang2, double ang3, double[] lengths)
    {
        double a = Math.abs(ang1 + ang2 - 180);
        double b = Math.abs(ang1 + ang2 + ang3 - 360);

        double realAng1 = 0;
        if (ang1>90)//if l1 is pointed backwards
            realAng1 = 180-ang1;
        else
            realAng1 = ang1;

        double y1 = lengths[0]*Math.sin(realAng1);
        double y2 = lengths[1]*Math.sin(a);
        double y3 = lengths[2]*Math.sin(b);

        double len = y1;

        if (ang1>90)//if l1 is tilted backward
        {
            if (90-realAng1+ang2 > 90)//if l2 is tilted up
            {
                len+=y2;
                if (a + ang3 > 180)//if l3 is tilted up
                    len+=y3;
                else//if l3 is tilted down
                    len-=y3;
            }
            else//if l2 is tilted down
            {
                len-=y2;
                if (a + 180 < ang3)//if l3 is tilted up
                    len+=y3;
                else//if l3 is tilted down
                    len-=y3;
            }
        }
        else//if l1 is tilted forward
        {
            if (ang1 + ang2 > 180)//if l2 is tilted up
            {
                len+=y2;
                if (a + ang3 > 180)//if l3 is tilted up
                    len+=y3;
                else//if l3 is tilted down
                    len-=y3;
            }
            else//if l2 is tilted down
            {
                len-=y2;
                if (a + 180 < ang3)//if l3 is tilted up
                    len+=y3;
                else//if l3 is tilted down
                    len-=y3;
            }
        }

        return len;
    }
}
