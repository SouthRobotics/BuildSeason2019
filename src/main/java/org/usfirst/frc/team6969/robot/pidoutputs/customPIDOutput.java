/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6969.robot.pidoutputs;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Add your docs here.
 */

public class customPIDOutput implements PIDOutput {
    public double outVal;

    public customPIDOutput()
    {
    }

	@Override
	public void pidWrite(double output) {
		outVal = output;
	}
}
