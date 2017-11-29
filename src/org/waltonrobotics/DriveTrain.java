package org.waltonrobotics;

public class DriveTrain {
	public static void setSpeeds(double leftVelocity, double rightVelocity) {
		System.out.printf("leftVelocity: %01.03f \t rightVelocity: %01.03f", leftVelocity, rightVelocity);
	}
	
	public static void main(String[] args)
	{
		setSpeeds(1.4456345,  1.32340);
	}
}
