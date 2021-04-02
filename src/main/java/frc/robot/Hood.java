package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public class Hood {
    Robot robot;

    private final I2C.Port lidarPort = I2C.Port.kMXP;
    private final I2C lidarSenseI2c = new I2C(lidarPort, 0x63);
    private byte[] bytes;

    public Hood(Robot robot){
        this.robot = robot;
    }


    public void goToAngle(double angle){
        //adjust shooter angle
        double hoodOffset = 1;
		double hoodEncoderSetpoint = angle * 7;
		double hoodEncoderPosition = robot.motor.shooterHood.getEncoder().getPosition();
        double distFromSetpoint = hoodEncoderPosition - hoodEncoderSetpoint;

		// System.out.println("Hood setpoint: " + hoodEncoderSetpoint);
        robot.tables.hoodSetpoint.setDouble(hoodEncoderSetpoint);
		// if(hoodEncoderPosition != hoodEncoderSetpoint + hoodOffset ||
		// 	hoodEncoderPosition != hoodEncoderSetpoint - hoodOffset){
		// 	if(hoodEncoderPosition > hoodEncoderSetpoint + hoodOffset){
		// 		System.out.println("Made it to moving hood part");
		// 		robot.motor.shooterHood.set(-0.08);
		// 	}else if(hoodEncoderPosition < hoodEncoderSetpoint - hoodOffset){
		// 		System.out.println("Made it to moving hood part - ");
		// 		robot.motor.shooterHood.set(0.08);
		// 	}
		// }else{
		// 	robot.motor.shooterHood.set(0);
		// }

        if(distFromSetpoint - hoodOffset > 0){
            robot.motor.shooterHood.set(-0.03);
        }else if(distFromSetpoint + hoodOffset < 0){
            robot.motor.shooterHood.set(0.07);
        } else {
            robot.motor.shooterHood.set(0);
        }
		// System.out.println("Hood encoder counts: " + hoodEncoderPosition);
        robot.tables.hoodEncoderCounts.setDouble(hoodEncoderPosition);
    }

    public double getHoodAngle(){
		double hoodAngle = Math.asin((9.81 * robot.tables.shooterDistance.getDouble(0)) / (230 * 230)); hoodAngle = Math.toDegrees(hoodAngle / 2);
		System.out.println("Shooter Distance: " + robot.tables.shooterDistance.getDouble(0));
		System.out.println("Hood angle: " + hoodAngle);
		return hoodAngle;
	}

    //the function to get the angle from our limelight and convert it to a distance (in [unit to put in later])
	public double limeLightDegreesToDistance(double LMAngle){
		double angleToTarget = 19.84 + LMAngle;
		double radianToTarget = Math.toRadians(angleToTarget);
		double distanceToTarget = (54.25/(Math.tan(radianToTarget)));
		return distanceToTarget;
	}

    public void getLidarDistance(){
        lidarSenseI2c.read(0x04, 8, bytes);
        System.out.println(bytes);
    }
}