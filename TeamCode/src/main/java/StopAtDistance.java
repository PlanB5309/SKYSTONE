import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class StopAtDistance {

    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public StopAtDistance (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    //
// Cozy up to the blocks in the quarry
    public void forward(double speed, int targetDistance, int maxDistance) throws InterruptedException{
        if (!linearOpMode.opModeIsActive())
            return;

        double adjustedSpeed;

        int maxClickDistance = maxDistance * robot.CLICKS_PER_INCH;
        double mainDirection = robot.getHeading();
        double currentDirection = mainDirection;
        robot.runUsingEncoder();

        double sensorDistance = robot.frontDistanceSensor.getDistance(DistanceUnit.CM);  // Distance in CM from color sensor
        int currentDistance = (int) Math.round(sensorDistance);    // integer distance rounded to the nearest cm

        // Keep moving until the max distance is reached or the target distance is achieved
        while ( linearOpMode.opModeIsActive() &&
                maxClickDistance > robot.rightRearDrive.getCurrentPosition() &&
                currentDistance != targetDistance) {
            adjustedSpeed = adjustSpeedForward(currentDistance, targetDistance, speed);
            currentDirection = robot.getHeading();

            sensorDistance = robot.frontDistanceSensor.getDistance(DistanceUnit.CM);
            currentDistance = (int) Math.round(sensorDistance);

            //Adjust for drift
            if (currentDirection < mainDirection) {
                robot.leftFrontDrive.setPower(adjustedSpeed - 0.02);
                robot.leftRearDrive.setPower(adjustedSpeed - 0.02);
                robot.rightFrontDrive.setPower(adjustedSpeed + 0.02);
                robot.rightRearDrive.setPower(adjustedSpeed + 0.02);
            } else if (currentDirection > mainDirection) {
                robot.leftFrontDrive.setPower(adjustedSpeed + 0.02);
                robot.leftRearDrive.setPower(adjustedSpeed + 0.02);
                robot.rightFrontDrive.setPower(adjustedSpeed - 0.02);
                robot.rightRearDrive.setPower(adjustedSpeed - 0.02);
            } else {
                robot.leftFrontDrive.setPower(adjustedSpeed);
                robot.leftRearDrive.setPower(adjustedSpeed);
                robot.rightFrontDrive.setPower(adjustedSpeed);
                robot.rightRearDrive.setPower(adjustedSpeed);
            }
        }

        robot.stop ();
    }

    public void left(double speed, int targetDistance, int maxDistance) throws InterruptedException{
        if (!linearOpMode.opModeIsActive())
            return;

        double adjustedSpeed;

        int maxClickDistance = maxDistance * robot.CLICKS_PER_INCH;

        double mainDirection = robot.getHeading();
        double currentDirection = mainDirection;
        robot.runUsingEncoder();

        double sensorDistance = robot.leftDistanceSensor.getDistance(DistanceUnit.CM);;  // Distance in CM from color sensor
        int currentDistance = (int) Math.round(sensorDistance);    // integer distance rounded to the nearest cm


        // Keep moving until the max distance is reached or the target distance is achieved
        while ( linearOpMode.opModeIsActive() &&
                maxClickDistance > Math.abs (robot.rightRearDrive.getCurrentPosition()) &&
                currentDistance != targetDistance) {
            adjustedSpeed = adjustSpeed(currentDistance, targetDistance, speed);
            currentDirection = robot.getHeading();

            sensorDistance = robot.leftDistanceSensor.getDistance(DistanceUnit.CM);
            currentDistance = (int) Math.round(sensorDistance);

            robot.leftFrontDrive.setPower(-adjustedSpeed);
            robot.leftRearDrive.setPower(adjustedSpeed);
            robot.rightFrontDrive.setPower(adjustedSpeed);
            robot.rightRearDrive.setPower(-adjustedSpeed);


            telemetry.addData("sensor: ", sensorDistance);
            telemetry.addData("clicks: ", robot.rightRearDrive.getCurrentPosition());
            telemetry.update();
        }

        robot.stop ();
    }

    public void right(double speed, int targetDistance, int maxDistance) throws InterruptedException{
        if (!linearOpMode.opModeIsActive())
            return;

        double adjustedSpeed;

        int maxClickDistance = maxDistance * robot.CLICKS_PER_INCH;

        double mainDirection = robot.getHeading();
        double currentDirection = mainDirection;
        robot.runUsingEncoder();

        double sensorDistance = robot.rightDistanceSensor.getDistance(DistanceUnit.CM);;  // Distance in CM from color sensor
        int currentDistance = (int) Math.round(sensorDistance);    // integer distance rounded to the nearest cm


        // Keep moving until the max distance is reached or the target distance is achieved
        while ( linearOpMode.opModeIsActive() &&
                maxClickDistance > Math.abs (robot.rightRearDrive.getCurrentPosition()) &&
                currentDistance != targetDistance) {
            adjustedSpeed = adjustSpeed(currentDistance, targetDistance, speed);
            currentDirection = robot.getHeading();

            sensorDistance = robot.rightDistanceSensor.getDistance(DistanceUnit.CM);
            currentDistance = (int) Math.round(sensorDistance);

            robot.leftFrontDrive.setPower(adjustedSpeed);
            robot.leftRearDrive.setPower(-adjustedSpeed);
            robot.rightFrontDrive.setPower(-adjustedSpeed);
            robot.rightRearDrive.setPower(adjustedSpeed);

        }

        robot.stop ();
        telemetry.addData("Right Distance Sensor", robot.rightDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
    public void backward(double speed, int targetDistance, int maxDistance) throws InterruptedException{
        if (!linearOpMode.opModeIsActive())
            return;

        double adjustedSpeed;

        int maxClickDistance = maxDistance * robot.CLICKS_PER_INCH;

        double mainDirection = robot.getHeading();
        double currentDirection = mainDirection;
        robot.runUsingEncoder();

        double sensorDistance = robot.rearDistanceSensor.getDistance(DistanceUnit.CM);;  // Distance in CM from color sensor
        int currentDistance = (int) Math.round(sensorDistance);    // integer distance rounded to the nearest cm


        // Keep moving until the max distance is reached or the target distance is achieved
        while ( linearOpMode.opModeIsActive() &&
                maxClickDistance > Math.abs (robot.rightRearDrive.getCurrentPosition()) &&
                currentDistance != targetDistance) {
            adjustedSpeed = adjustSpeed(currentDistance, targetDistance, speed);
            currentDirection = robot.getHeading();

            sensorDistance = robot.rearDistanceSensor.getDistance(DistanceUnit.CM);
            currentDistance = (int) Math.round(sensorDistance);

            robot.leftFrontDrive.setPower(-adjustedSpeed);
            robot.leftRearDrive.setPower(-adjustedSpeed);
            robot.rightFrontDrive.setPower(-adjustedSpeed);
            robot.rightRearDrive.setPower(-adjustedSpeed);

        }

        robot.stop ();
        telemetry.addData("Right Distance Sensor", robot.rearDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    /*
     *  Adjust speed depending on how close we are to the target distance
     */
    private double adjustSpeed (int currentDistance, int targetDistance, double defaultSpeed) {
        double newSpeed;

        // Adjust speed for distance
        if (7 < Math.abs(currentDistance - targetDistance))
            newSpeed = defaultSpeed;
        else
            newSpeed = .07;

        // Adjust direction for distance
        if (currentDistance < targetDistance)
            newSpeed = -newSpeed;
        return newSpeed;
    }

    //Adjust speed when going forward based on the less sensitive distance sensor on the front
    private double adjustSpeedForward (int currentDistance, int targetDistance, double defaultSpeed) {
        double newSpeed;

        // Adjust speed for distance
        if (currentDistance >= 7)
            newSpeed = defaultSpeed;
        else
            newSpeed = .07;

        // Adjust direction for distance
        if (currentDistance < targetDistance)
            newSpeed = -newSpeed;
        return newSpeed;
    }


}
