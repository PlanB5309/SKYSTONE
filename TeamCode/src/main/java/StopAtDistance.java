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


    public void left(double speed, int targetDistance) throws InterruptedException{
        if (!linearOpMode.opModeIsActive())
            return;
        robot.runUsingEncoder();
        int targetClickDistance = targetDistance * robot.CLICKS_PER_INCH;

        double currentSpeed = speed;
        robot.leftFrontDrive.setPower(-currentSpeed);
        robot.leftRearDrive.setPower(currentSpeed);
        robot.rightFrontDrive.setPower(currentSpeed);
        robot.rightRearDrive.setPower(-currentSpeed);

        double distanceValue = robot.leftDistanceSensor.getDistance(DistanceUnit.INCH);
        double mainDirection = robot.getHeading();
        double currentDirection = mainDirection;

        while ( robot.leftRearDrive.getCurrentPosition() < targetClickDistance &&
                linearOpMode.opModeIsActive() &&
                Math.abs(distanceValue - targetDistance) > robot.DISTANCE_THRESHOLD) {

            if (distanceValue < robot.SLOW_DISTANCE)
                currentSpeed = speed/2;
            else if (distanceValue < targetDistance)
                currentSpeed = -speed;
            else //if (distanceValue > targetDistance)
                currentSpeed = speed;

            if (currentDirection < mainDirection) {
                robot.leftFrontDrive.setPower(-currentSpeed - 0.02);
                robot.leftRearDrive.setPower(currentSpeed + 0.02);
                robot.rightFrontDrive.setPower(currentSpeed - 0.02);
                robot.rightRearDrive.setPower(-currentSpeed + 0.02);
            } else if (currentDirection > mainDirection) {
                robot.leftFrontDrive.setPower(-currentSpeed + 0.02);
                robot.leftRearDrive.setPower(currentSpeed - 0.02);
                robot.rightFrontDrive.setPower(currentSpeed + 0.02);
                robot.rightRearDrive.setPower(-currentSpeed - 0.02);
            } else {
                robot.leftFrontDrive.setPower(-currentSpeed);
                robot.leftRearDrive.setPower(currentSpeed);
                robot.rightFrontDrive.setPower(currentSpeed);
                robot.rightRearDrive.setPower(-currentSpeed);
            }
            currentDirection = robot.getHeading();
            distanceValue = robot.leftDistanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("current: ", currentDirection);
            telemetry.addData("main direction: ", mainDirection);
            telemetry.addData("speed: ", currentSpeed);
            telemetry.update();
        }

        robot.stop ();
    }


    public void right(double speed, int targetDistance, int maxDistance) throws InterruptedException{
        if (!linearOpMode.opModeIsActive())
            return;
        robot.runUsingEncoder();
        int maxClickDistance = maxDistance * robot.CLICKS_PER_INCH;

        double currentSpeed = speed;
        robot.leftFrontDrive.setPower(currentSpeed);
        robot.leftRearDrive.setPower(-currentSpeed);
        robot.rightFrontDrive.setPower(-currentSpeed);
        robot.rightRearDrive.setPower(currentSpeed);

        double distanceValue = robot.rightDistanceSensor.getDistance(DistanceUnit.INCH);
        double mainDirection = robot.getHeading();
        double currentDirection = mainDirection;

        while (robot.leftRearDrive.getCurrentPosition() < maxClickDistance &&
                linearOpMode.opModeIsActive() &&
                Math.abs(distanceValue - targetDistance) > robot.DISTANCE_THRESHOLD) {


            if (distanceValue < robot.SLOW_DISTANCE)
                currentSpeed = speed/2;
            else if (distanceValue < targetDistance)
                currentSpeed = -speed;
            else //if (distanceValue > targetDistance)
                currentSpeed = speed;

            if (currentDirection < mainDirection) {
                robot.leftFrontDrive.setPower(currentSpeed - 0.02);
                robot.leftRearDrive.setPower(-currentSpeed + 0.02);
                robot.rightFrontDrive.setPower(-currentSpeed - 0.02);
                robot.rightRearDrive.setPower(currentSpeed + 0.02);
            } else if (currentDirection > mainDirection) {
                robot.leftFrontDrive.setPower(currentSpeed + 0.02);
                robot.leftRearDrive.setPower(-currentSpeed - 0.02);
                robot.rightFrontDrive.setPower(-currentSpeed + 0.02);
                robot.rightRearDrive.setPower(currentSpeed - 0.02);
            } else {
                robot.leftFrontDrive.setPower(currentSpeed);
                robot.leftRearDrive.setPower(-currentSpeed);
                robot.rightFrontDrive.setPower(-currentSpeed);
                robot.rightRearDrive.setPower(currentSpeed);
            }
            currentDirection = robot.getHeading();
            distanceValue = robot.rightDistanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("current: ", currentDirection);
            telemetry.addData("main direction: ", mainDirection);
            telemetry.addData("speed: ", currentSpeed);
            telemetry.update();
        }

        robot.stop ();
    }


    public void forward(double speed, int targetDistance, int maxDistance) throws InterruptedException{
        if (!linearOpMode.opModeIsActive())
            return;
        robot.runUsingEncoder();
        int maxClickDistance = maxDistance * robot.CLICKS_PER_INCH;

        double currentSpeed = speed;
        robot.leftFrontDrive.setPower(currentSpeed);
        robot.leftRearDrive.setPower(currentSpeed);
        robot.rightFrontDrive.setPower(currentSpeed);
        robot.rightRearDrive.setPower(currentSpeed);

        double distanceValue = robot.frontDistanceSensor.getDistance(DistanceUnit.CM);
        double mainDirection = robot.getHeading();
        double currentDirection = mainDirection;

        while (robot.leftRearDrive.getCurrentPosition() < maxClickDistance &&
                linearOpMode.opModeIsActive() &&
                Math.abs(distanceValue - targetDistance) > robot.DISTANCE_THRESHOLD) {

            if (distanceValue < robot.SLOW_DISTANCE)
                currentSpeed = speed/2;
            else if (distanceValue < targetDistance)
                currentSpeed = -speed;
            else //if (distanceValue > targetDistance)
                currentSpeed = speed;

            if (currentDirection < mainDirection) {
                robot.leftFrontDrive.setPower(currentSpeed - 0.02);
                robot.leftRearDrive.setPower(currentSpeed + 0.02);
                robot.rightFrontDrive.setPower(currentSpeed - 0.02);
                robot.rightRearDrive.setPower(currentSpeed + 0.02);
            } else if (currentDirection > mainDirection) {
                robot.leftFrontDrive.setPower(currentSpeed + 0.02);
                robot.leftRearDrive.setPower(currentSpeed - 0.02);
                robot.rightFrontDrive.setPower(currentSpeed + 0.02);
                robot.rightRearDrive.setPower(currentSpeed - 0.02);
            } else {
                robot.leftFrontDrive.setPower(currentSpeed);
                robot.leftRearDrive.setPower(currentSpeed);
                robot.rightFrontDrive.setPower(currentSpeed);
                robot.rightRearDrive.setPower(currentSpeed);
            }
            currentDirection = robot.getHeading();
            distanceValue = robot.frontDistanceSensor.getDistance(DistanceUnit.INCH);

            telemetry.addData("encoder: ", robot.leftFrontDrive.getCurrentPosition());
            telemetry.addData("max clicks: ", maxClickDistance);
            telemetry.addData("sensor value: ", distanceValue);
            telemetry.update();
        }

        robot.stop ();
        Thread.sleep(3000);
    }
}
