import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class FindSkyStone {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    Drive drive;

    public FindSkyStone (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
        this.drive = new Drive(robot, telemetry, linearOpMode);
    }

    public int left(double speed, int distance) throws InterruptedException{
        robot.setupDriveTrain();

        robot.leftFrontDrive.setTargetPosition(distance*robot.CLICKS_PER_INCH);
        robot.leftRearDrive.setTargetPosition(distance*robot.CLICKS_PER_INCH);
        robot.rightFrontDrive.setTargetPosition(distance*robot.CLICKS_PER_INCH);
        robot.rightRearDrive.setTargetPosition(distance*robot.CLICKS_PER_INCH);

        int target = distance * robot.CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(target);
        robot.leftRearDrive.setTargetPosition(target);
        robot.rightFrontDrive.setTargetPosition(target);
        robot.rightRearDrive.setTargetPosition(target);

        robot.leftFrontDrive.setPower(speed);
        robot.leftRearDrive.setPower(-speed);
        robot.rightFrontDrive.setPower(-speed);
        robot.rightRearDrive.setPower(speed);

        int red = robot.frontColorSensor.red();

        telemetry.addData("linear opmode is working, target = ", target);
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());

        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() &&
                (red > threshold(robot.frontDistanceSensor.getDistance(DistanceUnit.CM)))) {

            red = robot.frontColorSensor.red();
            Thread.yield();
            telemetry.addData("Distance (cm) ",
                    String.format(Locale.US, "%.02f", robot.rightDistanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Encoder Clicks ", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("color value ", red);
            telemetry.update();
        }

        int blockNumber;
        int tolerance = 4;

        if (robot.leftDistanceSensor.getDistance(DistanceUnit.CM) < robot.blockDistance[2] + tolerance &&
                robot.leftDistanceSensor.getDistance(DistanceUnit.CM) > robot.blockDistance[2] - tolerance)
            blockNumber = 5;
        else if (robot.leftDistanceSensor.getDistance(DistanceUnit.CM) < robot.blockDistance[3] + tolerance &&
                robot.leftDistanceSensor.getDistance(DistanceUnit.CM) > robot.blockDistance[3] - tolerance)
            blockNumber = 6;
        else
            blockNumber = 4;

        robot.stop();

        return blockNumber;
    }
    public int right(double speed, int distance) throws InterruptedException{
        robot.setupDriveTrain();

        robot.leftFrontDrive.setTargetPosition(distance*robot.CLICKS_PER_INCH);
        robot.leftRearDrive.setTargetPosition(distance*robot.CLICKS_PER_INCH);
        robot.rightFrontDrive.setTargetPosition(distance*robot.CLICKS_PER_INCH);
        robot.rightRearDrive.setTargetPosition(distance*robot.CLICKS_PER_INCH);

        int target = distance * robot.CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(target);
        robot.leftRearDrive.setTargetPosition(target);
        robot.rightFrontDrive.setTargetPosition(target);
        robot.rightRearDrive.setTargetPosition(target);

        robot.leftFrontDrive.setPower(-speed);
        robot.leftRearDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);
        robot.rightRearDrive.setPower(-speed);

        int red = robot.frontColorSensor.red();

        telemetry.addData("linear opmode is working, target = ", target);
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());

        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() &&
                (red > threshold(robot.frontDistanceSensor.getDistance(DistanceUnit.CM)))) {

            red = robot.frontColorSensor.red();
            Thread.yield();
            telemetry.addData("Distance (cm) ",
                    String.format(Locale.US, "%.02f", robot.rightDistanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Encoder Clicks ", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("color value ", red);
            telemetry.update();
        }

        int blockNumber;
        int tolerance = 4;

        if (robot.leftDistanceSensor.getDistance(DistanceUnit.CM) < robot.blockDistance[2] + tolerance &&
                robot.leftDistanceSensor.getDistance(DistanceUnit.CM) > robot.blockDistance[2] - tolerance)
            blockNumber = 2;
        else if (robot.leftDistanceSensor.getDistance(DistanceUnit.CM) < robot.blockDistance[3] + tolerance &&
                robot.leftDistanceSensor.getDistance(DistanceUnit.CM) > robot.blockDistance[3] - tolerance)
            blockNumber = 3;
        else
            blockNumber = 1;

        robot.stop();

        return blockNumber;
    }

    private double threshold (double distance) {
        return (Math.pow(distance, -1.247)) * 3003;
    }
}
