import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.IOException;
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

    public int forward(double speed, int distance) throws InterruptedException{
        robot.setupDriveTrain();

        robot.leftFrontDrive.setTargetPosition(10*robot.CLICKS_PER_INCH);
        robot.leftRearDrive.setTargetPosition(10*robot.CLICKS_PER_INCH);
        robot.rightFrontDrive.setTargetPosition(10*robot.CLICKS_PER_INCH);
        robot.rightRearDrive.setTargetPosition(10*robot.CLICKS_PER_INCH);
        robot.leftFrontDrive.setPower(speed);
        robot.leftRearDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);
        robot.rightRearDrive.setPower(speed);
        while (robot.sideDistanceSensor.getDistance(DistanceUnit.CM) >= 6) {
            Thread.yield();
        }
        robot.stop();
        robot.setupDriveTrain();


        int target = distance * robot.CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(target);
        robot.leftRearDrive.setTargetPosition(target);
        robot.rightFrontDrive.setTargetPosition(target);
        robot.rightRearDrive.setTargetPosition(target);

        robot.leftFrontDrive.setPower(speed);
        robot.leftRearDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);
        robot.rightRearDrive.setPower(speed);

        int red = robot.sideColorSensor.red();

        telemetry.addData("linear opmode is working, target = ", target);
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());

        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() &&
                (red > threshold(robot.sideDistanceSensor.getDistance(DistanceUnit.CM)))) {

            red = robot.sideColorSensor.red();
            Thread.yield();
            telemetry.addData("Distance (cm) ",
                    String.format(Locale.US, "%.02f", robot.sideDistanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Encoder Clicks ", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("color value ", red);
            telemetry.update();
        }

        int blockNumber;

        if (robot.leftFrontDrive.getCurrentPosition() < robot.CLICKS_PER_INCH * 4)
            blockNumber = 1;
        else if (robot.leftFrontDrive.getCurrentPosition() < robot.CLICKS_PER_INCH * 12)
            blockNumber = 2;
        else if (robot.leftFrontDrive.getCurrentPosition() < robot.CLICKS_PER_INCH * 20)
            blockNumber = 3;
        else
            blockNumber = 4;

        drive.forward(0.1, 2);
        robot.stop();

        return blockNumber;
    }

    public int backward(double speed, int distance) throws InterruptedException {
        robot.setupDriveTrain();

        robot.leftFrontDrive.setTargetPosition(-10 * robot.CLICKS_PER_INCH);
        robot.leftRearDrive.setTargetPosition(-10 * robot.CLICKS_PER_INCH);
        robot.rightFrontDrive.setTargetPosition(-10 * robot.CLICKS_PER_INCH);
        robot.rightRearDrive.setTargetPosition(-10 * robot.CLICKS_PER_INCH);
        robot.leftFrontDrive.setPower(-speed);
        robot.leftRearDrive.setPower(-speed);
        robot.rightFrontDrive.setPower(-speed);
        robot.rightRearDrive.setPower(-speed);
        while (robot.sideDistanceSensor.getDistance(DistanceUnit.CM) >= 6) {
            Thread.yield();
        }
        robot.stop();
        robot.setupDriveTrain();

        int target = distance * robot.CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(-target);
        robot.leftRearDrive.setTargetPosition(-target);
        robot.rightFrontDrive.setTargetPosition(-target);
        robot.rightRearDrive.setTargetPosition(-target);

        robot.leftFrontDrive.setPower(-speed);
        robot.leftRearDrive.setPower(-speed);
        robot.rightFrontDrive.setPower(-speed);
        robot.rightRearDrive.setPower(-speed);

        int red = robot.sideColorSensor.red();

        telemetry.addData("linear opmode is working, target = ", target);
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());

        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", robot.sideDistanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());
        telemetry.addData("color value", red);
        telemetry.update();
        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() &&
                (red > threshold(robot.sideDistanceSensor.getDistance(DistanceUnit.CM)))) {
            red = robot.sideColorSensor.red();
            Thread.yield();
            telemetry.addData("Distance (cm) ",
                    String.format(Locale.US, "%.02f", robot.sideDistanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Encoder Clicks ", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("color value ", red);
            telemetry.addData("Threshold", threshold(robot.sideDistanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }

        int blockNumber;

        if (robot.leftFrontDrive.getCurrentPosition() > robot.CLICKS_PER_INCH * -4)
            blockNumber = 1;
        else if (robot.leftFrontDrive.getCurrentPosition() > robot.CLICKS_PER_INCH * -12)
            blockNumber = 2;
        else if (robot.leftFrontDrive.getCurrentPosition() > robot.CLICKS_PER_INCH * -20)
            blockNumber = 3;
        else
            blockNumber = 4;

        drive.backward(0.1, 2);
        robot.stop();

        return blockNumber;
    }

    private double threshold (double distance) {
        return (Math.pow(distance, -1.247)) * 3003;
    }
}
