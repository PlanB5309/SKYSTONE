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
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);

        return blockNumber;
    }

    public void backward(double speed, int distance) throws InterruptedException {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

//       Thread.sleep(5000);
//        telemetry.addData("", target);
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", robot.sideDistanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());
        telemetry.addData("color value", red);
        telemetry.update();
        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() &&
                (red > robot.SKYSTONE_COLOR_THRESHOLD))  {
            red = robot.sideColorSensor.red();
            Thread.yield();
            telemetry.addData("Distance (cm) ",
                    String.format(Locale.US, "%.02f", robot.sideDistanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Encoder Clicks ", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("color value ", red);
            telemetry.update();
        }

        if (robot.leftFrontDrive.getCurrentPosition() > robot.CLICKS_PER_INCH * -4) {
            drive.forward(0.1, 2);
        } else { //Or if the distance driven was more than 4 inches...
            drive.backward(0.1, 2);
        }

        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);

    }

    private double threshold (double distance ) {

        return (Math.pow(distance, -1.247)) * 3003;
    }
}
