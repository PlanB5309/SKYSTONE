import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StopOnLine {

    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    double mainDirection;
    double currentDirection;

    public StopOnLine (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void forward(double speed, int distance) throws InterruptedException{
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

        int red = robot.colorSensor.red();
        int blue  = robot.colorSensor.blue();

        mainDirection = robot.getHeading();
        currentDirection = mainDirection;

        while ( robot.leftRearDrive.isBusy() &&
                robot.leftFrontDrive.isBusy() &&
                robot.rightRearDrive.isBusy() &&
                robot.rightFrontDrive.isBusy() &&
                linearOpMode.opModeIsActive() &&
                red < robot.RED_THRESHOLD &&
                blue < robot.BLUE_THRESHOLD) {
            Thread.yield();
            if (currentDirection < mainDirection) {
                robot.leftFrontDrive.setPower(speed - 0.02);
                robot.leftRearDrive.setPower(speed - 0.02);
                robot.rightFrontDrive.setPower(speed + 0.02);
                robot.rightRearDrive.setPower(speed + 0.02);
            }
            else if (currentDirection > mainDirection) {
                robot.leftFrontDrive.setPower(speed + 0.02);
                robot.leftRearDrive.setPower(speed + 0.02);
                robot.rightFrontDrive.setPower(speed - 0.02);
                robot.rightRearDrive.setPower(speed - 0.02);
            }
            else {
                robot.leftFrontDrive.setPower(speed);
                robot.leftRearDrive.setPower(speed);
                robot.rightFrontDrive.setPower(speed);
                robot.rightRearDrive.setPower(speed);
            }
            currentDirection = robot.getHeading();
            blue  = robot.colorSensor.blue();
            red = robot.colorSensor.red();

            telemetry.addData("current: ", currentDirection);
            telemetry.addData("main direction: ", mainDirection);
            telemetry.addData("speed: ", speed);
            telemetry.update();
        }

        robot.stop ();
    }

    public void backward(double speed, int distance) {
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

        int red = robot.colorSensor.red();
        int blue  = robot.colorSensor.blue();

        mainDirection = robot.getHeading();
        currentDirection = mainDirection;

        while ( robot.leftRearDrive.isBusy() &&
                robot.leftFrontDrive.isBusy() &&
                robot.rightRearDrive.isBusy() &&
                robot.rightFrontDrive.isBusy() &&
                linearOpMode.opModeIsActive() &&
                red < robot.RED_THRESHOLD &&
                blue < robot.BLUE_THRESHOLD) {

            if (currentDirection < mainDirection) {
                robot.leftFrontDrive.setPower(speed + 0.02);
                robot.leftRearDrive.setPower(speed + 0.02);
                robot.rightFrontDrive.setPower(speed - 0.02);
                robot.rightRearDrive.setPower(speed - 0.02);
            }
            else if (currentDirection > mainDirection) {
                robot.leftFrontDrive.setPower(speed - 0.02);
                robot.leftRearDrive.setPower(speed - 0.02);
                robot.rightFrontDrive.setPower(speed + 0.02);
                robot.rightRearDrive.setPower(speed + 0.02);
            }
            else {
                robot.leftFrontDrive.setPower(-speed);
                robot.leftRearDrive.setPower(-speed);
                robot.rightFrontDrive.setPower(-speed);
                robot.rightRearDrive.setPower(-speed);
            }

            currentDirection = robot.getHeading();
            blue  = robot.colorSensor.blue();
            red = robot.colorSensor.red();

            telemetry.addData("current: ", currentDirection);
            telemetry.addData("main direction: ", mainDirection);
            telemetry.addData("speed: ", speed);
            telemetry.update();
        }
        robot.stop ();
    }

    public void strafeRight(double speed, int distance) {
        robot.setupDriveTrain();

        int target = distance * robot.STRAFE_CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(target);
        robot.leftRearDrive.setTargetPosition(-target);
        robot.rightFrontDrive.setTargetPosition(-target);
        robot.rightRearDrive.setTargetPosition(target);

        robot.leftFrontDrive.setPower(speed);
        robot.leftRearDrive.setPower(-speed);
        robot.rightFrontDrive.setPower(-speed);
        robot.rightRearDrive.setPower(speed);

        int red = robot.colorSensor.red();
        int blue  = robot.colorSensor.blue();

        mainDirection = robot.getHeading();
        currentDirection = mainDirection;

        while (robot.leftRearDrive.isBusy() &&
                robot.leftFrontDrive.isBusy() &&
                robot.rightRearDrive.isBusy() &&
                robot.rightFrontDrive.isBusy() &&
                linearOpMode.opModeIsActive() &&
                red < robot.RED_THRESHOLD &&
                blue < robot.BLUE_THRESHOLD) {

            if (currentDirection < mainDirection) {
                robot.leftFrontDrive.setPower(speed - 0.02);
                robot.leftRearDrive.setPower(speed + 0.02);
                robot.rightFrontDrive.setPower(speed - 0.02);
                robot.rightRearDrive.setPower(speed + 0.02);
            } else if (currentDirection > mainDirection) {
                robot.leftFrontDrive.setPower(speed + 0.02);
                robot.leftRearDrive.setPower(speed - 0.02);
                robot.rightFrontDrive.setPower(speed + 0.02);
                robot.rightRearDrive.setPower(speed - 0.02);
            } else {
                robot.leftFrontDrive.setPower(-speed);
                robot.leftRearDrive.setPower(speed);
                robot.rightFrontDrive.setPower(speed);
                robot.rightRearDrive.setPower(-speed);
            }

            currentDirection = robot.getHeading();
            red = robot.colorSensor.red();
            blue = robot.colorSensor.blue();

            telemetry.addData("current: ", currentDirection);
            telemetry.addData("main direction: ", mainDirection);
            telemetry.addData("speed: ", speed);
            telemetry.update();
        }

        robot.stop ();

    }

    public void strafeLeft(double speed, int distance) {
        robot.setupDriveTrain();

        int target = distance * robot.STRAFE_CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(-target);
        robot.leftRearDrive.setTargetPosition(target);
        robot.rightFrontDrive.setTargetPosition(target);
        robot.rightRearDrive.setTargetPosition(-target);

        robot.leftFrontDrive.setPower(-speed);
        robot.leftRearDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);
        robot.rightRearDrive.setPower(-speed);

        int red = robot.colorSensor.red();
        int blue  = robot.colorSensor.blue();
        
        mainDirection = robot.getHeading();
        currentDirection = mainDirection;

        while ( robot.leftRearDrive.isBusy() &&
                robot.leftFrontDrive.isBusy() &&
                robot.rightRearDrive.isBusy() &&
                robot.rightFrontDrive.isBusy() &&
                linearOpMode.opModeIsActive() &&
                red < robot.RED_THRESHOLD &&
                blue < robot.BLUE_THRESHOLD) {

            if (currentDirection < mainDirection) {
                robot.leftFrontDrive.setPower(speed + 0.02);
                robot.leftRearDrive.setPower(speed - 0.02);
                robot.rightFrontDrive.setPower(speed + 0.02);
                robot.rightRearDrive.setPower(speed - 0.02);
            } else if (currentDirection > mainDirection) {
                robot.leftFrontDrive.setPower(speed - 0.02);
                robot.leftRearDrive.setPower(speed + 0.02);
                robot.rightFrontDrive.setPower(speed - 0.02);
                robot.rightRearDrive.setPower(speed + 0.02);
            } else {
                robot.leftFrontDrive.setPower(-speed);
                robot.leftRearDrive.setPower(speed);
                robot.rightFrontDrive.setPower(speed);
                robot.rightRearDrive.setPower(-speed);
            }
            currentDirection = robot.getHeading();
            red = robot.colorSensor.red();
            blue = robot.colorSensor.blue();

            telemetry.addData("current: ", currentDirection);
            telemetry.addData("main direction: ", mainDirection);
            telemetry.addData("speed: ", speed);
            telemetry.update();

        }
        robot.stop();
    }

}
