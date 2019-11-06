

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class GyroTurn {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;
    double currHeading;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    public GyroTurn(RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void right(double degrees) throws InterruptedException {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updateHeading();
        double target = currHeading - degrees;
        double diff;
        diff = target - currHeading;
        while (Math.abs(diff) > 1 && linearOpMode.opModeIsActive()) {
            robot.rightFrontDrive.setPower(0.5);
            robot.rightRearDrive.setPower(0.5);
            robot.rightFrontDrive.setTargetPosition(0);
            robot.rightRearDrive.setTargetPosition(0);
            diff = target - currHeading;
            telemetry.addData("diff: ", diff);
            if (diff < 0) {
                if (Math.abs(diff) > 25) {
                    robot.leftFrontDrive.setPower(robot.HIGH_TURN_POWER);
                    robot.leftRearDrive.setPower(robot.HIGH_TURN_POWER);
                }
                else {
                    robot.leftFrontDrive.setPower(robot.LOW_TURN_POWER);
                    robot.leftRearDrive.setPower(robot.LOW_TURN_POWER);
                }
            }
            if (diff > 0) {
                if (Math.abs(diff) > 25) {
                    robot.leftFrontDrive.setPower(-robot.HIGH_TURN_POWER);
                    robot.leftRearDrive.setPower(-robot.HIGH_TURN_POWER);
                }
                else {
                    robot.leftFrontDrive.setPower(-robot.LOW_TURN_POWER);
                    robot.leftRearDrive.setPower(-robot.LOW_TURN_POWER);
                }
            }
            updateHeading();
        }

        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
    }

    public void left(double degrees) throws InterruptedException {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updateHeading();
        double target = currHeading + degrees;
        double diff;
        diff = target - currHeading;
        while (Math.abs(diff) > 1 && linearOpMode.opModeIsActive()) {
            robot.leftFrontDrive.setPower(0.5);
            robot.leftRearDrive.setPower(0.5);
            robot.leftFrontDrive.setTargetPosition(0);
            robot.leftRearDrive.setTargetPosition(0);
            diff = target - currHeading;
            telemetry.addData("diff:", diff);
            if (diff < 0) {
                if (Math.abs(diff) > 25) {
                    robot.rightFrontDrive.setPower(-robot.HIGH_TURN_POWER);
                    robot.rightRearDrive.setPower(-robot.HIGH_TURN_POWER);
                }
                else {
                    robot.rightFrontDrive.setPower(-robot.LOW_TURN_POWER);
                    robot.rightRearDrive.setPower(-robot.LOW_TURN_POWER);
                }
            }
            if (diff > 0) {
                if (Math.abs(diff) > 25) {
                    robot.rightFrontDrive.setPower(robot.HIGH_TURN_POWER);
                    robot.rightRearDrive.setPower(robot.HIGH_TURN_POWER);
                }
                else {
                    robot.rightFrontDrive.setPower(robot.LOW_TURN_POWER);
                    robot.rightRearDrive.setPower(robot.LOW_TURN_POWER);
                }
            }
            updateHeading();
        }
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
    }

    public void absolute(double target) {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updateHeading();
        double diff;
        diff = target - currHeading;
        while (Math.abs(diff) > 1 && linearOpMode.opModeIsActive()) {
            diff = target - currHeading;
            telemetry.addData("diff:", diff);
            telemetry.update();
            if (diff < 0) {
                if (Math.abs(diff) > 25) {
                    robot.rightFrontDrive.setPower(-robot.HIGH_TURN_POWER);
                    robot.rightRearDrive.setPower(-robot.HIGH_TURN_POWER);
                    robot.leftFrontDrive.setPower(robot.HIGH_TURN_POWER);
                    robot.leftRearDrive.setPower(robot.HIGH_TURN_POWER);
                } else {
                    robot.leftFrontDrive.setPower(robot.LOW_TURN_POWER);
                    robot.leftRearDrive.setPower(robot.LOW_TURN_POWER);
                    robot.rightFrontDrive.setPower(-robot.LOW_TURN_POWER);
                    robot.rightRearDrive.setPower(-robot.LOW_TURN_POWER);
                }
            }
            if (diff > 0) {
                if (Math.abs(diff) > 25) {
                    robot.rightFrontDrive.setPower(robot.HIGH_TURN_POWER);
                    robot.rightRearDrive.setPower(robot.HIGH_TURN_POWER);
                    robot.leftFrontDrive.setPower(-robot.HIGH_TURN_POWER);
                    robot.leftRearDrive.setPower(-robot.HIGH_TURN_POWER);
                } else {
                    robot.leftFrontDrive.setPower(-robot.LOW_TURN_POWER);
                    robot.leftRearDrive.setPower(-robot.LOW_TURN_POWER);
                    robot.rightFrontDrive.setPower(robot.LOW_TURN_POWER);
                    robot.rightRearDrive.setPower(robot.LOW_TURN_POWER);
                }
            }
            updateHeading();
        }
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
    }


    public void updateHeading() {
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = robot.imu.getGravity();
        currHeading = angles.firstAngle;
        telemetry.addData("Heading: ", currHeading);
        telemetry.update();
    }
//    String formatAngle(AngleUnit angleUnit, double angle) {
//        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
//    }
//
//    String formatDegrees(double degrees){
//        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
//    }

}
