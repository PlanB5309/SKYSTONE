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

    public void strafe(double speed, int targetDistance, int maxDistance) throws InterruptedException{
        if (!linearOpMode.opModeIsActive())
            return;
        robot.setupDriveTrain();

        int target = maxDistance * robot.CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(target);
        robot.leftRearDrive.setTargetPosition(-target);
        robot.rightFrontDrive.setTargetPosition(-target);
        robot.rightRearDrive.setTargetPosition(target);

        robot.leftFrontDrive.setPower(speed);
        robot.leftRearDrive.setPower(-speed);
        robot.rightFrontDrive.setPower(-speed);
        robot.rightRearDrive.setPower(speed);


        telemetry.addData("linear opmode is working, target = ", target);
        double distanceValue = robot.sideDistanceSensor.getDistance(DistanceUnit.CM);


        telemetry.addData("linear opmode is working, target = ", target);
        double mainDirection = robot.getHeading();
        double currentDirection = mainDirection;

        while (robot.leftRearDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                && robot.rightFrontDrive.isBusy() && linearOpMode.opModeIsActive() && distanceValue > targetDistance) {
            Thread.yield();

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
            distanceValue = robot.sideDistanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("current: ", currentDirection);
            telemetry.addData("main direction: ", mainDirection);
            telemetry.addData("speed: ", speed);
            telemetry.update();
        }


//       Thread.sleep(5000);
//        telemetry.addData("", target);
        telemetry.addData("Distance (cm) ",
                String.format(Locale.US, "%.02f", robot.sideDistanceSensor.getDistance(DistanceUnit.CM)));
        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() && targetDistance < distanceValue)  {

            distanceValue = robot.sideDistanceSensor.getDistance(DistanceUnit.CM);
            Thread.yield();
            telemetry.addData("Distance (cm) ",
                    String.format(Locale.US, "%.02f", robot.sideDistanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());
            telemetry.update();
        }

        robot.stop ();
    }
}
