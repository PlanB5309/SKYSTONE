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

    public int left(double speed, int distance) throws InterruptedException{        int target = distance * robot.CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(-target);
        robot.leftRearDrive.setTargetPosition(target);
        robot.rightFrontDrive.setTargetPosition(target);
        robot.rightRearDrive.setTargetPosition(-target);

        robot.setupDriveTrain();

        robot.leftFrontDrive.setPower(-speed);
        robot.leftRearDrive.setPower(speed);
        robot.rightFrontDrive.setPower(speed);
        robot.rightRearDrive.setPower(-speed);

        int red = getAdjustedRed();
        boolean foundSkyStone = false;

        telemetry.addData("linear opmode is working, target = ", target);
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());

        if(red < threshold(robot.frontDistanceSensor.getDistance(DistanceUnit.CM)))
            foundSkyStone=true;

        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() &&
                (red > threshold(robot.frontDistanceSensor.getDistance(DistanceUnit.CM)))) {

            red = getAdjustedRed();
            if(red < threshold(robot.frontDistanceSensor.getDistance(DistanceUnit.CM)))
                foundSkyStone=true;
            Thread.yield();
            telemetry.addData("Distance (cm) ",
                    String.format(Locale.US, "%.02f", robot.rightDistanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Encoder Clicks ", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("color value ", red);
            telemetry.update();
        }

        int blockNumber = 1;
        int tolerance = 7;

        if (robot.leftDistanceSensor.getDistance(DistanceUnit.CM) < robot.blockDistance[2] + tolerance &&
                robot.leftDistanceSensor.getDistance(DistanceUnit.CM) > robot.blockDistance[2] - tolerance && foundSkyStone)
            blockNumber = 2;
        else if (robot.leftDistanceSensor.getDistance(DistanceUnit.CM) < robot.blockDistance[3] + tolerance &&
                robot.leftDistanceSensor.getDistance(DistanceUnit.CM) > robot.blockDistance[3] - tolerance && foundSkyStone)
            blockNumber = 3;

        robot.stop();
        telemetry.addData("Distance Sensor", robot.leftDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Skystone Found?:", foundSkyStone);
        telemetry.addData("Block Number:", blockNumber);
        telemetry.update();
        return blockNumber;
    }
    public int right(double speed, int distance) throws InterruptedException{        int target = distance * robot.CLICKS_PER_INCH;
        robot.leftFrontDrive.setTargetPosition(target);
        robot.leftRearDrive.setTargetPosition(-target);
        robot.rightFrontDrive.setTargetPosition(-target);
        robot.rightRearDrive.setTargetPosition(target);

        robot.setupDriveTrain();

        robot.leftFrontDrive.setPower(speed);
        robot.leftRearDrive.setPower(-speed);
        robot.rightFrontDrive.setPower(-speed);
        robot.rightRearDrive.setPower(speed);

        int red = getAdjustedRed();
        boolean foundSkyStone = false;

        telemetry.addData("linear opmode is working, target = ", target);
        telemetry.addData("Encoder Clicks", robot.leftRearDrive.getCurrentPosition());

        if(red < threshold(robot.frontDistanceSensor.getDistance(DistanceUnit.CM)))
            foundSkyStone=true;

        while (robot.leftRearDrive.isBusy() && linearOpMode.opModeIsActive() &&
                (red > threshold(robot.frontDistanceSensor.getDistance(DistanceUnit.CM)))) {

            red = getAdjustedRed();
            if(red < threshold(robot.frontDistanceSensor.getDistance(DistanceUnit.CM)))
                foundSkyStone=true;
            Thread.yield();
            telemetry.addData("Distance (cm) ",
                    String.format(Locale.US, "%.02f", robot.rightDistanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("Encoder Clicks ", robot.leftRearDrive.getCurrentPosition());
            telemetry.addData("color value ", red);
            telemetry.update();
        }

        int blockNumber = 1;
        int tolerance = 7;

        if (robot.leftDistanceSensor.getDistance(DistanceUnit.CM) < robot.blockDistance[2] + tolerance &&
                robot.leftDistanceSensor.getDistance(DistanceUnit.CM) > robot.blockDistance[2] - tolerance && foundSkyStone)
            blockNumber = 2;
        else if (robot.leftDistanceSensor.getDistance(DistanceUnit.CM) < robot.blockDistance[3] + tolerance &&
                robot.leftDistanceSensor.getDistance(DistanceUnit.CM) > robot.blockDistance[3] - tolerance && foundSkyStone)
            blockNumber = 3;


        robot.stop();
        telemetry.addData("Distance Sensor", robot.leftDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Skystone Found?:", foundSkyStone);
        telemetry.addData("Block Number:", blockNumber);
        telemetry.update();
        return blockNumber;
    }

    private double threshold (double distance) {
        return (Math.pow(distance, -1.247)) * 3003;
    }
    private int getAdjustedRed () { return robot.frontColorSensor.red() + robot.AMBIENT_LIGHT_MODIFIER;}
}
