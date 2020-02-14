import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Red Extra Block Autonomous")
public class RedSecondBlockAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    FindSkyStone findSkyStone = new FindSkyStone(robot, telemetry, this);
    StopAtDistance stopAtDistance = new StopAtDistance(robot, telemetry, this);
    Drive drive = new Drive(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    BlockGrabber blockGrabber = new BlockGrabber(robot, telemetry, this);
    BlockArm blockArm = new BlockArm(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            blockGrabber.release();
            telemetry.addData("3", "");
            telemetry.update();
            Thread.sleep(1000);
            telemetry.addData("2", "");
            telemetry.update();
            Thread.sleep(1000);
            telemetry.addData("1", "");
            telemetry.update();
            Thread.sleep(1000);
            telemetry.addData("Go! ", "");
            telemetry.update();

            strafe.right(0.2, 4);
            stopAtDistance.forward(0.2, 7, 84);
            gyroTurn.absolute(-90);
//            stopAtDistance.left(0.2, robot.blockDistance[5], 70);
            blockArm.down();
            stopAtDistance.forward(0.2, 7, 24);

            stopAtDistance.left(0.2, robot.blockDistance[5], 70);
            gyroTurn.absolute(-90);
            int blockNum = findSkyStone.instant(AllianceColor.Red);
            telemetry.addData("block number", blockNum);
            telemetry.update();
            Thread.sleep(200);

            if (blockNum == 3) {/*Can't get this one*/}
            else {
                if (blockNum == 1)
                    stopAtDistance.right(0.2, robot.blockDistance[4], 10);
                blockGrabber.grab();
                blockArm.up();
                drive.backward(0.2, 30);
                drive.forward(0.2, 4);
                gyroTurn.absolute(-180);
//                stopOnLine.forward(0.15, 60);
                drive.forward(0.2, 72 + (blockNum)*8);
                gyroTurn.absolute(-180);
                blockArm.down();
                blockGrabber.release();
                blockArm.up();
                stopOnLine.backward(0.1, 20);
            }
            Thread.sleep(100000);

        }
    }

}
