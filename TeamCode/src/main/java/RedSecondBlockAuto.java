import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Extra Block Autonomous", group = "2 Red Auto")
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

        //Drive into the loading zone
        strafe.left(0.2, 4);
        drive.backward(0.35, 66);
        gyroTurn.absolute(90);
        blockArm.down();
        stopAtDistance.forward(0.2, 7, 24);

        //Strafe until the robot is next to the wall, then scan the stones
        stopAtDistance.left(0.2, robot.blockDistance[5], 70);
        gyroTurn.absolute(90);
        int blockNum = findSkyStone.instant(AllianceColor.Red);
        telemetry.addData("block number", blockNum);
        telemetry.update();
        Thread.sleep(200);

        //Grab the stone and back up
        gyroTurn.absolute(90);
        if (blockNum == 1)
            stopAtDistance.right(0.2, robot.blockDistance[4], 10);
        blockGrabber.grab();
        blockArm.middle();
        drive.backward(0.2, 30);
        drive.forward(0.3, 4);

        //Drag it under the bridge and let go
        gyroTurn.absolute(0);
        drive.forward(0.35, 55 + (blockNum)*8);
        gyroTurn.absolute(0);
        blockArm.down();
        blockGrabber.release();
        blockArm.up();

        //Park
        stopOnLine.backward(0.1, 20);
        Thread.sleep(100000);

    }

}
