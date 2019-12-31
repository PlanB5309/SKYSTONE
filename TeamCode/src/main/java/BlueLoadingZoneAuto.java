import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Loading Zone Autonomous", group = "Blue Auto")
public class BlueLoadingZoneAuto extends LinearOpMode{
    RobotHardware robot           = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    SkyStoneClaw skyStoneClaw = new SkyStoneClaw(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);
    FindSkyStone findSkyStone = new FindSkyStone(robot, telemetry, this);
    StopAtDistance stopAtDistance = new StopAtDistance(robot, telemetry, this);
    BlockIntake blockIntake = new BlockIntake(robot, telemetry, this);
    BlockFlipper blockFlipper = new BlockFlipper(robot, telemetry, this);

    public void runOpMode () throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        strafe.right(0.2, 25);
        gyroTurn.absolute(0);

        strafe.right(0.1, 4);
        int skyStoneNumber = findSkyStone.backward(0.08, 24);
        Thread.sleep(1000);
        telemetry.addData("Stone number: ", skyStoneNumber);

        skyStoneClaw.down();
        strafe.left(0.2, 15);
        gyroTurn.absolute(0);
        drive.forward(0.3, 40 + (skyStoneNumber * 8));
        gyroTurn.absolute(0);
        skyStoneClaw.up();


        // If the Skystone was the first or second block:
        if (skyStoneNumber == 1 || skyStoneNumber == 2) {
            drive.backward(0.3, 47 + (skyStoneNumber * 8));
            gyroTurn.absolute(0);
            stopAtDistance.strafe(0.1, 4, 24);
            gyroTurn.absolute(0);

            findSkyStone.backward(0.08,20);
            skyStoneClaw.down();
            strafe.left(0.2,13);
            gyroTurn.absolute(0);
            drive.forward(0.3,60 + (skyStoneNumber * 8));
            skyStoneClaw.up();
        }

        // If the Skystone was the third block:
        if (skyStoneNumber ==  3) {
            drive.backward(0.3, 74);
            gyroTurn.absolute(0);
            stopAtDistance.strafe(0.1, 5, 24);
            gyroTurn.absolute(0);
            blockIntake.startSucking();
            strafe.right(0.2, 12);
            drive.backward(0.2, 5);
            Thread.sleep(500);
            blockIntake.stopSucking();
            strafe.left(0.2, 24);
            gyroTurn.absolute(0);
            robot.blockKickerServo.setPosition(robot.KICKER_IN_POSITION);
            drive.forward(0.4, 70);
            robot.blockKickerServo.setPosition(robot.KICKER_OUT_POSITION);
            robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_GRAB);
            Thread.sleep(400);
            blockFlipper.liftOut();
        }

        //Either way, stop under the skybridge afterwards
        stopOnLine.backward(0.2, 36);
    }
}
