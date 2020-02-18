import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Loading Zone Autonomous", group = "Red Auto")
public class RedLoadingZoneAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware();   // Use a Pushbot's hardware
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    FindSkyStone findSkyStone = new FindSkyStone(robot, telemetry, this);
    SkyStoneClaw skyStoneClaw = new SkyStoneClaw(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);
    StopAtDistance stopAtDistance = new StopAtDistance(robot, telemetry, this);
    BlockGrabber blockGrabber = new BlockGrabber(robot, telemetry, this);
    BlockArm blockArm = new BlockArm(robot, telemetry, this);
    FoundationClaws foundationClaws = new FoundationClaws(robot, telemetry, this);


    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_DOWN);
        drive.forward(0.4, 22);
        stopAtDistance.forward(0.08, 5, 7);
        gyroTurn.absolute(0);
        int blockNum = 1;
        blockNum = findSkyStone.instant(AllianceColor.Red);

        if(blockNum == 1){
            stopAtDistance.left(0.15, robot.blockDistance[blockNum], 20);
        }
        if(blockNum == 2) {
            blockGrabber.grab();
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
            drive.backward(0.25, 6);
            gyroTurn.absolute(0);
            strafe.right (0.5, 88);

            //Put the block on the tray
            gyroTurn.absolute(0);
            drive.forward(0.3,7);
            blockArm.down();
            robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);

            //Grab the tray and spin it
            foundationClaws.down();
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
            drive.backward(0.3,18);
            gyroTurn.absolute(-90);
            robot.rightFoundationServo.setPosition(robot.RIGHT_FOUNDATION_SERVO_UP);
            robot.leftFoundationServo.setPosition(robot.LEFT_FOUNDATION_SERVO_UP);
            drive.forward(0.3,10);

            //Go get the second Skystone
            stopAtDistance.right(0.3,65,10);
            gyroTurn.absolute(-90);
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_DOWN);
            drive.backward(0.5,71 + 8 * 3);
            gyroTurn.absolute(0);

            //position for next skystone
            stopAtDistance.left(0.25,robot.blockDistance [5],10);
            stopAtDistance.forward(0.1, 5, 12);

            //grab the second skystone
            blockGrabber.grab();
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_BARLEY_UP);
            drive.backward(0.25, 6);
            gyroTurn.absolute(-90);
            drive.forward(0.5, 75);

            //drop the block and park
            robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);
            gyroTurn.absolute(-90);
            stopOnLine.backward(0.4,20);
        }
        if(blockNum == 3) {

        }

    }
}