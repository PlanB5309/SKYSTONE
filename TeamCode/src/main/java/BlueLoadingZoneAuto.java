import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Loading Zone Autonomous", group = "Blue Auto")
public class BlueLoadingZoneAuto extends LinearOpMode{
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
        stopAtDistance.forward(0.08,5, 7);
        gyroTurn.absolute(0);
        int blockNum = findSkyStone.right(0.15, 8);
        gyroTurn.absolute(0);
        stopAtDistance.right(0.15, robot.blockDistance[blockNum], 20);
        gyroTurn.absolute(0);
        blockGrabber.grab();
        robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
        drive.backward(0.15, 4);
        gyroTurn.absolute(-0);

        strafe.left(0.3, 71 + (8*blockNum));
        gyroTurn.absolute(0);
        drive.forward(0.15, 6);
        blockArm.down();
        robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);
        robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
//        if(blockNum != 1) {
//            drive.backward(0.15, 5);
//            gyroTurn.absolute(0);
//            strafe.right(0.6, robot.blockTravelDistance[blockNum]);
//            gyroTurn.absolute(0);
//            stopAtDistance.right(0.1, robot.blockDistance[blockNum + 3], 10);
//            gyroTurn.absolute(0);
//            blockArm.down();
//            stopAtDistance.forward(0.08, 5, 20);
//            blockGrabber.grab();
//            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
//            drive.backward(0.15, 5);
//            gyroTurn.absolute(0);
//            strafe.left(0.6, robot.blockTravelDistance[blockNum] + 24);
//            gyroTurn.absolute(0);
//            drive.forward(0.15, 6);
//            blockArm.down();
//            robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);
//            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
//        }
        drive.forward(0.15,2);
        foundationClaws.down();
        drive.backward(0.15, 20);
        gyroTurn.absolute(90);
        strafe.right(0.15,10);
        stopOnLine.backward(0.15,48);

        Thread.sleep(9999999);
    }
}
