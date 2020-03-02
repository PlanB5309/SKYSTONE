import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Loading Zone Autonomous", group = "1 Blue Auto")
public class BlueLoadingZoneAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware();   // Use a Pushbot's hardware
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    FindSkyStone findSkyStone = new FindSkyStone(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);
    StopAtDistance stopAtDistance = new StopAtDistance(robot, telemetry, this);
    BlockGrabber blockGrabber = new BlockGrabber(robot, telemetry, this);
    BlockArm blockArm = new BlockArm(robot, telemetry, this);
    FoundationClaws foundationClaws = new FoundationClaws(robot, telemetry, this);


    public void runOpMode() throws InterruptedException {
        Double backDistance = 7.0;
        int distanceForward;
        robot.init(hardwareMap);
        waitForStart();
        double startTime = System.currentTimeMillis();
        robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_DOWN);
        drive.forward(0.4, 22);
        stopAtDistance.forward(0.07, 6, 7);
        gyroTurn.absolute(0);
        Thread.sleep(100);
        int blockNum;
        blockNum = findSkyStone.instant(AllianceColor.Blue);
        int distanceToGo;

        if (blockNum == 1) {
            //go to the first skystone
            //stopAtDistance.instantLeft(.15, robot.blockDistance[blockNum], 20, Direction.Right);
            drive.backward(0.15, 1);
            distanceToGo = stopAtDistance.getTarget(robot.blockDistanceInches[blockNum], 8, Sensor.Right, Direction.Left);
            strafe.left(.15, distanceToGo);

            //get the block and take it to the tray
            drive.forward(0.15, 3);
            blockGrabber.grab();
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
            drive.backward(0.25, 7);
            gyroTurn.absolute(-6);
            strafe.left (0.5, 80);

            //Put the block on the tray
            gyroTurn.absolute(0);
            distanceToGo = stopAtDistance.getTarget(30, 6, Sensor.Back, Direction.Forward);
            drive.forward(0.3, distanceToGo);
            blockArm.down();
            robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);


            //Grab the tray and spin it
            foundationClaws.down();
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
            drive.backward(0.3,14);
            gyroTurn.absolute(90);
            robot.rightFoundationServo.setPosition(robot.RIGHT_FOUNDATION_SERVO_UP);
            robot.leftFoundationServo.setPosition(robot.LEFT_FOUNDATION_SERVO_UP);
            drive.forward(0.3,10);

            //Go get the second Skystone
//            stopAtDistance.right(0.3,65,20);
            distanceToGo = stopAtDistance.getTarget(27, 4, Sensor.Left, Direction.Right);
            strafe.right(0.3, distanceToGo);
            gyroTurn.absolute(87);
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_DOWN);
            drive.backward(0.5,62 + 8 * 3);
            gyroTurn.absolute(0);

            //position for next skystone
//            stopAtDistance.left(0.25,robot.blockDistance [4],10);
            distanceToGo = stopAtDistance.getTarget(robot.blockDistanceInches[4], 0, Sensor.Right, Direction.Right);
            strafe.right(0.25, distanceToGo);
            gyroTurn.absolute(0);
//            stopAtDistance.forward(0.1, 8, 7);
            distanceToGo = stopAtDistance.getTarget(27, 5, Sensor.Back, Direction.Forward);
            drive.forward(0.15, distanceToGo);

            //grab the second skystone
            blockGrabber.grab();
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_BARLEY_UP);
            drive.backward(0.25, 5);
            gyroTurn.absolute(85);
            drive.forward(0.5, 59);

            //drop the block and park
            robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);
            Thread.sleep(350);
            gyroTurn.absolute(87);
            stopOnLine.backward(0.4,20);

        }
        if (blockNum == 2) {
            //get the block and take it to the tray
            drive.forward(0.15, 2);
            blockGrabber.grab();
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
            drive.backward(0.25, 7);
            gyroTurn.absolute(-7);
            strafe.left (0.5, 88);

            //Put the block on the tray
            gyroTurn.absolute(0);
            distanceToGo = stopAtDistance.getTarget(30, 6, Sensor.Back, Direction.Forward);
            drive.forward(0.3, distanceToGo);
            blockArm.down();
            robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);


            //Grab the tray and spin it
            foundationClaws.down();
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
            drive.backward(0.3,14);
            gyroTurn.absolute(90);
            robot.rightFoundationServo.setPosition(robot.RIGHT_FOUNDATION_SERVO_UP);
            robot.leftFoundationServo.setPosition(robot.LEFT_FOUNDATION_SERVO_UP);
            drive.forward(0.3,10);

            //Go get the second Skystone
//            stopAtDistance.right(0.3,65,20);
            distanceToGo = stopAtDistance.getTarget(27, 4, Sensor.Left, Direction.Right);
            strafe.right(0.3, distanceToGo);
            gyroTurn.absolute(87);
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_DOWN);
            drive.backward(0.5,62 + 8 * 4);
            gyroTurn.absolute(0);

            //position for next skystone
//            stopAtDistance.left(0.25,robot.blockDistance [4],10);
            distanceToGo = stopAtDistance.getTarget(robot.blockDistanceInches[5], 0, Sensor.Right, Direction.Right);
            strafe.right(0.25, distanceToGo);
            gyroTurn.absolute(0);
//            stopAtDistance.forward(0.1, 8, 7);
            distanceToGo = stopAtDistance.getTarget(27, 5, Sensor.Back, Direction.Forward);
            drive.forward(0.15, distanceToGo);

            //grab the second skystone
            blockGrabber.grab();
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_BARLEY_UP);
            drive.backward(0.25, 5);
            gyroTurn.absolute(85);
            if(System.currentTimeMillis() > startTime + 26000) {
                //drop the block and park
                drive.forward(0.5, 67);
                robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);
                stopOnLine.backward(0.4, 20);
            }
            else{
                drive.forward(0.65, 99);
                robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);
                gyroTurn.absolute(90);
                stopOnLine.backward(0.4, 40);
            }

        }
        if (blockNum == 3) {
            //get the block and take it to the tray
            drive.backward(0.15, 1);
            stopAtDistance.right(0.15, robot.blockDistance[blockNum], 10);
            drive.forward(0.15, 3);
            blockGrabber.grab();
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
            drive.backward(0.25, 7);
            gyroTurn.absolute(-7);
            strafe.left (0.5, 96);

            //Put the block on the tray
            gyroTurn.absolute(0);
            distanceToGo = stopAtDistance.getTarget(30, 6, Sensor.Back, Direction.Forward);
            drive.forward(0.3, distanceToGo);
            blockArm.down();
            robot.blockGrabbingServo.setPosition(robot.BLOCK_SERVO_RELEASE);


            //Grab the tray and spin it
            foundationClaws.down();
            robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_UP);
            drive.backward(0.3,14);
            gyroTurn.absolute(90);
            robot.rightFoundationServo.setPosition(robot.RIGHT_FOUNDATION_SERVO_UP);
            robot.leftFoundationServo.setPosition(robot.LEFT_FOUNDATION_SERVO_UP);
            drive.forward(0.3,10);

            //don't run into alliance partner
            distanceToGo = stopAtDistance.getTarget(27, 4, Sensor.Left, Direction.Right);
            strafe.right(0.2, distanceToGo);
            gyroTurn.absolute(90);

            //park on line
            stopAtDistance.left(0.3, 65, 10);
            stopOnLine.backward(0.4, 40);
        }

    }
}