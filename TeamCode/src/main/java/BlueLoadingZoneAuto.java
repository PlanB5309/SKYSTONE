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

    public void runOpMode () throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        strafe.right(0.2, 25);
        gyroTurn.absolute(0);

        strafe.right(0.2, 5);

        int skyStoneNumber = findSkyStone.backward(0.08, 24);
        telemetry.addData("Stone number: ", skyStoneNumber);

        skyStoneClaw.down();
        strafe.left(0.2, 15);
        gyroTurn.absolute(0);
        drive.forward(0.3, 31 + (skyStoneNumber * 8));
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
            drive.forward(0.3,51 + (skyStoneNumber * 8));
            skyStoneClaw.up();
        }

        // If the Skystone was the third block:
        if (skyStoneNumber ==  3) {
            gyroTurn.absolute(0);
            //Don't get the other skystone because it is against the wall
        }

        //Either way, stop under the skybridge afterwards
        stopOnLine.backward(0.2, 36);
    }
}
