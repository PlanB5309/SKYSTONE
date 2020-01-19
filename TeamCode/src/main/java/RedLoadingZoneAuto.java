import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Loading Zone Autonomous", group = "Red Auto")
public class RedLoadingZoneAuto extends LinearOpMode{
    RobotHardware robot = new RobotHardware();   // Use a Pushbot's hardware
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    FindSkyStone findSkyStone = new FindSkyStone(robot, telemetry, this);
    SkyStoneClaw skyStoneClaw = new SkyStoneClaw(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);
    StopAtDistance stopAtDistance = new StopAtDistance(robot, telemetry, this);

    public void runOpMode () throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        //Strafe until close enough to the blocks to read them accurately, then scan for the skystone
        stopAtDistance.strafe(0.15, 5, 33);
        gyroTurn.absolute(0);
        drive.backward(0.15,6);
        int skyStoneNumber = findSkyStone.forward(0.09, 24);
        telemetry.addData("Stone number: ", skyStoneNumber);

        //Grab the first skystone, then drag it through the skybridge and let go
        strafe.right(0.1, 2);
        skyStoneClaw.down();
        strafe.left(0.2, 17);
        gyroTurn.absolute(0);
        drive.backward(0.5, 40 + (skyStoneNumber * 8));
        gyroTurn.absolute(0);
        skyStoneClaw.up();

        // If the Skystone was the first or second block, get the second skystone
        if (skyStoneNumber == 1 || skyStoneNumber == 2) {
            drive.forward(0.3, 58 + (skyStoneNumber * 8));
            gyroTurn.absolute(0);
            stopAtDistance.strafe(0.1, 7, 24);
            gyroTurn.absolute(0);

            findSkyStone.forward(0.08,20);
            strafe.right(0.15,2);
            skyStoneClaw.down();
            strafe.left(0.2,15);
            gyroTurn.absolute(0);
            drive.backward(0.5,58 + (skyStoneNumber * 8));
            skyStoneClaw.up();
        }

        // If the Skystone was the third block:
        if (skyStoneNumber ==  3) {
            //Don't get the second skystone because it is against the wall
            gyroTurn.absolute(0);
        }

        //Either way, stop under the skybridge afterwards
        stopOnLine.forward(0.2, 36);
    }
}
