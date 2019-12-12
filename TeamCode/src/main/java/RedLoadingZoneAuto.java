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
        strafe.right(0.2, 25);
        gyroTurn.absolute(0);


        strafe.right(0.1, 4);

        int skyStoneNumber = findSkyStone.forward(0.08, 24);
        //1 - 31 + 8*num blocks
        telemetry.addData("Stone number: ", skyStoneNumber);

        skyStoneClaw.down();
        strafe.left(0.2, 15);
        gyroTurn.absolute(0);
        drive.backward(0.3, 31 + (skyStoneNumber * 8));
        gyroTurn.absolute(0);
        skyStoneClaw.up();

        // If the Skystone was the first or second block:
        if (skyStoneNumber == 1 || skyStoneNumber == 2) {
            drive.forward(0.3, 47 + (skyStoneNumber * 8));
            gyroTurn.absolute(0);
            stopAtDistance.strafe(0.1, 4, 24);
            gyroTurn.absolute(0);

            findSkyStone.forward(0.08,20);
            skyStoneClaw.down();
            strafe.left(0.2,13);
            gyroTurn.absolute(0);
            drive.backward(0.3,60 + (skyStoneNumber * 8));
            skyStoneClaw.up();
        }

        // If the Skystone was the third block:
        if (skyStoneNumber ==  3) {
            gyroTurn.absolute(0);
            //Don't get the second skystone because it is against the wall
        }

        //Either way, stop under the skybridge afterwards
        stopOnLine.forward(0.2, 36);
    }
}
