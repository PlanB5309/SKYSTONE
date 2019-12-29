import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "RedBuildingZoneAuto", group = "Red Auto")
public class RedBuildingZoneAuto extends LinearOpMode{
    RobotHardware robot           = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe (robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);
    FoundationClaws foundationClaws =  new FoundationClaws(robot, telemetry, this);
    StopAtDistance stopAtDistance = new StopAtDistance (robot, telemetry, this);
    SkyStoneClaw skyStoneClaw = new SkyStoneClaw (robot, telemetry, this);
    FindSkyStone findSkyStone = new FindSkyStone(robot, telemetry, this);
    public void runOpMode () throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        drive.backward(0.2, 27);
        gyroTurn.absolute(0);
        strafe.left(0.2, 12);
        gyroTurn.absolute(0);
        drive.backward(0.1, 4);
        foundationClaws.down();
        drive.forward(0.2, 33);
        foundationClaws.up();
        drive.backward (0.2, 1);
        gyroTurn.absolute(-4);
        strafe.right(0.2, 30);
        drive.backward(0.2, 18);
        gyroTurn.absolute(-90);
        drive.forward(0.4, 48);
        stopAtDistance.strafe(0.1, 5, 10);
        int blockFound = findSkyStone.forward(0.1, 24);
        if (blockFound == 1)
            drive.backward (0.1, 2);
        skyStoneClaw.down();
        strafe.left (0.2, 15);
        drive.backward(0.3, (blockFound * 8) + 40);
        skyStoneClaw.up();
        stopOnLine.forward(0.3, 48);
    }
}
