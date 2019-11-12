import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "RedBuildingZoneAuto", group = "Red Auto")
public class RedBuildingZoneAuto extends LinearOpMode{
    RobotHardware robot           = new RobotHardware();
    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    SkyStoneClaw skyStoneClaw = new SkyStoneClaw(robot, telemetry, this);
    StopOnLine stopOnLine = new StopOnLine(robot, telemetry, this);

    public void runOpMode () throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        if (opModeIsActive())
            strafe.right(0.2, 27);
        if (opModeIsActive())
            gyroTurn.absolute(0);
        if (opModeIsActive())
            strafe.right(0.2, 4);
        if (opModeIsActive())
            skyStoneClaw.tray_down();
        if (opModeIsActive())
            strafe.left(0.2, 31);
        if (opModeIsActive())
            skyStoneClaw.up();
        if (opModeIsActive())
            stopOnLine.forward(0.2, 48);

    }
}

