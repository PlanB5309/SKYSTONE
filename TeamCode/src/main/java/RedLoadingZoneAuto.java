import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red Loading Zone Autonomous", group = "Red Auto")
public class RedLoadingZoneAuto extends LinearOpMode{
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
            strafe.right(0.2, 26);
        if (opModeIsActive())
            gyroTurn.absolute(0);
        if (opModeIsActive())
            strafe.right(0.2, 4);
        if (opModeIsActive())
            skyStoneClaw.down();
        if (opModeIsActive())
            strafe.left(0.2, 17);
        if (opModeIsActive())
            drive.backward(0.2, 48);
        if (opModeIsActive())
            skyStoneClaw.up();
        if (opModeIsActive())
            gyroTurn.absolute(0);
        if (opModeIsActive())
            drive.forward(0.2, 72);
        if (opModeIsActive())
            gyroTurn.absolute(0);
        if (opModeIsActive())
            strafe.right(0.2, 15);
        if (opModeIsActive())
            gyroTurn.absolute(0);
        if (opModeIsActive())
            strafe.right(0.2, 4);
        if (opModeIsActive())
            skyStoneClaw.down();
        if (opModeIsActive())
            strafe.left(0.2, 21);
        if (opModeIsActive())
            gyroTurn.absolute(0);
        if (opModeIsActive())
            drive.backward(0.4, 72);
        if (opModeIsActive())
            skyStoneClaw.up();
        if (opModeIsActive())
            stopOnLine.forward(0.2, 48);
    }
}
