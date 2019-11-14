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

        strafe.right(0.2, 26);
        gyroTurn.absolute(0);
        strafe.right(0.2, 4);
        skyStoneClaw.down();
        strafe.left(0.2, 17);
        drive.backward(0.2, 48);
        skyStoneClaw.up();
        gyroTurn.absolute(0);
        drive.forward(0.2, 72);
        gyroTurn.absolute(0);
        strafe.right(0.2, 15);
        gyroTurn.absolute(0);
        strafe.right(0.2, 4);
        skyStoneClaw.down();
        strafe.left(0.2, 21);
        gyroTurn.absolute(0);
        drive.backward(0.4, 72);
        skyStoneClaw.up();
        stopOnLine.forward(0.2, 48);
    }
}
