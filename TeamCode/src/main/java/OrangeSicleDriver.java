
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="OrangeSicle")
public class OrangeSicleDriver extends LinearOpMode {
    RobotHardware robot           = new RobotHardware();   // Use a Pushbot's hardware

    Drive drive = new Drive(robot, telemetry, this);
    Strafe strafe = new Strafe(robot, telemetry, this);
    GyroTurn gyroTurn = new GyroTurn(robot, telemetry, this);
    FindSkyStone findSkyStone = new FindSkyStone(robot, telemetry, this);
    SkyStoneClaw skyStoneClaw = new SkyStoneClaw(robot, telemetry, this);
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        strafe.right(0.2, 26);
        gyroTurn.absolute(0);
        strafe.right(0.2, 4);

        findSkyStone.forward(0.07, 24);

        skyStoneClaw.down();
        strafe.left(0.2, 20);
        gyroTurn.absolute(0);
        drive.backward(0.2, 48);
        gyroTurn.absolute(0);
        Thread.sleep(30000);
    }
}
