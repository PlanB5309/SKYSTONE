
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

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
        strafe.right(0.2, 25);
        gyroTurn.absolute(0);
//        for (int i = 0; i < 1000; i++) {
//            telemetry.addData("Distance (cm): ",
//                    String.format(Locale.US, "%.02f", robot.sideDistanceSensor.getDistance(DistanceUnit.CM)));
//            telemetry.addData("Encoder Clicks: ", robot.leftRearDrive.getCurrentPosition());
//            telemetry.addData("red color value: ", robot.sideColorSensor.red());
//            telemetry.update();
//            Thread.sleep(10);
//        }

        strafe.right(0.2, 3);

        int skyStoneNumber = findSkyStone.forward(0.07, 24);
        //1 - 31 + 8*num blocks
        telemetry.addData("Stone number: ", skyStoneNumber);

        skyStoneClaw.down();
        strafe.left(0.2, 20);
        gyroTurn.absolute(0);
        drive.backward(0.2, 31 + (skyStoneNumber*8));
        gyroTurn.absolute(0);
        skyStoneClaw.up();
        Thread.sleep(30000);
    }
}
