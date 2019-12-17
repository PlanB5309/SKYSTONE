import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BlockFlipper {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public BlockFlipper (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void liftOut () throws InterruptedException{
        robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_TOP);
        Thread.sleep(750);
        robot.blockTurningServo.setPosition(robot.BLOCK_TURNING_SERVO_OUT);
        Thread.sleep(750);
        robot.blockFlippingServo.setPosition(robot.LIFT_BLOCK_SERVO_START);
        Thread.sleep(750);
    }
}
