import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BlockIntake {
    RobotHardware robot;
    Telemetry telemetry;
    LinearOpMode linearOpMode;

    public BlockIntake (RobotHardware robot, Telemetry telemetry, LinearOpMode linearOpMode){
        this.robot = robot;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }

    public void startSucking () {
        robot.blockKickerServo.setPosition(robot.KICKER_OUT_POSITION);
        robot.leftIntakeMotor.setPower(robot.INTAKE_WHEEL_SPEED);
        robot.rightIntakeMotor.setPower(robot.INTAKE_WHEEL_SPEED);
    }

    public void stopSucking () {
        robot.leftIntakeMotor.setPower(0);
        robot.rightIntakeMotor.setPower(0);
        robot.blockKickerServo.setPosition(robot.KICKER_STANDARD_POSITION);
    }

    public void kickBlock () {
        robot.blockKickerServo.setPosition(robot.KICKER_IN_POSITION);
    }
}
