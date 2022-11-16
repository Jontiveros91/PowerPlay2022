
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "TeleOp13670_PP", group = "Ontiveros")
public class TeleOp13670_PP extends LinearOpMode {

    Hardware13670_4 robot = new Hardware13670_4();
    private final ElapsedTime runtime = new ElapsedTime();
    static final double FORWARD_SPEED = 0.6;
    double          armOffset = 0;                // Servo mid position
    final double    ARM_SPEED  = 0.001 ;
    // sets rate to move servo
    // Test
    //double flipOffset = 0.5;
    
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        Hardware13670_4.noEncoder();

        waitForStart();
        while (opModeIsActive()) {

            // use gamepad1 left stick to go forward, backward, and turn
            if (gamepad1.left_stick_button) {
                Hardware13670_4.leftFront.setPower(-1* (gamepad1.left_stick_y) + 1 * (gamepad1.left_stick_x));
                Hardware13670_4.rightFront.setPower(-1 * (gamepad1.left_stick_y) - 1 * (gamepad1.left_stick_x));
                Hardware13670_4.leftBack.setPower(-1 * (gamepad1.left_stick_y) + 1 * (gamepad1.left_stick_x));
                Hardware13670_4.rightBack.setPower(-1 * (gamepad1.left_stick_y) - 1 * (gamepad1.left_stick_x));
            }
            else {
                Hardware13670_4.leftFront.setPower(-0.7 * (gamepad1.left_stick_y) + 0.7 * (gamepad1.left_stick_x));
                Hardware13670_4.rightFront.setPower(-0.7 * (gamepad1.left_stick_y) - 0.7 * gamepad1.left_stick_x);
                Hardware13670_4.leftBack.setPower(-0.7 * (gamepad1.left_stick_y) + 0.7 * gamepad1.left_stick_x);
                Hardware13670_4.rightBack.setPower(-0.7 * (gamepad1.left_stick_y) - 0.7 * (gamepad1.left_stick_x));
            }

            telemetry.addData("leftFront position", Hardware13670_4.leftFront.getCurrentPosition());
            telemetry.update();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors",gamepad1.left_stick_y +", " + gamepad1.left_stick_x);
            telemetry.update();

            //use gamepad1 right stick to move left and right
            if(gamepad1.right_stick_x> 0){
                robot.goRight(gamepad1.right_stick_x);
            }
            if(gamepad1.right_stick_x< 0) {
                robot.goLeft(-gamepad1.right_stick_x);
            }
            // use right & left trigger to move the lift up & down
            if (gamepad1.left_trigger > 0 ) {
                robot.lift.setPower(gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger > 0) {
                robot.lift.setPower(-gamepad1.right_trigger);
            }
            else {
                robot.lift.setPower(0);
            }
            // use right & left bumper to open/close claw
            if (gamepad1.right_bumper){
                robot.claw.setPower(1);
            }
            else if (gamepad1.left_bumper){
                robot.claw.setPower(-1);
            }
            else {
                robot.claw.setPower(0);
            }

        }

    }
}

