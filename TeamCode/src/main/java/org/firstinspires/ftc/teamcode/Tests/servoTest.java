package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name ="servoTest")
public class servoTest extends LinearOpMode {

    private Servo claw;

    @Override
    public void runOpMode() {
        claw = hardwareMap.servo.get("claw");

        waitForStart();

        while (opModeIsActive()){

            if(gamepad2.a)
                claw.setPosition(1.0);
            if(gamepad2.b)
                claw.setPosition(0.0);
            if(gamepad2.y)
                claw.setPosition(0.5);
            if(gamepad2.x)
                claw.setPosition(0.25);
        }
    }
}
