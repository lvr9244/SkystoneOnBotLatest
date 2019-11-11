/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="EncoderParkRight-Autonomous", group="Linear Opmode")
//@Disabled
public class EncoderParkRight extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;
    private Servo foundationLift = null;

    static final double COUNTS_PER_MOTOR_REV = 1680;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    double pi = 3.14159265358979;
    double turnFourtyFive = (15.78*pi)/8;
    double turnNinety = (17.78*pi)/4;
    double turn180 = (17.78*pi)/2;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159);

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;
    private static final float mmTargetHeight = (6) * mmPerInch;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "leftMotorFront");
        rightDrive = hardwareMap.get(DcMotor.class, "rightMotorFront");
        leftDriveBack = hardwareMap.get(DcMotor.class, "leftMotorRear");
        rightDriveBack = hardwareMap.get(DcMotor.class, "rightMotorRear");
        foundationLift = hardwareMap.get(Servo.class, "foundationDrag");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        foundationLift.setDirection(Servo.Direction.FORWARD);
        
        

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
            
            //move forward 10 inches
            foundationLift.setPosition(-1);
            encoderDrive(0.75, 10, 10, 10, 10, 30);
            sleep(500); //half a second MAYBE???
            //DRIVE RIGHT 50 INCHES MAYBE
            encoderDrive(0.75, 45, -45, -45, 45, 30);
            sleep(500);
            
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
          //  telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        
    }
    
    public void encoderDrive(double speed, double leftFrontInches, double rightFrontInches, double leftRearInches, double rightRearInches, double timeOutSeconds){
        int newLeftFrontTarget, newRightFrontTarget, newLeftRearTarget, newRightRearTarget;
        
        if(opModeIsActive()){
            
            //define targets
            newLeftFrontTarget = leftDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftRearTarget = leftDriveBack.getCurrentPosition() + (int)(leftRearInches * COUNTS_PER_INCH);
            newRightRearTarget = rightDriveBack.getCurrentPosition() + (int)(rightRearInches * COUNTS_PER_INCH);
            
            //set targets for dcmotors
            leftDrive.setTargetPosition(newLeftFrontTarget);
            rightDrive.setTargetPosition(newRightFrontTarget);
            leftDriveBack.setTargetPosition(newLeftRearTarget);
            rightDriveBack.setTargetPosition(newRightRearTarget);
            
            //set mode to RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            //reset runtime and start motion
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));
            leftDriveBack.setPower(Math.abs(speed));
            rightDriveBack.setPower(Math.abs(speed));
            
            while(opModeIsActive() && (runtime.seconds()<timeOutSeconds) && (leftDrive.isBusy() && rightDrive.isBusy() && leftDriveBack.isBusy() && rightDriveBack.isBusy())){
             telemetry.addData("path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
             telemetry.addData("path2", "Running at %7d :%7d", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition(), leftDriveBack.getCurrentPosition(), rightDriveBack.getCurrentPosition());
             telemetry.update();
            }
            
            //stop motion
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            leftDriveBack.setPower(0);
            rightDriveBack.setPower(0);
            
            //turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            sleep(250);
        }
    }
    
        public void regularDrive(double speed){
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);
            leftDriveBack.setPower(speed);
            rightDriveBack.setPower(speed);
        }
    }

