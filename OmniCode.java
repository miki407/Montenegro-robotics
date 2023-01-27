package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;




@TeleOp(name="OmniCode", group="Linear Opmode")

public class OmniCode extends LinearOpMode {
    
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left1 = null;
    private DcMotor right1 = null;
    private DcMotor left2 = null;
    private DcMotor right2 = null;
    
    private DcMotor arm_r = null;
    private DcMotor arm_1 = null;
    private DcMotor arm_2 = null;
    private DcMotor arm_3 = null;
    // todo: write your code here
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        left1  = hardwareMap.get(DcMotor.class, "left1");
        right1 = hardwareMap.get(DcMotor.class, "right1");
        left2  = hardwareMap.get(DcMotor.class, "left2");
        right2 = hardwareMap.get(DcMotor.class, "right2");
        
        left1.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right2.setDirection(DcMotor.Direction.REVERSE);
        
        arm_r  = hardwareMap.get(DcMotor.class, "arm_r");
        arm_1 = hardwareMap.get(DcMotor.class, "arm_1");
        arm_2  = hardwareMap.get(DcMotor.class, "arm_2");
        arm_3 = hardwareMap.get(DcMotor.class, "arm_3");
        
        arm_r.setDirection(DcMotor.Direction.FORWARD);
        arm_1.setDirection(DcMotor.Direction.FORWARD);
        arm_2.setDirection(DcMotor.Direction.REVERSE);
        arm_3.setDirection(DcMotor.Direction.REVERSE);
        
        arm_r.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        arm_r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        waitForStart();
        runtime.reset();

        int left1_target = 0;
        int right1_target = 0;
        int left2_target = 0;
        int right2_target = 0;
        
        int target_a = 0;
        int target_b = 0;
        int target_c = 0;
        int target_r = 0;
        
        double count_per_rad = 472.5;
        
        double angle_a_nom = Math.PI/2;
        double angle_b_nom = Math.PI/2;
        double angle_c_nom = Math.PI/2;
        
        double angle_c = angle_c_nom;

        double a = 340;
        double b = 350;
        double c = 200;
        
        double pos_x_0 = Math.cos(angle_a_nom) * a + Math.cos(angle_b_nom) * b + Math.cos(angle_c_nom) * c;
        double pos_y_0 = Math.sin(angle_a_nom) * a + Math.sin(angle_b_nom) * b + Math.sin(angle_c_nom) * c;
        
        int one_time_run = 0;
        
        double angle_a_offset = Math.PI/2;
        double angle_b_offset = Math.PI;
        double angle_c_offset = Math.PI;
        
        while (opModeIsActive()) {
            double r2l1 = 0.0;
            double r1l2 = 0.0;
            
            double x = gamepad1.right_stick_x;
            double y = -gamepad1.right_stick_y;
            
            double power = Math.sqrt( Math.pow(x , 2) + Math.pow(y , 2));
             
            double angle = Math.atan2(x,y);
            
            double angle_rel = 0.785 + angle;
            
            r2l1 = Math.sin(angle_rel) * power;
            r1l2 = Math.cos(angle_rel) * power;
            
            while (gamepad1.left_trigger > 0.5) {
            
            double rotation = gamepad1.left_stick_x;
            
            power = Range.clip(rotation, -1, 1);
            left1.setPower(power);
            right1.setPower(-power);
            left2.setPower(power);
            right2.setPower(-power);
            
            }
            
            left1.setPower(r2l1);
            right1.setPower(r1l2);
            left2.setPower(r1l2);
            right2.setPower(r2l1);
            
            /*     
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "r2l1 (%.2f), r1l2 (%.2f)", r2l1, r1l2);
            telemetry.addData("Target:", left1_target + " " + left2_target + " " + right1_target + " " + right2_target);
            telemetry.addData("Position:", left1.getCurrentPosition() + " " + left2.getCurrentPosition() + " " + right1.getCurrentPosition() + " " + right2.getCurrentPosition());
            telemetry.addData("Angle:", Math.toDegrees(angle));
            telemetry.update();
            */
            
            pos_x_0 += gamepad2.right_stick_x;
            pos_y_0 -= gamepad2.right_stick_y;
            if (gamepad2.dpad_down == true) {
                angle_c -= 0.01;
            }
            else if (gamepad2.dpad_up == true) {
                angle_c += 0.01; 
            }
            if (gamepad2.dpad_left == true) {
                target_r -= 10;
            }
            else if (gamepad2.dpad_right == true) {
                target_r += 10; 
            }
            
            //angle_c += 0.01 * gamepad2.left_stick_y;
            
            double pos_x_1 = pos_x_0 - Math.cos(angle_c) * c;
            double pos_y_1 = pos_y_0 - Math.sin(angle_c) * c;
            
            double lenght_ab = Math.sqrt(Math.pow(pos_x_1, 2) + Math.pow(pos_y_1, 2));
            
            double angle_a_1 = Math.acos((Math.pow(a, 2) + Math.pow(lenght_ab, 2) - Math.pow(b, 2)) / (2 * a * lenght_ab));
            double angle_a_2 = Math.atan2(pos_y_1,pos_x_1);
            double angle_a = angle_a_1 + angle_a_2; 
            
            double pos_x_2 = Math.cos(angle_a) * a;
            double pos_y_2 = Math.sin(angle_a) * b;
            double pos_x_b = pos_x_1 - pos_x_2 ;
            double pos_y_b = pos_y_1 - pos_y_2;
            
            double angle_b = Math.atan2(pos_y_b, pos_x_b);
            
            target_a = (int)(count_per_rad * (angle_a - angle_a_nom));
            target_b = (int)(count_per_rad * (Math.PI - angle_a + angle_b - angle_b_offset));
            target_c = (int)(count_per_rad * (Math.PI - angle_b + angle_c - angle_c_offset));
             
            arm_1.setTargetPosition(target_a);
            arm_2.setTargetPosition(target_b);
            arm_3.setTargetPosition(target_c);
            arm_r.setTargetPosition(target_r);
            
            if (one_time_run == 0) {
            one_time_run = 1;
            arm_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_1.setPower(1);
            arm_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_2.setPower(1);
            arm_3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_3.setPower(1);
            arm_r.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_r.setPower(1);
            }
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Position:", "\n X = " + pos_x_0 + "\n Y = " + pos_y_0);
            telemetry.addData("Forward:", "\n X = " + (Math.cos(angle_a) * a + Math.cos(angle_b) * b + Math.cos(angle_c) * c) + "\n Y = " + (Math.sin(angle_a) * a + Math.sin(angle_b) * b + Math.sin(angle_c) * c));
            telemetry.addData("Angles", "\n Angle_c = " + Math.toDegrees(angle_c) + "\n Angle_b = " + Math.toDegrees(angle_b) + "\n Angle_a = " + Math.toDegrees(angle_a));
            telemetry.addData("Encoder", arm_1.getCurrentPosition() + " " + arm_2.getCurrentPosition() + " " + arm_3.getCurrentPosition());
            telemetry.addData("Targets", target_a + " " + target_b + " " + target_c);
            telemetry.update();
            sleep(10);
        }
    }
    
}

