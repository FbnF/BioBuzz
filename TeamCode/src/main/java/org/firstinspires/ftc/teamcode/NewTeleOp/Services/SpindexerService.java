package org.firstinspires.ftc.teamcode.NewTeleOp.Services;





public class SpindexerService{

        /*the Axon MAX MK2 Servo has a max range of 350 degrees at a base. We are running the spindexer on a 5:1 ratio with this servo
        so we should be able to get around 1750 degrees of rotation at around 355 rpm.  */

        private final double ratio = 0.2;
        //gear ratio
        private final double start_pos = 0.0;
        private final double max_pose = 1.0;
        private double current_pos = 0.0;
        private final double quarter_rot = 175 * ratio;//amount of change for each quarter rotation
        private int color_counter;//how many times has it seen a ball
        private int spindex_pos = 1;//pose of spindexer relitive to intake
        private int eject_pos = 2;//Might change based on layout
        private int[] spots_filled = new int[4];//which spots are filled. Filled is represented by 1, empty by 0

        private static final int MAX_BALLS = 4;

        private boolean intake = false;
        private boolean launch = false;
        private boolean color_seen = false;
        private boolean op_color = false;
        private boolean ejecting = false;


        void spindexing() {
                if (color_seen && intake) {
                        if (color_counter < MAX_BALLS) {
                                current_pos = current_pos + quarter_rot; //rotates 90 degrees from current pose
                                spindex_pos = (spindex_pos % 4) + 1; //changes pose of intake
                                spots_filled[spindex_pos-1] = 1; // fills array spot so we know which spots arn't filled
                                color_counter++; // adds to amounts of balls seen
                                // TODO: add if statement that checks if opposing color
                                oppositeColor(); // if  opposite color it will eject it

                                if (color_counter == MAX_BALLS) {
                                        // just took the 4th ball — full, stop intaking
                                        intake = false;
                                        // TODO: trigger reverse-intake / eject logic here
                                }
                        } else {
                                // already full — a ball is still being detected while intake is (or should be) off
                                // TODO: reverse intake / eject logic here too, as a safety net
                        }
                }
        }
        private static final int NUM_POSITIONS = 4;

        void oppositeColor() {
                if (op_color && !(spindex_pos==eject_pos)) { // checks if opposite color and checks if we arn't alr at eject pose
                        int steps = ((eject_pos - spindex_pos) % NUM_POSITIONS + NUM_POSITIONS) % NUM_POSITIONS; //how  many 90 degree rotations are needed to get ball to ejection pose
                        current_pos += steps * quarter_rot;
                        spindex_pos = eject_pos;// sets it to correct pose
                        spots_filled[spindex_pos - 1] = 0; //empties spot in array

                } else if (op_color) {
                        ejecting = true;
                }
                if (ejecting) {
                        // TODO: slow intake, flash light, and eject the ball
                        ejecting = false;
                }

        }
//TODO launching logic


}

