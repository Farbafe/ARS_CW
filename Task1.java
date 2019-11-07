import java.io.IOException;
import java.util.Stack;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.video.Video;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
public class Task1 {
    private static boolean isGreyDetectorMode = true;

    private static final int WIDTH = 160;
    private static final int HEIGHT = 120;
    //private static final int NUM_PIXELS = WIDTH * HEIGHT;
    
    private static int [][] luminanceFrame = new int [HEIGHT][WIDTH];
    private static int threshold = 90;
    private static RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
    private static RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.D);
    enum RobotMovement {
        LEFT,
        STRAIGHT,
        RIGHT,
        REVERSE,
        TURN,
        RIGHT_REVERSE,
        LEFT_REVERSE,
        NONE
    }
    private static RobotMovement robotMovement = RobotMovement.NONE;

    public Task1() {
        // Initialize luminance frame
        for (int x=0; x<WIDTH; x += 1) {
            for (int y=0; y<HEIGHT; y += 1) {
                luminanceFrame[y][x] = 0;
            }
        }
    }
        
    public static void main(String[] args) throws IOException  {
         
        EV3 ev3 = (EV3) BrickFinder.getLocal();
        Video video = ev3.getVideo();
        video.open(WIDTH, HEIGHT);
        byte[] frame = video.createFrame();
        int blackLeft, blackRight, greyArea;
        boolean isAllBlack = false;
            
        while (Button.ESCAPE.isUp()) {
            blackLeft = 0;
            blackRight = 0;
            greyArea = 0;
            video.grabFrame(frame);
                       
            // Create a frame of luminance values
            extractLuminanceValues(frame);   
            
            // Adjust threshold?
            if (Button.UP.isDown()) {
                threshold +=5;
                if (threshold > 255)
                    threshold = 255;
            }
            else if (Button.DOWN.isDown()) { // if down is pushed down and hold, does threshold keep changing? - need to check, maybe that's why we were getting weird results 
                threshold -=5;
                if (threshold < 0)
                    threshold = 0;
            }
            
            if (Button.RIGHT.isDown()) {
                isGreyDetectorMode = !isGreyDetectorMode;
                stop();
                Delay.msDelay(500);
            }
            
            if (isGreyDetectorMode) {
                if (isGrey(true)) {
                    Sound.beep();
                }
                
//                for (int y = HEIGHT / 2 - 5; y < HEIGHT / 2 + 5; ++y) {
//                    for (int x = WIDTH / 2 - 5; x < WIDTH / 2 + 5; ++x) {
//                        if (luminanceFrame[y][x] < threshold && luminanceFrame[y][x] > (threshold - 20)) {
//                            ++greyArea;
//                            if (Button.LEFT.isDown()) {
//                                System.out.println("");
//                                System.out.println("");
//                                System.out.println("");
//                                System.out.println("");
//                                System.out.println("");
//                                System.out.println("   greyArea:  " + greyArea);
//                                System.out.println("   threshold: " + threshold);
//                            }
//                            else {
//                                LCD.setPixel(x + 10, y, 1);
//                            }
//                        }
//                        else {
//                            --greyArea;
//                            if (Button.LEFT.isDown()) {
//                                System.out.println("");
//                                System.out.println("");
//                                System.out.println("");
//                                System.out.println("");
//                                System.out.println("");
//                                System.out.println("   greyArea:  " + greyArea);
//                                System.out.println("   threshold: " + threshold);
//                            }
//                            else {
//                                LCD.setPixel(x + 10, y, 0);
//                            }
//                        }
//                    }
//                }
//                if (greyArea > 0) {
//                    Sound.beep();
//                }
                
//                setDisp(0, WIDTH-10, 10, WIDTH, greyArea > 0 ? 1 : 0);
                continue;
            }
            else {
//                dispFrame(); // todo we should comment dispFrame for Task1 (to reduce computation needed)
            }
            
            for (int y = 18; y <= HEIGHT / 2 + 18; y += 2) {
                if (isAllBlack) {
                    for (int x = WIDTH / 16 * 5; x <= WIDTH / 16 * 5 + 2; ++x) {
                        if (luminanceFrame[y][x] < threshold) {
                            ++blackLeft;
                        }
                    }
                    for (int x = WIDTH / 16 * 11 - 2; x <= WIDTH / 16 * 11; ++x) {
                        if (luminanceFrame[y][x] < threshold) {
                            ++blackRight;
                        }
                    }
                }
                else {
                    for (int x = WIDTH / 16 * 7; x <= WIDTH / 16 * 7 + 2; ++x) {
                        if (luminanceFrame[y][x] < threshold) {
                            ++blackLeft;
                        }
                    }
                    for (int x = WIDTH / 16 * 9 - 2; x <= WIDTH / 16 * 9; ++x) {
                        if (luminanceFrame[y][x] < threshold) {
                            ++blackRight;
                        }
                    }
                }
            }
            
            //Four Conditions
            if (blackLeft >= blackRight * 0.9 && blackLeft <= blackRight * 1.1 && (blackLeft >= 81 || blackRight >= 81)) {
                if (robotMovement != RobotMovement.REVERSE) {
                    goBackward();
                    Delay.msDelay(40);
                    stop();
                    isAllBlack = true; 
                    // the idea is that by reversing and checking a wider line
                    // in the next while loop, the robot can detect some difference 
                    // in the darkness on the right and left and act differently.
                    // However, if there is still no measured difference then
                    // the robot is first going to check for an even wider line 
                    // and then if that fails it explicitly checks right and left
                    // views of the robot and then decide what to do.
                }
                else {
                    blackLeft = 0;
                    blackRight = 0;
                    // the for loop and if statement below checks for little
                    // darkness at wider lines. If true the robot will turn
                    // around and the while loop continues.
                    goBackward();
                    Delay.msDelay(90);
                    stop();
                    for (int y = 18; y <= HEIGHT / 2 + 18; y += 2) {
                        for (int x = WIDTH / 16 * 5; x <= WIDTH / 16 * 5 + 2; ++x) {
                            if (luminanceFrame[y][x] < threshold) {
                                ++blackLeft;
                            }
                        }
                        for (int x = WIDTH / 16 * 11 - 2; x <= WIDTH / 16 * 11; ++x) {
                            if (luminanceFrame[y][x] < threshold) {
                                ++blackRight;
                            }
                        }
                    }
                    if (Math.abs(blackRight - blackLeft) <= 10 && (blackLeft <= 10 || blackRight <= 10)) {
                        turnAround(); // this is a blocking function
                        Sound.beep();
                        isAllBlack = false;
                        Delay.msDelay(50);
                    }
                    else {
                        // now the robot has failed multiple tests, first it failed to 
                        // reverse and check for wider lines to know what to do, then it
                        // failed to check for even wider lines. Now we explicitly turn
                        // right and check and if that fails, we go back to original position
                        // turn left and decide what to do before continuing the while loop.
                        if (robotMovement != RobotMovement.RIGHT) {
                            turnRight();
                            Delay.msDelay(70);
                            stop();
                        }
                        else if (robotMovement != RobotMovement.LEFT) {
                            turnRightReversed(); // this brings the robot back to original position
                            Delay.msDelay(70);
                            turnLeft();
                            Delay.msDelay(70);
                            stop();
                        }
                        else {
                            Sound.beep();
                            turnAround();
                            Delay.msDelay(50);
                        }
                    }
                }
            }
            else if ((blackLeft >= blackRight * 0.9 && blackLeft <= blackRight * 1.1) || (Math.abs(blackRight - blackLeft) <= 10)) {
                if (robotMovement != RobotMovement.STRAIGHT) {
                    if (robotMovement == RobotMovement.LEFT) {
                        // added the goBackward for a small distance
                        // because turnLeft and turnRight always move
                        // the robot a little forward, so hopefully
                        // this backwards movement will offset that a bit
                        goBackward();
                        Delay.msDelay(30);
                        turnRight();
                        Delay.msDelay(75);
                    }
                    else {
                        // added the goBackward for a small distance
                        // because turnLeft and turnRight always move
                        // the robot a little forward, so hopefully
                        // this backwards movement will offset that a bit
                        goBackward();
                        Delay.msDelay(30);
                        turnLeft();
                        Delay.msDelay(75);
                    }
                    goStraight();
                }
                isAllBlack = false;
            }
            else if (blackLeft > blackRight) {
                if (robotMovement != RobotMovement.RIGHT) {
                    Delay.msDelay(50);
                    turnRight();
                }
                isAllBlack = false;
            }
            else if (blackLeft < blackRight) {
                if (robotMovement != RobotMovement.LEFT) {
                    Delay.msDelay(50);
                    turnLeft();
                }
                isAllBlack = false;
            }                
        }
        video.close();
    }
    
    
    public static void extractLuminanceValues(byte [] frame) {
        int x,y;
        int doubleWidth = 2*WIDTH; // y1: pos 0; u: pos 1; y2: pos 2; v: pos 3.
        int frameLength = frame.length;
        for(int i=0;i<frameLength;i+=2) {
            x = (i / 2) % WIDTH;
            y = i / doubleWidth;
            //luminanceFrame[y][x] = 2*Math.abs((int) frame[i]);
            luminanceFrame[y][x] = frame[i] & 0xFF;           
        }
    }
    
    
    public static void dispFrame() {
        for (int y=0; y<HEIGHT; y++) {
            for (int x=0; x<WIDTH; x++) {
                if (luminanceFrame[y][x] <= threshold) {
                    LCD.setPixel(x, y, 1);
                }
                else {
                    LCD.setPixel(x, y, 0);
                }    
            }
        }
    }
    
    public static void setDisp(int y0, int x0, int height, int width, int color) {
        for (int y = y0; y < height; ++y) {
            for (int x = x0; x < width; ++x) {
                LCD.setPixel(x, y, color);
            }
        }
    }
        
    public static void stop() {
        motorRight.stop(true);
        motorLeft.stop(true);
    }    
    
    public static void turnRight() {
        motorLeft.setSpeed(180);
        motorRight.setSpeed(45);
        motorLeft.forward();
        motorRight.backward();
        robotMovement = RobotMovement.RIGHT;
    }
       
    public static void turnLeft() {
        motorLeft.setSpeed(45); // todo if needed we can multiply by normalisedTimeTurning which will be a variable between 0 and 1
        motorRight.setSpeed(180); // try 140, it will turn slower so it'll give the robot more time to react after turning
        motorLeft.backward();
        motorRight.forward();
        robotMovement = RobotMovement.LEFT;
    }
    
    public static void goStraight() {
        motorLeft.setSpeed(180);
        motorRight.setSpeed(180);
        motorRight.forward();
        motorLeft.forward();
        robotMovement = RobotMovement.STRAIGHT;
    }
    
    public static void goBackward() {
        motorLeft.setSpeed(150);
        motorRight.setSpeed(150);
        motorRight.backward();
        motorLeft.backward();
        robotMovement = RobotMovement.REVERSE;
    }
    
    public static void turnAround() {
        motorLeft.setSpeed(360);
        motorRight.setSpeed(360);
        motorRight.rotate(170, false);
        motorLeft.rotate(-170, false);
        robotMovement = RobotMovement.TURN;
    }
    
    public static void turnRightReversed() {
        motorLeft.setSpeed(180);
        motorRight.setSpeed(45);
        motorLeft.backward();
        motorRight.forward();
        robotMovement = RobotMovement.RIGHT_REVERSE;
    }
    
    public static void turnLeftReversed() {
        motorLeft.setSpeed(45); 
        motorRight.setSpeed(180);
        motorLeft.forward();
        motorRight.backward();
        robotMovement = RobotMovement.LEFT_REVERSE;
    }
    
    //check a 20*5 center block if it is grey
    public static boolean isGrey(boolean outputValues) {
        int pixelTotalCount = 0;
        int totalLuminance = 0;
        
        // average white block luminance
        // need further testing 
        int WHITE = 150;
        
        for (int y = 18; y <= HEIGHT / 6 + 18; ++y) {
            for (int x = WIDTH / 2 - 2; x <= WIDTH / 2 + 2; ++x) {
                totalLuminance += luminanceFrame[y][x];
                ++pixelTotalCount;
            }
        }
        
        int avgLuminance = totalLuminance / pixelTotalCount;
        
        if (outputValues) {
            // testing part
            System.out.println("");
            System.out.println("");
            System.out.println("");
            System.out.println("");
            System.out.println("");
            System.out.println("");
            System.out.println(" AVGLuminance: " + avgLuminance);
            System.out.println("   threshold: " + threshold);
        }
        
        // if the block average luminance is greater than threshold
        // and lesser than average white area luminance
        if (avgLuminance > threshold && avgLuminance < WHITE) {
            return true;
        }
        else {
            return false;
        }
    }
}
