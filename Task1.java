import java.io.IOException;
import java.util.Stack;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.video.Video;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.Move;
import lejos.utility.Delay;
public class Task1 {
    private static final int WIDTH = 160;
    private static final int HEIGHT = 120;
    //private static final int NUM_PIXELS = WIDTH * HEIGHT;

    private static int [][] luminanceFrame = new int [HEIGHT][WIDTH];
    private static int threshold = 90;
    private static RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
    private static RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.D);
    private static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
    private static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
    
    enum RobotMovement {
        LEFT,
        STRAIGHT,
        RIGHT,
        REVERSE,
        TURN_AROUND,
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
        int blackTotal, blackLeft, blackCentre, blackRight, blackLeftWide, blackRightWide, 
            greyLoopCount = 0, rightJunctionCount = 0, leftJunctionCount = 0;
        colorSensor.setCurrentMode("Ambient");
        gyroSensor.setCurrentMode("Angle");
        float[] colorSample = new float[colorSensor.sampleSize()];
        float[] gyroSample = new float[gyroSensor.sampleSize()];
        Movement[] movements = new Movement[50]; // ArrayList is preferable but Array is simpler
        Movement movementBuffer;
        RobotMovement previousMovement;
        
        while (Button.ESCAPE.isUp()) {
            Delay.msDelay(500);
            goStraight();
            while (motorLeft.getTachoCount() < 1875) {
                
            }
            stop();
            Movement movement1 = new Movement(1875, 1875, 0);
            Delay.msDelay(500);
            motorLeft.rotate(-140, true);
            motorRight.rotate(140, false);
            gyroSensor.fetchSample(gyroSample, 0);
            Delay.msDelay(500);
            resetTachoCounts();
            goStraight();
            while (motorLeft.getTachoCount() < 1875) {
                
            }
            stop();
            int angleComplement = (int) (180 - gyroSample[0]);
            Movement movement2 = new Movement(1875, 1875, gyroSample[0], movement1);
            Movement movement3 = movement2.sum(movement1);
            Delay.msDelay(250);
            gyroSensor.reset();
            Delay.msDelay(250);
            do {
                motorLeft.rotate(-12, true);
                motorRight.rotate(12, false);
                gyroSensor.fetchSample(gyroSample, 0);                
            } while (180 + movement3.getBearing() - angleComplement + 180 >= gyroSample[0]);            
            resetTachoCounts();
            goStraight();
            int tachoCount = movement3.getLeftTachoCount();
            while (motorLeft.getTachoCount() < tachoCount) {
                
            }
            stop();
//            Delay.msDelay(1500);
//            Movement movement4 = movement3.reverseMovement();
            if (true) {
                break;
            }
            // end test
            
            
            gyroSensor.fetchSample(gyroSample, 0);
            --greyLoopCount;
            if (greyLoopCount < 0) {
                colorSensor.fetchSample(colorSample, 0);
                if (colorSample[0] < 0.37 && colorSample[0] > 0.34) {
                    Delay.msDelay(40);
                    colorSensor.fetchSample(colorSample, 0);
                    if (colorSample[0] < 0.37 && colorSample[0] > 0.34) {
                        previousMovement = robotMovement;
                        goStraight();
                        Delay.msDelay(100);
                        stop();
                        colorSensor.fetchSample(colorSample, 0);
                        if (colorSample[0] < 0.37 && colorSample[0] > 0.34) {
                            Delay.msDelay(1000);
                            Sound.beep();
                            greyLoopCount = 100;
                            goBackward();
                            Delay.msDelay(300);                           
                            if (previousMovement == RobotMovement.STRAIGHT) {
                                goStraight();
                            }
                            else if (previousMovement == RobotMovement.LEFT) {
                                turnLeft();
                            }
                            else if (previousMovement == RobotMovement.RIGHT) {
                                turnRight();
                            }
                        }
                    }
                }
            }
            
            blackTotal = 0;
            blackLeftWide = 0;
            blackLeft = 0;
            blackCentre = 0;
            blackRight = 0;
            blackRightWide = 0;
            video.grabFrame(frame);

            // Create a frame of luminance values
            extractLuminanceValues(frame);               
            
            for (int y = 22; y <= 57; y += 2) {
                for (int x = 5; x <= 7; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackLeftWide;
                    }
                }
                for (int x = WIDTH / 16 * 5; x <= WIDTH / 16 * 5 + 2; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackLeft;
                    }
                }
                for (int x = WIDTH / 2 - 1; x <= WIDTH / 2 + 1; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackCentre;
                    }
                }
                for (int x = WIDTH / 16 * 11 - 2; x <= WIDTH / 16 * 11; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackRight;
                    }
                }
                for (int x = 154; x <= 156; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackRightWide;
                    }
                }
            }

            if ((robotMovement == RobotMovement.LEFT || robotMovement == RobotMovement.RIGHT) && blackCentre >= 16) { // changing from 9 to 27 hopes to not need a correction after turning but it may not work for tighter turns so check and see.
                continue; // i.e keep turning until the centre is less black
            }
            
            blackTotal = blackLeftWide + blackLeft + blackCentre + blackRight + blackRightWide;
            if (blackTotal >= 216) {
                turnAround();
                continue;
            }
            
//            if (blackLeftWide <= 6 && blackRightWide <= 6) {
//                Button.LEDPattern(1); // green
//            }
//            else if (blackLeftWide <= 6) {
//                Button.LEDPattern(2); // red
//            }
//            else if (blackRightWide <= 6) {
//                Button.LEDPattern(3); // orange
//            }
//            else {
//                Button.LEDPattern(0); // no led
//            }
            
            if (blackLeftWide <= 16) {
                if (leftJunctionCount < 0) {
                    leftJunctionCount = 0;
                }
                ++leftJunctionCount;
            }
            else {
                --leftJunctionCount;           
            }
            if (blackRightWide <= 16) {
                if (rightJunctionCount < 0) {
                    rightJunctionCount = 0;
                }
                ++rightJunctionCount;
            }
            else {
                --rightJunctionCount;           
            }
            
            if (leftJunctionCount > 1) {
                if (rightJunctionCount > 1) {
                    // create left and right junction
                }
                else {
                    // create left junction
                }
            }
            if (rightJunctionCount > 1) {
                if (leftJunctionCount > 1) {
                    // create left and right junction
                }
                else {
                    // create right junction
                }
                // reset counters to 0
            } // todo junctions
                        
            if (blackCentre <= 6) {
                goStraight();                
            }
            else if (blackRight <= 16) {
                if (robotMovement != RobotMovement.RIGHT) {
                    turnRight();
                    // save current movement
                    gyroSensor.fetchSample(gyroSample, 0);
                    movementBuffer = new Movement(motorLeft.getTachoCount(), motorRight.getTachoCount(), gyroSample[0]);
                }
            }
            else if (blackLeft <= 16) {
                if (robotMovement != RobotMovement.LEFT) {
                    turnLeft();
                }
            }            
            else {
                turnAround();
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
        robotMovement = RobotMovement.NONE;
    }    

    public static void turnRight() {
        motorLeft.setSpeed(180);
        motorRight.setSpeed(45);
        motorLeft.forward();
        motorRight.backward();
        robotMovement = RobotMovement.RIGHT;
    }

    public static void turnLeft() {
        motorLeft.setSpeed(45);
        motorRight.setSpeed(180);
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
        motorLeft.setSpeed(180);
        motorRight.setSpeed(180);
        motorRight.backward();
        motorLeft.backward();
        robotMovement = RobotMovement.REVERSE;
    }

    public static void turnAround() {
        goBackward();
        Delay.msDelay(80);
        motorLeft.setSpeed(180);
        motorRight.setSpeed(180);
        motorRight.rotate(369, true);
        motorLeft.rotate(-369, false);
        Delay.msDelay(100); // let camera adjust
        goStraight();
        Delay.msDelay(40);
//        robotMovement = RobotMovement.TURN_AROUND;
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
    
    public static void resetTachoCounts() {
        motorLeft.resetTachoCount();
        motorRight.resetTachoCount();
    }
    
    enum JunctionType {
        LEFT_RIGHT,
        LEFT,
        RIGHT
    }
    // should it backtrack after second junction detection? or do the whole maze and then decide?
    
    class Junction {
        private int movementIndex;
        private JunctionType junctionType;
        boolean isAllOptionsTravelled = false;
        
        public Junction(int movementIndex, JunctionType junctionType) {
            this.movementIndex = movementIndex;
            this.junctionType = junctionType;
        }
        
        public int getMovementIndex() {
            return this.movementIndex;
        }
        
        public JunctionType getJunctionType() {
            return this.junctionType;
        }
    }
}

class Movement {
    //
    // Any turning event should create a new movement
    // unless the turning is after less than 20 tacho counts?
    // this ensures that each vector is a straight line
    // as for turning vectors... we ignore for now.
    // there is no need to know a turn
    // we just need to know the straight paths
    
    //
    // this means that we will have gaps in our movement
    // but it is fine?
    // what we care about is the ifStraight when turnin then make movement and reset tacho
    //
    
    // this whole class just needs starting bearing, ending bearing and magnitude
    // can we calculate radius of curvature if we have left and right tachos? i think we can
    public Movement position; // the movement that absolutely describes this movement
    private int leftTacho = 0;
    private int rightTacho = 0;
    private int distance;
    private int bearing = 0; // movement started with this bearing (should be changed to start bearing and endbearing?)
    private int parentMovementIndex = -1; // this can be used to backtrack and go to a 
    // previous movement. We can also check if we've reached the base by we reach the
    // same place (either x or y plane) to a movement that has parentMovementIndex == -1?
    public Movement parentMovement;
    
    public Movement(int leftTacho, int rightTacho, float bearing) {
        this.leftTacho = leftTacho;
        this.rightTacho = rightTacho;
        this.bearing = (int) ((bearing + 360) % 360);
        this.distance = (int) ((this.leftTacho + this.rightTacho) * 2 * 3.14159 * 1.52 / 360); // in cm
    }
    
    public Movement(int leftTacho, int rightTacho, float bearing, Movement parentMovement) {
        this.leftTacho = leftTacho;
        this.rightTacho = rightTacho;
        this.bearing = (int) ((bearing + 360) % 360);
        this.distance = (int) ((this.leftTacho + this.rightTacho) * 2 * 3.14159 * 1.52 / 360); // in cm
        this.parentMovement = parentMovement;
    }
    
    public Movement(int leftTacho, int rightTacho, float bearing, int parentMovementIndex) {
        this.leftTacho = leftTacho;
        this.rightTacho = rightTacho;
        this.bearing = (int) ((bearing + 360) % 360);
        this.distance = (int) ((this.leftTacho + this.rightTacho) * 2 * 3.14159 * 1.52 / 360); // in cm
        this.parentMovementIndex = parentMovementIndex;
    }
    
    public void setParentMovementIndex(int index) {
        this.parentMovementIndex = index;
    }
    
    public void setBearing(int bearing) {
        this.bearing = bearing;
    }
    
    public int getBearing() {
        return this.bearing;
    }
    
    public int getLeftTachoCount() {
        return this.leftTacho;
    }
    
    public float getRightTachoCount() {
        return this.rightTacho;
    }
    
    public int getParentMovementIndex() {
        return this.parentMovementIndex;
    }
    
    public float getMotorLeftSpeed() {
        return this.leftTacho / this.rightTacho;
    }
    
    public float getMotorRightSpeed() {
        return this.rightTacho / this.leftTacho;
    }
    
    public Movement reverseMovement() {
        return new Movement(this.rightTacho, this.leftTacho, 360 - this.bearing);
    }
    
    public Movement reverseMovement(Movement movement) {
        return new Movement(movement.rightTacho, movement.leftTacho, 360 - movement.bearing);
    }
    
    public boolean isSameMovement(Movement movement, boolean isReversed) {
        if (isReversed) {
            if ((Math.abs(movement.leftTacho - this.rightTacho) < 100) || (movement.leftTacho >= this.rightTacho * 0.9 && movement.leftTacho <= this.rightTacho * 1.1) && ((Math.abs(movement.rightTacho - this.leftTacho) < 100) || (movement.rightTacho >= this.leftTacho * 0.9 && movement.rightTacho <= this.leftTacho * 1.1))) {
                return false;
            }
        }
        else {
            if ((Math.abs(movement.leftTacho - this.leftTacho) < 100) || (movement.leftTacho >= this.leftTacho * 0.9 && movement.leftTacho <= this.leftTacho * 1.1) && ((Math.abs(movement.rightTacho - this.rightTacho) < 100) || (movement.rightTacho >= this.rightTacho * 0.9 && movement.rightTacho <= this.rightTacho * 1.1))) {
                return true;
            }
        }
        return false;
    }
    
    public int splitDistanceIntoEqualTachoCounts() {
        int tachoCount = (int) (this.distance * 360 / 2 / 3.14159 / 1.52 / 2);
        return tachoCount;
    }
    
    public Movement sum(Movement movement) {
        int tachoCount = this.splitDistanceIntoEqualTachoCounts() + movement.splitDistanceIntoEqualTachoCounts();
        int bearing = (this.bearing + movement.bearing) % 360;
        Movement sum = new Movement(tachoCount, tachoCount, bearing);
        return sum;
    }
    
    public Movement getAbsolutePosition() {
        // returns the sum of vectors of movements using this.parentMovementIndex and recursion
        if (this.parentMovementIndex == -1) {
            int tachoCount = splitDistanceIntoEqualTachoCounts();
            this.position = new Movement(tachoCount, tachoCount, this.bearing);
            return this.position;
        }
        else {
            int tachoCount = splitDistanceIntoEqualTachoCounts();
            this.position = new Movement(tachoCount, tachoCount, this.bearing);
            return this.sum(this.parentMovement.getAbsolutePosition());
        }
    }
    
    public boolean isSamePosition(Movement movement) {
        this.position = this.getAbsolutePosition();
        movement.position = movement.getAbsolutePosition();
        if ((Math.abs(this.position.leftTacho - movement.position.leftTacho) < 500) && (Math.abs(this.position.rightTacho - movement.position.rightTacho) < 500) && (Math.abs(this.position.bearing - movement.position.bearing) < 72)) {
            return true;
        }
        return false;
    }
}
