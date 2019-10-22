import java.io.IOException;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.video.Video;
import lejos.utility.Delay;

public class CamDispLCD1 {
	private static final boolean DISPLAY_ENABLED = true;
	private static final boolean MOVEMENT_ENABLED = false;
	
	private static final int WIDTH = 160;
    private static final int HEIGHT = 120;
    //private static final int NUM_PIXELS = WIDTH * HEIGHT;
    
    private static int [][] luminanceFrame = new int [HEIGHT][WIDTH];
    private static int threshold = 70;
    private static int intensityThreshold = 528; // 66% of the pixels used for each section
    private static EV3LargeRegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.D);
	private static EV3LargeRegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
	
	enum LightIntensity {
		LIGHT,
		DARK
	}
	
	private static LightIntensity leftLightIntensity;
	private static LightIntensity centreLightIntensity;
	private static LightIntensity rightLightIntensity;
    
    public CamDispLCD1() {
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
        int leftTemp, centreTemp, rightTemp;
        
        float speed = (float) (motorLeft.getMaxSpeed() * 0.8);
        motorLeft.setSpeed(speed);
        motorRight.setSpeed(speed);
         
        while(Button.ESCAPE.isUp()) {
        	leftTemp = 0;
        	centreTemp = 0;
        	rightTemp = 0;
        	
            video.grabFrame(frame);
            // y1: pos 0; u: pos 1; y2: pos 2; v: pos 3.
            // Create a frame of luminance values
            extractLuminanceValues(frame);
            
            for (int y = HEIGHT / 2; y < HEIGHT; y = y + 2) {
            	for (int x = 0; x < WIDTH / 3; x = x + 2) {
            		if (luminanceFrame[y][x] <= threshold) {
            			++leftTemp;
            		}
            	}
            	for (int x = WIDTH / 3; x < (WIDTH / 3) * 2; x = x + 2) {
            		if (luminanceFrame[y][x] <= threshold) {
            			++centreTemp;
            		}
            	}
            	for (int x = (WIDTH / 3) * 2; x < WIDTH; x = x + 2) {
            		if (luminanceFrame[y][x] <= threshold) {
            			++rightTemp;
            		}
            	}
            }
            
            if (leftTemp > intensityThreshold) {
            	leftLightIntensity = LightIntensity.LIGHT;
            }
            else {
            	leftLightIntensity = LightIntensity.DARK;
            }
            
            if (centreTemp > intensityThreshold) {
            	centreLightIntensity = LightIntensity.LIGHT;
            }
            else {
            	centreLightIntensity = LightIntensity.DARK;
            }
            
            if (rightTemp > intensityThreshold) {
            	rightLightIntensity = LightIntensity.LIGHT;
            }
            else {
            	rightLightIntensity = LightIntensity.DARK;
            }
            
            // Display the frame
            if (DISPLAY_ENABLED) {
            	dispSectionedFrame();
            }
//            dispFrame();
            
            // todo use readButtons for better performance
//            if (Button.UP.isDown()) {
//                threshold +=1;
//                if (threshold > 255)
//                    threshold = 255;
//            }
//            else if (Button.DOWN.isDown()) { 
//            	threshold -=1;
//                if (threshold < 0)
//                    threshold = 0;
//            }
            
//            if (Button.RIGHT.isDown()) {
//                intensityThreshold += 1;
//                if (intensityThreshold > 255)
//                    intensityThreshold = 255;
//            }
//            else if (Button.LEFT.isDown()) { 
//            	intensityThreshold -= 1;
//                if (intensityThreshold < 0)
//                    intensityThreshold = 0;
//            }
            
            if (MOVEMENT_ENABLED) {
	            if (centreLightIntensity == LightIntensity.LIGHT) {
	            	Delay.msDelay(120);
	            	motorLeft.forward();
	            	motorRight.forward();
	            }
	            else if (leftLightIntensity == LightIntensity.LIGHT && rightLightIntensity == LightIntensity.DARK) {
	            	Delay.msDelay(120);
	            	motorLeft.flt();
	            	motorRight.forward();
	            }
	            else if (leftLightIntensity == LightIntensity.DARK && rightLightIntensity == LightIntensity.LIGHT) {
	            	Delay.msDelay(120);
	            	motorLeft.forward();
	            	motorRight.flt();
	            	// do not float - instead use 30% of the speed for the flt and 100% of the non-flt
	            }
	            else {
	//            	motorLeft.backward();
	//            	motorRight.backward();
	//            	Delay.msDelay(120);
	//            	motorLeft.flt();
	//            	motorRight.flt();
	//            	Delay.msDelay(120);
	            	
	            	// todo
	            	// rotate left and check ahead for white (needs white for at least 10 cm?)
	            	// if failed then
	            	// rotate right to current bearing and check ahead for white
	            	// if failed then
	            	// rotate right of then position and traverse (i.e robot reached dead-end)
	                // because the square is quite white at the edges, the robot should have a clause to not go for x tacho counts in a straiht line (otherwise it may miss the previous exit points and just go around in a square)
	            }
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
    
    public static void dispSectionedFrame() {
    	if (leftLightIntensity == LightIntensity.LIGHT) {
    		setDisp(0, 0, HEIGHT, WIDTH/3, 0);
    	}
    	else {
    		setDisp(0, 0, HEIGHT, WIDTH/3, 1);
    	}
    	if (centreLightIntensity == LightIntensity.LIGHT) {
    		setDisp(0, WIDTH/3, HEIGHT, (WIDTH/3)*2, 0);
    	}
    	else {
    		setDisp(0, WIDTH/3, HEIGHT, (WIDTH/3)*2, 1);
    	}
    	if (rightLightIntensity == LightIntensity.LIGHT) {
    		setDisp(0, (WIDTH/3)*2, HEIGHT, WIDTH, 0);
    	}
    	else {
    		setDisp(0, (WIDTH/3)*2, HEIGHT, WIDTH, 1);
    	}
    }
    
    public static void setDisp(int y0, int x0, int height, int width, int color) {
    	for (int y = y0; y < height; ++y) {
    		for (int x = x0; x < width; ++x) {
    			LCD.setPixel(x, y, color);
    		}
    	}
    }
}
