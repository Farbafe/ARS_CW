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
	private static final int WIDTH = 160;
    private static final int HEIGHT = 120;
    //private static final int NUM_PIXELS = WIDTH * HEIGHT;
    
    private static int [][] luminanceFrame = new int [HEIGHT][WIDTH];
    private static int threshold = 70;
    private static int intensityThreshold = 522; // 0.5 * 0.66 * 0.33 * 0.5 * 0.5 * 160 * 120 (66% of one quadrant when every other pixel in x and y directions is sampled with a frame of 160x120)
    private static EV3LargeRegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.D);
	private static EV3LargeRegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
	enum LightIntensity {
		LIGHT,
		DARK
	}
     
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
        LightIntensity leftLightIntensity, centreLightIntensity, rightLightIntensity;
        int leftTemp, centreTemp, rightTemp;
        
        float speed = (float) (motorLeft.getMaxSpeed() * 0.8);// .2 temp
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
            // Display the frame
//            dispFrame(); // temp
            
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

            for (int y = HEIGHT / 2; y < HEIGHT; y = y + 2) {
            	for (int x = 0; x < WIDTH / 3; x = x + 2) {
            		if (luminanceFrame[y][x] <= threshold) {
            			++leftTemp;
            		}
            	}
            	for (int x = WIDTH / 3; x < WIDTH / 3 * 2; x = x + 2) {
            		if (luminanceFrame[y][x] <= threshold) {
            			++centreTemp;
            		}
            	}
            	for (int x = WIDTH / 3 * 2; x < WIDTH; x = x + 2) {
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
            
            if (centreLightIntensity == LightIntensity.LIGHT) {
            	motorLeft.forward();
            	motorRight.forward();
            }
            else if (leftLightIntensity == LightIntensity.LIGHT && rightLightIntensity == LightIntensity.DARK) {
            	motorLeft.flt();
            	motorRight.forward();
            }
            else if (leftLightIntensity == LightIntensity.DARK && rightLightIntensity == LightIntensity.LIGHT) {
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
        video.close();
    }
     
    /*private static void play(File file) {
        long now = System.currentTimeMillis();
         
        if (now - lastPlay > 2000) {
            System.out.println("Playing " + file.getName());
            Sound.playSample(file);
            lastPlay = now;
        }
    }*/
    
    
//    public static void extractLuminanceValues(byte [] frame) {
//    	int x,y;
//    	int quadWidth = 4*WIDTH; // y1: pos 0; u: pos 1; y2: pos 2; v: pos 3.
//    	for(int i=0;i<frame.length;i+=4) {
//    		x = (i % quadWidth)/4;
//    		y = i / quadWidth;
//    		luminanceFrame[y][x] = frame[i];
//    	}	
//    }
    
    // DO: Improve this possibly by combining with chrominance values.
/*    public static void extractLuminanceValues(byte [] frame) {
    	int x,y;
    	int doubleWidth = 2*WIDTH; // y1: pos 0; u: pos 1; y2: pos 2; v: pos 3.
    	int frameLength = frame.length;
    	for(int i=0;i<frameLength;i+=2) {
    		x = (i / 2) % WIDTH;
    		y = i / doubleWidth;
    		luminanceFrame[y][x] = frame[i];
    	}
    }*/
    
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
      
//    private static int convertYUVtoARGB(int y, int u, int v) {
//        int c = y - 16;
//        int d = u - 128;
//        int e = v - 128;
//        int r = (298*c+409*e+128)/256;
//        int g = (298*c-100*d-208*e+128)/256;
//        int b = (298*c+516*d+128)/256;
//        r = r>255? 255 : r<0 ? 0 : r;
//        g = g>255? 255 : g<0 ? 0 : g;
//        b = b>255? 255 : b<0 ? 0 : b;
//        return 0xff000000 | (r<<16) | (g<<8) | b;
//    }

}
