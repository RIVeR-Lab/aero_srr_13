package instruments;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Point;
import java.awt.Toolkit;
import java.awt.image.BufferedImage;
import java.awt.image.FilteredImageSource;
import java.awt.image.ImageFilter;
import java.awt.image.ImageProducer;
import java.awt.image.RGBImageFilter;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

/**
 * An attitude indicator that visually displays the pitch and roll of the rover
 * @author joe
 *
 */
public class AttitudeIndicator extends VisualInstrument {
	double roll = 0.0;
	double pitch = 0.0;
	Image attitudeBackground, attitudeSky,attitudeMarker;
	float scale;
	
	public AttitudeIndicator() throws IOException{
		//Grab the images from resources
		attitudeBackground = makeColorTransparent(ImageIO.read(AttitudeIndicator.class.getResource("resources/AttitudeBackground.bmp")));
		attitudeMarker = makeColorTransparent(ImageIO.read(AttitudeIndicator.class.getResource("resources/AttitudeMarker.bmp")));
		attitudeSky = makeColorTransparent(ImageIO.read(AttitudeIndicator.class.getResource("resources/AttitudeGroundSky.bmp")));
		scale = 0.0f;
	}
	
	/**
	 * This method makes the color yellow transparent. It allows for images to overlap without fully covering eachother
	 * @param im The image to be made transparent
	 * @return The image with transparency
	 */
	  public Image makeColorTransparent(BufferedImage im) {
	        ImageFilter filter = new RGBImageFilter() {
	                public int markerRGB = Color.YELLOW.getRGB() | 0xFF000000;

	                public final int filterRGB(int x, int y, int rgb) {
	                        if ((rgb | 0xFF000000) == markerRGB) {
	                                return 0x00FFFFFF & rgb;
	                        } else {
	                                return rgb;
	                        }
	                }
	        };
	        ImageProducer ip = new FilteredImageSource(im.getSource(), filter);
	        return Toolkit.getDefaultToolkit().createImage(ip);
	  } 
	  
	  /**
	   * Scales an image to fit within  a given width/height
	   */
	  public void getScaledImages(){
			scale = (float)this.getWidth() / attitudeBackground.getWidth(null);
			if(scale==0.0) return;
			System.out.println("Scale" + scale);
			 attitudeSky = attitudeSky.getScaledInstance(Math.round(attitudeSky.getWidth(null)*scale),
					 Math.round(attitudeSky.getHeight(null)*scale),Image.SCALE_SMOOTH);
			 
			 attitudeBackground = attitudeBackground.getScaledInstance(Math.round(attitudeBackground.getWidth(null)*scale),
					 Math.round(attitudeBackground.getHeight(null)*scale),Image.SCALE_SMOOTH);
			 
			 attitudeMarker = attitudeMarker.getScaledInstance(Math.round(attitudeMarker.getWidth(null)*scale),
					 Math.round(attitudeMarker.getHeight(null)*scale),Image.SCALE_SMOOTH);
	  }
	  
	  public void paint(Graphics g) {
		  if(scale==0.0f) getScaledImages();
			double rollRad = roll*Math.PI/180;		
			//Center of Horizon Image
			Point ptRotation = new Point(125, 360);
			//Center of Instrument
			Point ptCenter = new Point(150,150);			
			//Modified Center of Rotation
	        Point ptModRot = new Point(ptCenter.x-2*ptRotation.x, ptCenter.y-2*ptRotation.y);
	        Graphics2D g2d = (Graphics2D)g;        
	        
	       //Set bounds
	        g2d.setClip(0, 0, (int)(attitudeBackground.getWidth(null)),(int)( attitudeBackground.getHeight(null)));
	        RotateAndTranslate(g2d, attitudeSky, rollRad, 0.0, ptModRot, 4*pitch, ptRotation,scale);
	        g2d.drawImage(attitudeMarker,
	        		Math.round((float)((0.5 * attitudeBackground.getWidth(null) - 0.5 *attitudeMarker.getWidth(null)))),
	        		Math.round((float)((0.5 * attitudeBackground.getHeight(null) - 0.5 * attitudeMarker.getHeight(null)) )),
	        	    Math.round(attitudeMarker.getWidth(null)), 
	        		Math.round(attitudeMarker.getHeight(null)),
	        		null);
	        
	        g2d.dispose();
	        g.dispose();
	        
			}
	  
	  public void setAttitude(int roll, int pitch){
		  this.roll=roll;
		  this.pitch=pitch;
		  repaint();
	  }
	  
	  public double getPitch(){
		  return pitch;
	  }
	  
	  public double getRoll(){
		  return roll;
	  }
	  
	  
}
