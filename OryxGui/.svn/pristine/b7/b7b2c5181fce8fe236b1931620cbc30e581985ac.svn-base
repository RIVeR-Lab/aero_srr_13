package instruments;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Point;
import java.awt.Polygon;
import java.io.File;
import java.io.IOException;
import java.math.BigDecimal;
import java.text.DecimalFormat;

import javax.imageio.ImageIO;

/**
 * This class displays a topdown view of the rover, indicating the direction of the camera
 * as well as the height of each wheel
 * @author joe
 *
 */
public class CameraDirectionInstrument extends VisualInstrument {
	Image robot;
	double flHeight=0,frHeight=0,blHeight=0,brHeight=0;
	double pan=0,tilt=0;
	
	double fieldOfView = 50.0;
	int viewLength = 220;
	
	public CameraDirectionInstrument() throws IOException{
		robot = ImageIO.read(CameraDirectionInstrument.class.getResource("resources/NewTop.JPG"));
		
	}
	
	public void paint(Graphics g){
		Graphics2D g2d = (Graphics2D)g;
		float scale = (float)this.getWidth()/robot.getWidth(null);
		g2d.drawImage(robot,0,0,Math.round(robot.getWidth(null)*scale),Math.round(robot.getHeight(null)*scale),null);
		
		int xStart = 114;
		int yStart = 200;
		
		double panRad = Math.toRadians(pan);
		double fovRad = Math.toRadians(fieldOfView);
		
		//Do some math to figure out the triangle formed by the cameras field of view
		int xLeft= (int) (xStart+Math.sin(-fovRad/2+panRad)*viewLength);
		int yLeft = (int)(yStart-Math.cos(-fovRad/2+panRad)*viewLength);
		
		int xRight= (int) (xStart+Math.sin(fovRad/2+panRad)*viewLength);
		int yRight = (int)(yStart-Math.cos(fovRad/2+panRad)*viewLength);
	
		
		int[] xs = { xStart, xLeft, xRight };
		int[] ys = { yStart, yLeft, yRight};
		Polygon triangle = new Polygon(xs, ys, xs.length);
		g.setColor(new Color(.2f,.2f,.2f,.6f));
		g.fillPolygon(triangle);
		
		g.setColor(Color.BLACK);
		g2d.setFont(new Font( "Dialog", Font.BOLD, 11 ));
		g2d.drawString(Double.toString(flHeight),3, 97);
		g2d.drawString(Double.toString(blHeight),3, (this.getHeight()-85));
		g2d.drawString(Double.toString(frHeight),(this.getWidth()-30), 97);
		g2d.drawString(Double.toString(brHeight),(this.getWidth()-30), (this.getHeight()-85));
        g2d.dispose();
        g.dispose();
	}
	
	public void setHeights(double fl,double fr, double bl, double br){
		DecimalFormat decimalFormat = new DecimalFormat( "#.##" );
		flHeight=new Double(decimalFormat.format(fl)).doubleValue();
		frHeight=new Double(decimalFormat.format(fr)).doubleValue();
		blHeight=new Double(decimalFormat.format(bl)).doubleValue();
		brHeight=new Double(decimalFormat.format(br)).doubleValue();
		repaint();
	}
	
	public void setPanTilt(double pan, double tilt){
		this.pan=pan;
		this.tilt=tilt;
		repaint();
	}
}
