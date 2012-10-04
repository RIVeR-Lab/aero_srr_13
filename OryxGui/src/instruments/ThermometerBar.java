package instruments;

import java.awt.Color;
import java.awt.GradientPaint;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.RoundRectangle2D;

import javax.swing.JComponent;

/**
 * This is a class for a progress bar that has a dual color gradient
 * @author joe
 */
public class ThermometerBar extends JComponent {
	private int minimum,maximum;
	private Color startColor = Color.green;
	private Color endColor = Color.red;
	private Color midColor = Color.yellow;
	private float value=0;
	float percent=0;
	
	public int getMinimum() {
		return minimum;
	}



	public void setMinimum(int minimum) {
		this.minimum = minimum;
	}



	public int getMaximum() {
		return maximum;
	}



	public void setMaximum(int maximum) {
		this.maximum = maximum;
	}



	public float getValue() {
		return value;
	}



	public void setValue(float value) {
		this.value = value;
		repaint();
	}



	public float getPercent() {
		return percent;
	}



	public void setStartColor(Color startColor) {
		this.startColor = startColor;
	}



	public void setEndColor(Color endColor) {
		this.endColor = endColor;
	}




	
	public ThermometerBar(int min, int max) {
		this.minimum=min;
		this.maximum=max;
	}
	

	/**This method paints the double gradient effect on the thermometer bar
	 * 
	 */
	@Override
	public void paint(Graphics g){
		Graphics2D g2d = (Graphics2D)g;
		GradientPaint startMidGradient = new GradientPaint(0, 0, startColor, this.getWidth()/2, 0, midColor, false); 
		GradientPaint midEndGradient = new GradientPaint(this.getWidth()/2, 0, midColor, this.getWidth(), 0, endColor, false); 
		g2d.setPaint(startMidGradient); 		
		percent = (float)(value-minimum)/(float)(maximum-minimum);
		g2d.setClip(0, 0, (int) (this.getWidth() * percent), this.getHeight());
		g2d.fill(new RoundRectangle2D.Double(0, 0, this.getWidth()/2, this.getHeight(), 0, 0));
		g2d.setPaint(midEndGradient); 
		g2d.fill(new RoundRectangle2D.Double(this.getWidth()/2, 0, this.getWidth(), this.getHeight(), 0, 0));
	}
}
