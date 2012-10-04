package instruments;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Point;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.awt.image.BufferedImageOp;
import javax.swing.JComponent;

public class VisualInstrument extends JComponent {
	
	public VisualInstrument() {
	}
	
	protected void RotateImage(Graphics2D g, Image img, double alpha, Point ptImg, Point ptRot, float scaleFactor){
		AffineTransform m_affineTransform  = new AffineTransform(); 
        m_affineTransform.setToTranslation(ptImg.x*scaleFactor,ptImg.y*scaleFactor); 
        //rotate with the rotation point as the mid of the image 
      m_affineTransform.rotate(alpha, ptRot.x*scaleFactor,ptRot.y*scaleFactor); 
         //draw the image using the AffineTransform 
      g.drawImage(img, m_affineTransform, null); 
    
	}
	

	
    protected void TranslateImage(Graphics2D g, Image img, int deltaPx, float alpha, Point ptImg, float scaleFactor)
    {
        // Computed offset
        float deltaX = (float)(deltaPx * (Math.sin(alpha)));
        float deltaY = (float)(- deltaPx * (Math.cos(alpha)));
        // Dispay image
        g.drawImage(img, (int)((ptImg.x + deltaX) * scaleFactor), (int)((ptImg.y + deltaY) * scaleFactor),(int) (img.getWidth(null) * scaleFactor),(int)( img.getHeight(null) * scaleFactor),null);
    }
    
    protected void RotateAndTranslate(Graphics2D g, Image img, Double roll, Double alphaTrs, Point ptImg, double pitch, Point ptRot, float scaleFactor)
    {
    	AffineTransform m_affineTransform  = new AffineTransform(); 
       float deltaXTrs = (float)(-pitch * (Math.sin(roll)));
       float deltaYTrs = (float)(pitch * (Math.cos(roll)));
  
        m_affineTransform.setToTranslation((ptImg.x)*scaleFactor,(ptImg.y)*scaleFactor); 
        //rotate with the rotation point as the mid of the image 
        m_affineTransform.translate((ptRot.x+deltaXTrs)*scaleFactor,(ptRot.y+deltaYTrs)*scaleFactor);   
        m_affineTransform.rotate(roll, (ptRot.x+0)*scaleFactor,(ptRot.y+0)*scaleFactor);    
      //draw the image using the AffineTransform 
        g.drawImage(img, m_affineTransform, null); 
    }
    	
   
    
    protected float InterpolPhyToAngle(float phyVal, float minPhy, float maxPhy, float minAngle, float maxAngle)
    {
        float a;
        float b;
        float y;
        float x;

        if (phyVal < minPhy)
        {
            return (float)(minAngle * Math.PI / 180);
        }
        else if (phyVal > maxPhy)
        {
            return (float)(maxAngle * Math.PI / 180);
        }
        else
        {

            x = phyVal;
            a = (maxAngle - minAngle) / (maxPhy - minPhy);
            b = (float)(0.5 * (maxAngle + minAngle - a * (maxPhy + minPhy)));
            y = a * x + b;
            return (float)(y * Math.PI / 180);
        }
    }

    protected Point FromCartRefToImgRef(Point cartPoint)
    {
        Point imgPoint = new Point();
        imgPoint.y = cartPoint.x + (this.getWidth()/2);
        imgPoint.y = -cartPoint.y + (this.getHeight()/2);
        return (imgPoint);
    }

    protected double FromDegToRad(double degAngle)
    {
        double radAngle = degAngle * Math.PI / 180;
        return radAngle;
    }
    
}
