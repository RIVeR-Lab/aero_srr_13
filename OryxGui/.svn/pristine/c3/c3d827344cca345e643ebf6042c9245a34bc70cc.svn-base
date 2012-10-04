package gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;

import javax.swing.ImageIcon;
import javax.swing.JLabel;
import javax.swing.JPanel;

import org.ros.message.MessageListener;
import org.ros.message.OryxMessages.Blob;
import org.ros.message.OryxMessages.BlobList;

/**
 * Displays a video feed
 * @author joe
 *
 */
public class VideoDisplay extends JPanel {

	BufferedImage img;
	private boolean newImage=false;
	JLabel picLabel;
	ImageIcon picIcon;
	public BlobListener blobListener;
	BlobList newestMsg;
	BlobList newestRed, newestGreen, newestBlue,newestOrange, newestYellow, newestPurple;
	boolean messageChanged=false;
	boolean redChanged=false,blueChanged=false,greenChanged=false,yellowChanged=false,orangeChanged=false,purpleChanged=false;
	/**
	 * Create the panel.
	 */
	public VideoDisplay() {
		img = new BufferedImage(960, 544, BufferedImage.TYPE_3BYTE_BGR);
		blobListener = new BlobListener();

	}
	
	public void show(BufferedImage image)  {
		this.img = image;
		newImage = true;
		repaint();
	}

	
	@Override
	public void paint(Graphics g) {
		if(img == null) {
			System.err.println("WARNING: IMAGE IS NULL");
			return;
		}
		float scale = ((float)this.getWidth() / (float)img.getWidth(null));
		Graphics2D g2d = (Graphics2D) g;
		
		//Render for improved color quality
		g2d.setRenderingHint(RenderingHints.KEY_COLOR_RENDERING,RenderingHints.VALUE_COLOR_RENDER_QUALITY);
		g2d.setClip(0, 0, (int) (img.getWidth()*scale), (int) (img.getHeight()*scale));
		
		//If there is a new image, draw it
		if (newImage){
			g2d.drawImage(img, 0, 0, (int) (img.getWidth(null) * scale),(int) (img.getHeight(null) * scale), null);
			newImage=false;
		}
		
		//If there is a new message
		if(newestMsg!=null && messageChanged){
			g2d.setStroke(new BasicStroke(4));
			
			//Draw circles around blobs
			if( newestRed!= null && redChanged){
				g2d.setColor(Color.RED);
				for (Blob blob : newestRed.blobs) {
					g2d.drawOval((int)(scale*(blob.x-15)), (int)(scale*(blob.y-15)), (int)(scale*30), (int)(scale*30));
				}
				redChanged=false;
			}
			if( newestGreen!= null && greenChanged){
				g2d.setColor(Color.GREEN);
				for (Blob blob : newestGreen.blobs) {
					g2d.drawOval((int)(scale*(blob.x-15)), (int)(scale*(blob.y-15)), (int)(scale*30), (int)(scale*30));
				}
			greenChanged=false;
			}
			if( newestBlue!= null && blueChanged){
				g2d.setColor(Color.BLUE);
				for (Blob blob : newestBlue.blobs) {
					g2d.drawOval((int)(scale*(blob.x-15)), (int)(scale*(blob.y-15)), (int)(scale*30), (int)(scale*30));
				}
				blueChanged=false;
			}
			if( newestPurple!= null && purpleChanged){
				g2d.setColor(Color.MAGENTA.darker());
				for (Blob blob : newestPurple.blobs) {
					g2d.drawOval((int)(scale*(blob.x-15)), (int)(scale*(blob.y-15)), (int)(scale*30), (int)(scale*30));
				}
				purpleChanged=false;
			}
			if( newestOrange!= null && orangeChanged){
				g2d.setColor(Color.ORANGE);
				for (Blob blob : newestOrange.blobs) {
					g2d.drawOval((int)(scale*(blob.x-15)), (int)(scale*(blob.y-15)), (int)(scale*30), (int)(scale*30));
				}
				orangeChanged=false;
			}
			if( newestYellow!= null && yellowChanged){
				g2d.setColor(Color.YELLOW.darker());
				for (Blob blob : newestYellow.blobs) {
					g2d.drawOval((int)(scale*(blob.x-15)), (int)(scale*(blob.y-15)), (int)(scale*30), (int)(scale*30));
				}
				yellowChanged=false;
			}
			
			messageChanged=false;
		}

	}

	public class BlobListener implements MessageListener<BlobList> {

		@Override
		public void onNewMessage(BlobList msg) {
		//	System.out.println("Message Received");
			if(msg.color == 1) {
				newestRed = msg;
				redChanged = true;
			}
			else if(msg.color == 2) {
				newestGreen = msg;
				greenChanged = true;
			}
			else if(msg.color == 3){
				newestBlue = msg;
				blueChanged = true;
			}
			else if(msg.color == 4) {
				newestYellow = msg;
				yellowChanged = true;
			}
			else if(msg.color == 5) {
				newestOrange = msg;
				orangeChanged = true;
			}
			else if(msg.color == 6){
				newestPurple = msg;
				purpleChanged = true;
			}
			
			newestMsg = msg;
			messageChanged = true;
	
	

		}

	}

}
