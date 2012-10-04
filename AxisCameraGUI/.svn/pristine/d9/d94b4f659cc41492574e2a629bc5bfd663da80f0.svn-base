import java.awt.Color;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.image.BufferedImage;

import javax.swing.JPanel;

import com.googlecode.javacv.FFmpegFrameGrabber;
import com.googlecode.javacv.cpp.opencv_core.IplImage;

public class Viewer extends JPanel {
	private final Object sync = new Object();
	private Image buffer;
	private FFmpegFrameGrabber frameGrabber;

	public Viewer() {
		frameGrabberThread.start();
	}

	public void setURL(String source) {
		synchronized (sync) {
			if (frameGrabber != null) {
				try {
					frameGrabber.stop();
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				try {
					frameGrabber.release();
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			if (source == null)
				frameGrabber = null;
			else {
				frameGrabber = new FFmpegFrameGrabber(source);
				
				try {
					frameGrabber.start();
				} catch (Exception e) {
					e.printStackTrace();
					setURL(null);
				}
			}
		}
	}

	private Thread frameGrabberThread = new Thread() {
		public void run() {
			while (true) {
				synchronized (sync) {
					int width = getWidth();
					int height = getHeight();
					if (width <= 0 || height <= 0)
						continue;// do not bother if no size
					if (buffer == null || buffer.getWidth(Viewer.this) != width
							|| buffer.getHeight(Viewer.this) != height)
						buffer = new BufferedImage(width, height,
								BufferedImage.TYPE_INT_RGB);
					Graphics g = buffer.getGraphics();

					if (frameGrabber != null) {
						try {
							IplImage nativeImage = frameGrabber.grab();
							if (nativeImage != null) {
								Image image = nativeImage.getBufferedImage();
								g.drawImage(image, 0, 0, width, height,
										Viewer.this);
							}
							else{
								g.setColor(Color.BLUE);
								g.drawRect(0, 0, width, height);
							}
						} catch (Exception e) {
							e.printStackTrace();
							g.setColor(Color.ORANGE);
							g.drawRect(0, 0, width, height);
						}
					} else {
						g.setColor(Color.BLACK);
						g.drawRect(0, 0, width, height);
						try {
							Thread.sleep(20);
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
				}

				paint(getGraphics());
			}
		}
	};

	public void paint(Graphics g) {
		g.drawImage(buffer, 0, 0, this);
	}
}
