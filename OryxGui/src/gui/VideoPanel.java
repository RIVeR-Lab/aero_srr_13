package gui;

import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_imgproc.CV_THRESH_BINARY;
import static com.googlecode.javacv.cpp.opencv_imgproc.cvThreshold;
import instruments.SmartComboBox;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Font;
import java.util.ArrayList;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JPanel;

import org.ros.message.MessageListener;
import org.ros.message.sensor_msgs.Image;
import org.ros.node.topic.Publisher;

import com.sun.org.apache.bcel.internal.generic.IF_ACMPEQ;

/**
 * A panel containing a video selection box and a video feed
 * @author joe
 *
 */
public class VideoPanel extends JPanel {
	public JPanel videoSelectionPanel;
	public SmartComboBox videoSourceSelector;
	public JCheckBox redFilterCheckBox;
	public JCheckBox greenFilterCheckBox;
	public JCheckBox blueFilterCheckBox;
	public JCheckBox orangeFilterCheckBox;
	public JCheckBox yellowFilterCheckBox;
	public JCheckBox purpleFilterCheckBox;


	public ArrayList<String> CameraList = new ArrayList<String>();


	public Publisher<Image> redImagePublisher;
	public Publisher<Image> blueImagePublisher;
	public Publisher<Image> greenImagePublisher;
	public Publisher<Image> yellowImagePublisher;
	public Publisher<Image> orangeImagePublisher;
	public Publisher<Image> purpleImagePublisher;
	
	public VideoDisplay videoDisplayPanel;
	public ImageListener videoListener=new ImageListener();
	
	public int redThresh=50;
	public int blueThresh=20;
	public int greenThresh=20;
	/**
	 * Create the panel.
	 */
	public VideoPanel(boolean includeColorTracker) {
		initGUI(includeColorTracker);
	}

	private void initGUI(boolean includeColorTracker) {

		setLayout(new BorderLayout(0, 0));

		this.videoSelectionPanel = new JPanel();
		add(this.videoSelectionPanel, BorderLayout.SOUTH);
		this.videoSelectionPanel.setLayout(new BoxLayout(
				this.videoSelectionPanel, BoxLayout.X_AXIS));

		this.videoSourceSelector = new SmartComboBox();
		videoSourceSelector.addItem("No Selection");
		this.videoSelectionPanel.add(this.videoSourceSelector);

		Component horizontalStrut = Box.createHorizontalStrut(20);
		if(includeColorTracker)this.videoSelectionPanel.add(horizontalStrut);


		//Initialize check boxes for color filtering
		JLabel lblColorSelection = new JLabel("Color Filtering:");
		lblColorSelection.setFont(new Font("Dialog", Font.BOLD, 14));
		if(includeColorTracker)this.videoSelectionPanel.add(lblColorSelection);
		
		this.redFilterCheckBox = new JCheckBox("Red");
		this.redFilterCheckBox.setFont(new Font("Dialog", Font.BOLD, 14));
		if(includeColorTracker)this.videoSelectionPanel.add(this.redFilterCheckBox);

		this.greenFilterCheckBox = new JCheckBox("Green");
		this.greenFilterCheckBox.setFont(new Font("Dialog", Font.BOLD, 14));
		if(includeColorTracker)this.videoSelectionPanel.add(this.greenFilterCheckBox);

		this.blueFilterCheckBox = new JCheckBox("Blue");
		this.blueFilterCheckBox.setFont(new Font("Dialog", Font.BOLD, 14));
		if(includeColorTracker)this.videoSelectionPanel.add(this.blueFilterCheckBox);
		
		this.yellowFilterCheckBox = new JCheckBox("Yellow");
		this.yellowFilterCheckBox.setFont(new Font("Dialog", Font.BOLD, 14));
		if(includeColorTracker)this.videoSelectionPanel.add(this.yellowFilterCheckBox);

		this.orangeFilterCheckBox = new JCheckBox("Orange");
		this.orangeFilterCheckBox.setFont(new Font("Dialog", Font.BOLD, 14));
		if(includeColorTracker)this.videoSelectionPanel.add(this.orangeFilterCheckBox);

		this.purpleFilterCheckBox = new JCheckBox("Purple");
		this.purpleFilterCheckBox.setFont(new Font("Dialog", Font.BOLD, 14));
		if(includeColorTracker)this.videoSelectionPanel.add(this.purpleFilterCheckBox);

		
		this.videoDisplayPanel = new VideoDisplay();
		add(this.videoDisplayPanel, BorderLayout.CENTER);
		this.videoDisplayPanel.setLayout(new BorderLayout(0, 0));	
	}

	public class ImageListener implements MessageListener<Image> {
		long time;
		@Override
		public void onNewMessage(Image img) {
			time = System.currentTimeMillis();
			String cameraName = img.header.frame_id;
			
			//If the camera name is not in the selection box, add it
			if(!CameraList.contains(cameraName)){
				CameraList.add(cameraName);
				videoSourceSelector.addItem(cameraName);
			}
			//Only display image if its header name is the same as the name in the selection box
			if (cameraName.equals((String)videoSourceSelector.getSelectedItem( )) ){	
				IplImage cvImage = JavaCVBridge.imageMessageToCv(img, CV_8UC3);
				//videoDisplayPanel.show(cvImage.getBufferedImage());
				videoDisplayPanel.show(JavaCVBridge.messageToBufferedImage(img));
				if(redFilterCheckBox.isSelected())redImagePublisher.publish(img);
				if(blueFilterCheckBox.isSelected())blueImagePublisher.publish(img);
				if(greenFilterCheckBox.isSelected())greenImagePublisher.publish(img);
				if(orangeFilterCheckBox.isSelected())orangeImagePublisher.publish(img);
				if(yellowFilterCheckBox.isSelected())yellowImagePublisher.publish(img);
				if(purpleFilterCheckBox.isSelected())purpleImagePublisher.publish(img);
				
			}
			//System.out.println("Time To Display: " + (System.currentTimeMillis()-time));
			
		}
	}
	

}
