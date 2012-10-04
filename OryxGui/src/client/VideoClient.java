package client;

import gui.OryxGuiFrame;
import gui.VideoFrame;

import java.awt.EventQueue;

import org.apache.commons.logging.Log;
import org.ros.message.OryxMessages.BlobList;
import org.ros.message.geometry_msgs.Quaternion;
import org.ros.message.sensor_msgs.Image;
import org.ros.message.sensor_msgs.Joy;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import com.google.common.base.Preconditions;

import controller.MotorController;

public class VideoClient implements NodeMain {
	VideoFrame videoFrame;
	private Node videoNode;
	

	@Override
	public void onStart(Node node) {
		Preconditions.checkState(videoNode == null);	
		videoNode = node;
		final Log log = videoNode.getLog();
			EventQueue.invokeLater(new Runnable() {
				public void run() {
					try {
						videoFrame = new VideoFrame();
						videoFrame.setVisible(true);
					} catch (Exception e) {
						e.printStackTrace();
					}
				}
			});
			
			try {Thread.sleep(1500);} catch (InterruptedException e) {e.printStackTrace();}

			Subscriber<Image> videoSubscriber = node.newSubscriber("Video_Stream", "sensor_msgs/Image");
			videoSubscriber.addMessageListener(videoFrame.videoPanel.videoListener);
			videoSubscriber.addMessageListener(videoFrame.armVideoPanel.videoListener);
			videoSubscriber.addMessageListener(videoFrame.driveVideoPanel.videoListener);
			videoSubscriber.setQueueLimit(5);
			
			Subscriber<BlobList> redBlobSubscriber = node.newSubscriber("ColorTracking/Result", "OryxMessages/BlobList");
			redBlobSubscriber.addMessageListener(videoFrame.videoPanel.videoDisplayPanel.blobListener);
		
			Subscriber<Quaternion> ptzAngleSubscriber = node.newSubscriber("PTZ/Orientation", "geometry_msgs/Quaternion");
			ptzAngleSubscriber.addMessageListener(videoFrame.ptzAnglePanel.orientationListener);
			ptzAngleSubscriber.setQueueLimit(1);
			
			Subscriber<Joy> driverJoySubscriber = node.newSubscriber("OperatorJoy", "sensor_msgs/Joy");
			driverJoySubscriber.addMessageListener(videoFrame.xboxController);
			driverJoySubscriber.setQueueLimit(1);
			
			videoFrame.videoPanel.redImagePublisher = videoNode.newPublisher("ColorTracking/Red", "sensor_msgs/Image");
			videoFrame.videoPanel.blueImagePublisher = videoNode.newPublisher("ColorTracking/Blue", "sensor_msgs/Image");
			videoFrame.videoPanel.greenImagePublisher = videoNode.newPublisher("ColorTracking/Green", "sensor_msgs/Image");
			videoFrame.videoPanel.yellowImagePublisher = videoNode.newPublisher("ColorTracking/Yellow", "sensor_msgs/Image");
			videoFrame.videoPanel.orangeImagePublisher = videoNode.newPublisher("ColorTracking/Orange", "sensor_msgs/Image");
			videoFrame.videoPanel.purpleImagePublisher = videoNode.newPublisher("ColorTracking/Purple", "sensor_msgs/Image");
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("OperatorGui");
	}

	@Override
	public void onShutdown(Node arg0) {
		arg0.shutdown();
		
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}


}
