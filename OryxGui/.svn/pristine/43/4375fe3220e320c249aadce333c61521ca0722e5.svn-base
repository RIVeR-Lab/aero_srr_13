/**
 * 
 */
package client;

import gui.OryxGuiFrame;


import java.awt.EventQueue;
import org.apache.commons.logging.Log;
import org.ros.RosCore;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;
import org.ros.message.EposManager.GroupMotorInfo;
import org.ros.message.OryxMessages.Battery;
import org.ros.message.OryxMessages.BlobList;
import org.ros.message.OryxMessages.Temperature;
import org.ros.message.geometry_msgs.PoseArray;
import org.ros.message.geometry_msgs.Quaternion;
import org.ros.message.geometry_msgs.QuaternionStamped;
import org.ros.message.sensor_msgs.*;

import com.google.common.base.Preconditions;

/**
 * @author oryx
 *
 */
public class OryxClient implements NodeMain{
	private OryxGuiFrame Gui;
	private Node guiNode;

	@Override
	public void onStart(Node node){
	Preconditions.checkState(guiNode == null);	
	guiNode = node;
	final Log log = guiNode.getLog();
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					Gui = new OryxGuiFrame();
					Gui.setVisible(true);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
		
		try {Thread.sleep(1500);} catch (InterruptedException e) {e.printStackTrace();}
		while(Gui.xboxController==null){
			try {Thread.sleep(100);} catch (InterruptedException e) {e.printStackTrace();}
		}

		Subscriber<Image> videoSubscriber = node.newSubscriber("Video_Stream", "sensor_msgs/Image");
		videoSubscriber.addMessageListener(Gui.videoPanel.videoListener);
		videoSubscriber.addMessageListener(Gui.subVideoPanel.videoListener);
		videoSubscriber.setQueueLimit(5);
		
		Subscriber<Joy> joySubscriber = node.newSubscriber("joy", "sensor_msgs/Joy");
		joySubscriber.addMessageListener(Gui.xboxController);
		joySubscriber.setQueueLimit(1);
		
		Subscriber<Joy> driverJoySubscriber = node.newSubscriber("DriverJoy", "sensor_msgs/Joy");
		driverJoySubscriber.addMessageListener(Gui.xboxController);
		driverJoySubscriber.setQueueLimit(1);
		
		Subscriber<GroupMotorInfo> driveMotorSubscriber = node.newSubscriber("motors/Drive_Motors/Group_Motor_Info", "EposManager/GroupMotorInfo");
		driveMotorSubscriber.addMessageListener(Gui.driveMotorController);

		Subscriber<GroupMotorInfo> ptzMotorSubscriber = node.newSubscriber("motors/PTZ_Motors/Group_Motor_Info", "EposManager/GroupMotorInfo");
		ptzMotorSubscriber.addMessageListener(Gui.ptzMotorController);
		
		Subscriber<GroupMotorInfo> armMotorSubscriber = node.newSubscriber("motors/Arm_Motors/Group_Motor_Info", "EposManager/GroupMotorInfo");
		armMotorSubscriber.addMessageListener(Gui.armMotorController);
		
		Subscriber<Temperature> temperatureSubscriber = node.newSubscriber("Temps", "OryxMessages/Temperature");
		temperatureSubscriber.addMessageListener(Gui.tempController);
		temperatureSubscriber.setQueueLimit(30);
		
		Subscriber<Battery> batterySubscriber = node.newSubscriber("Battery", "OryxMessages/Battery");
		batterySubscriber.addMessageListener(Gui.batteryController);
		batterySubscriber.setQueueLimit(10);
		
		Subscriber<Quaternion> ptzAngleSubscriber = node.newSubscriber("PTZ/Orientation", "geometry_msgs/Quaternion");
		ptzAngleSubscriber.addMessageListener(Gui.topDownPanel.orientationListener);
		ptzAngleSubscriber.setQueueLimit(1);
		
		Subscriber<Imu> orientationSubscriber = node.newSubscriber("imu", "sensor_msgs/Imu");
		orientationSubscriber.addMessageListener(Gui.attitudeIndicatorPanel);
		orientationSubscriber.setQueueLimit(1);
	
		Subscriber<PoseArray> heightSubscriber = node.newSubscriber("Heights", "geometry_msgs/PoseArray");
		heightSubscriber.addMessageListener(Gui.topDownPanel.wheelListener);
		heightSubscriber.setQueueLimit(1);
		//TODO: Implement Subscribers and Publishers
		//Subscriber<BlobList> redBlobSubscriber = node.newSubscriber("ColorTracking/Output", "OryxMessages/BlobList");
		//redBlobSubscriber.addMessageListener(Gui.videoPanel.videoDisplayPanel.blobListener);
	
		
		Gui.videoPanel.redImagePublisher = guiNode.newPublisher("ColorTracking/Red", "sensor_msgs/Image");
		Gui.videoPanel.blueImagePublisher = guiNode.newPublisher("ColorTracking/Blue", "sensor_msgs/Image");
		Gui.videoPanel.greenImagePublisher = guiNode.newPublisher("ColorTracking/Green", "sensor_msgs/Image");
		Gui.videoPanel.yellowImagePublisher = guiNode.newPublisher("ColorTracking/Yellow", "sensor_msgs/Image");
		Gui.videoPanel.orangeImagePublisher = guiNode.newPublisher("ColorTracking/Orange", "sensor_msgs/Image");
		Gui.videoPanel.purpleImagePublisher = guiNode.newPublisher("ColorTracking/Purple", "sensor_msgs/Image");
		

	}

	@Override
	public void onShutdown(Node node) {
		node.shutdown();
	}
	

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("OryxGui/Gui");
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

}
