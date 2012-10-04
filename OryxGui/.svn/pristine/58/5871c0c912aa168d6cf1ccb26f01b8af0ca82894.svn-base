package client;

import gui.OryxGuiFrame;
import gui.TelemetryGuiFrame;

import java.awt.EventQueue;

import org.apache.commons.logging.Log;
import org.ros.message.OryxMessages.Battery;
import org.ros.message.OryxMessages.Temperature;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import com.google.common.base.Preconditions;

public class TelemetryClient implements NodeMain{

	//	private OryxGuiFrame Gui;
	private Node telemetryNode;
	private TelemetryGuiFrame gui;
	
	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("OryxGui/Telemetry");
	}

	@Override
	public void onShutdown(Node node) {
		node.shutdown();
		
	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStart(Node node) {

		
		Preconditions.checkState(telemetryNode == null);	
		 telemetryNode = node;
			EventQueue.invokeLater(new Runnable() {
				public void run() {
					try {
						gui = new TelemetryGuiFrame();
						gui.setVisible(true);
					} catch (Exception e) {
						e.printStackTrace();
					}
				}
			});
			try {Thread.sleep(500);} catch (InterruptedException e) {e.printStackTrace();}
			final Log log = telemetryNode.getLog();
			
			Subscriber<Temperature> temperatureSubscriber = node.newSubscriber("Temps", "OryxMessages/Temperature");
			temperatureSubscriber.addMessageListener(gui.temperatureController);
			temperatureSubscriber.setQueueLimit(100);
			
			Subscriber<Battery> batterySubscriber = node.newSubscriber("Battery", "OryxMessages/Battery");
			batterySubscriber.addMessageListener(gui.batteryController);
			batterySubscriber.setQueueLimit(30);
		
	}

}
