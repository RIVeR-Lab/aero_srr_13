package instruments;

import javax.swing.JPanel;
import javax.swing.JLabel;
import javax.swing.BoxLayout;
import javax.swing.SwingConstants;
import javax.swing.SwingWorker;

import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

import javax.swing.border.EtchedBorder;
import javax.swing.JTextField;

/**
 * A panel that displays the ping time to the selected url/ip
 * @author joe
 *
 */
public class PingIndicatorPanel extends JPanel {
	private JLabel lblPingTimes;
	private JTextField fieldIP;
	private JLabel lblTimes;
	String ip;

	/**
	 * Create and initialize the panel
	 */
	public PingIndicatorPanel() {
		setBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null));
		setLayout(new GridLayout(0, 1, 0, 0));
		setMaximumSize(new Dimension(232,50));
		//Default to Google
		this.ip = "www.google.com";
		lblPingTimes = new JLabel("Ping Times:");
		lblPingTimes.setFont(new Font("Dialog", Font.BOLD, 16));
		lblPingTimes.setHorizontalAlignment(SwingConstants.CENTER);
		add(lblPingTimes);
		
		fieldIP = new JTextField(this.ip);
		fieldIP.addActionListener(new ActionListener() {	
			@Override
			public void actionPerformed(ActionEvent arg0) {
				ip = fieldIP.getText();
				
			}
		});
		fieldIP.setHorizontalAlignment(SwingConstants.CENTER);
		fieldIP.setFont(new Font("Dialog", Font.BOLD, 14));
		add(fieldIP);
		
		lblTimes = new JLabel("0 ms");
		lblTimes.setHorizontalAlignment(SwingConstants.CENTER);
		lblTimes.setFont(new Font("Dialog", Font.BOLD, 14));
		add(lblTimes);
		//Start the ping thread
		PingFinderThread.start();
		
	}
	
	private Thread PingFinderThread = new Thread(){
		public void run(){
			BufferedReader reader;
			
			while(true){
				try {
					//This runs a ping command once with a timeout of 15 seconds
					Process p = Runtime.getRuntime().exec("ping -c1 -w 15 " + ip);
					p.waitFor();
					reader=new BufferedReader(new InputStreamReader(p.getInputStream())); 
					String line; 
					
					while((line=reader.readLine())!=null) { 	
						String[] splits=line.split("time=");
						//Find where the ping time is displayed
						if(splits.length == 2){
							lblTimes.setText(splits[1]);	}
						
					} 
					p.getInputStream().close();
					reader.close();
					//Close the process
					p.destroy();
					p.waitFor();
					//Wait 1 second
					Thread.sleep(1000);
				} catch (IOException e) {
					e.printStackTrace();
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	};
	
	


}
