package instruments;

import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.GridLayout;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.SwingConstants;
import javax.swing.border.EtchedBorder;
import java.awt.BorderLayout;

public class BatteryIndicator extends JPanel {
	private float threshold,minimum,maximum;
	private float voltage;
	private int cellNumber;
	JLabel lblCurrentVoltage;
	JProgressBar batteryBar;
	int scale=100;
	public JPanel batteryInfoPanel;
	NumberFormat formatter = new DecimalFormat("#.##");
	
	//TODO:Switch threshold and cellNumber
	public BatteryIndicator(float minimum, float maximum, float threshold, int cellNumber) {
		setBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null));
		this.cellNumber=cellNumber;
		this.threshold=threshold;
		this.minimum = minimum;
		this.maximum=maximum;
		setLayout(new BorderLayout(0, 0));
		this.setPreferredSize(new Dimension(232, 70));
		
		batteryInfoPanel = new JPanel();
		add(batteryInfoPanel, BorderLayout.NORTH);
		batteryInfoPanel.setLayout(new GridLayout(0, 2, 0, 0));
		JLabel lblCellNumber;
		if(cellNumber == 0){
			lblCellNumber = new JLabel("Total");
		}
		else {
			lblCellNumber = new JLabel("Cell " + String.valueOf(cellNumber));
		}
		batteryInfoPanel.add(lblCellNumber);
		lblCellNumber.setAlignmentX(Component.CENTER_ALIGNMENT);
		lblCellNumber.setHorizontalAlignment(SwingConstants.CENTER);
		lblCellNumber.setFont(new Font("Dialog", Font.BOLD, 14));
		lblCellNumber.setPreferredSize(new Dimension(30,20));
		
		lblCurrentVoltage = new JLabel("0.0V");
		batteryInfoPanel.add(lblCurrentVoltage);
		lblCurrentVoltage.setHorizontalAlignment(SwingConstants.CENTER);
		lblCurrentVoltage.setFont(new Font("Dialog", Font.BOLD, 14));
		lblCurrentVoltage.setPreferredSize(new Dimension(60, 20));
		
		JPanel panel = new JPanel();
		add(panel);
		panel.setLayout(new GridLayout(0, 1, 0, 0));
		
		
		batteryBar = new JProgressBar();
		batteryBar.setStringPainted(true);
		panel.add(batteryBar);
		batteryBar.setForeground(new Color(0, 255, 0));
		batteryBar.setMaximum((int)(maximum*scale));
		batteryBar.setMinimum((int)(minimum*scale));		
		batteryBar.setValue((int)(4.3*scale));
		batteryBar.setPreferredSize(new Dimension(70, 50));
	}
	
	public void setVoltage(float voltage){
		if(voltage <=threshold) {
			lblCurrentVoltage.setForeground(Color.red);
			batteryBar.setForeground(Color.red);
		}
		else{
			lblCurrentVoltage.setForeground(Color.black);
			batteryBar.setForeground(Color.green);
		}
		
		lblCurrentVoltage.setText(formatter.format(voltage)+"V");
		batteryBar.setValue((int)(voltage*scale));
	
	}

}
