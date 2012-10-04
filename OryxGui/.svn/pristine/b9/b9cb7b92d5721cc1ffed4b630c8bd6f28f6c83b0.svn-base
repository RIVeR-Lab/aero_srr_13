package instruments;

import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.GridLayout;
import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;
import javax.swing.border.EtchedBorder;

import sun.awt.windows.ThemeReader;

/**
 * A Thermometer instrument containing both a visible thermometer bar as well
 * as a text display of the name of the instrument and the value in Fahrenheit
 * @author joe
 *
 */
public class Thermometer extends JPanel {
	private ThermometerBar thermometerBar;
	public JPanel thermometerNamePanel;
	public JLabel thermometerNameLabel;
	public JLabel temperatureLabel;
	public int thermometersNumber,minTemp,maxTemp;
	public String thermometerName="Thermometer Name";
	NumberFormat formatter = new DecimalFormat("#.##");
	public Component horizontalGlue;
	public Component horizontalStrut;
	
	/**
	 * Creates a thermometer instrument
	 * @param number The node number associated with the thermometer
	 * @param name The name of the instrument/location where the thermometer is
	 * @param min The lowest value likely to be seen on the thermometer
	 * @param max The maximum allowed value on the thermometer
	 */
	public Thermometer(int number, String name, int min, int max) {
		setBorder(new EtchedBorder(EtchedBorder.LOWERED, null, null));
		thermometersNumber=number;
		thermometerName=name;
		minTemp = min;
		maxTemp = max;	
		init();
	}
	
	private void init() {
		setLayout(new GridLayout(0, 1, 0, 0));
		this.setPreferredSize(new Dimension(232, 30));
		this.setMaximumSize(new Dimension(232, 30));
		this.thermometerNamePanel = new JPanel();
		add(this.thermometerNamePanel);
		this.thermometerNamePanel.setLayout(new BoxLayout(this.thermometerNamePanel, BoxLayout.X_AXIS));
		
		this.thermometerNameLabel = new JLabel(thermometerName);
		this.thermometerNameLabel.setFont(new Font("Dialog", Font.PLAIN, 14));
		this.thermometerNamePanel.add(this.thermometerNameLabel);
		
		this.horizontalGlue = Box.createHorizontalGlue();
		this.thermometerNamePanel.add(this.horizontalGlue);
		
		this.temperatureLabel = new JLabel("0.0F");
		this.temperatureLabel.setFont(new Font("Dialog", Font.BOLD, 14));
		this.temperatureLabel.setHorizontalAlignment(SwingConstants.RIGHT);
		this.thermometerNamePanel.add(this.temperatureLabel);
		
		this.horizontalStrut = Box.createHorizontalStrut(10);
		this.thermometerNamePanel.add(this.horizontalStrut);
		thermometerBar = new ThermometerBar(minTemp,maxTemp);
		this.thermometerBar.setValue(0);

		this.add(thermometerBar);
	}
	
	/**
	 * Sets the temperature of the thermometer
	 * @param temperature The temperature in degrees fahrenheit
	 */
	public void setTemperature(float temperature){
		thermometerBar.setValue(temperature);
		if(temperature >= maxTemp){
			temperatureLabel.setForeground(Color.red);
		}
		else temperatureLabel.setForeground(Color.BLACK);
		temperatureLabel.setText(formatter.format(temperature)+"F");
	}

}
