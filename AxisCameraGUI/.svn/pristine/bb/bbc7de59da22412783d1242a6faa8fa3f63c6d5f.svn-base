import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JPanel;
import javax.swing.JTextField;

import axiscamera.AxisCamera;
import axiscamera.image.ImageFormat;
import axiscamera.image.ResolutionConfig;
import axiscamera.image.RotationConfig;
import axiscamera.image.VideoConfig;

public class ImageDisplay extends JPanel {

	public static final int HTTP_CACHING = 100;
	public static final int RTSP_CACHING = 150;

	private JTextField urlDisplay;
	private Viewer viewer;
	private JPanel controlPanel;
	private JButton updateVideoButton;
	private JComboBox formatSelector;
	private JCheckBox rstpCheckbox;
	private JComboBox resolutionSelector;
	private JComboBox rotationSelector;
	private JComboBox fpsSelector;

	public ImageDisplay(final AxisCamera camera) {
		setLayout(new BorderLayout());
		add(urlDisplay = new JTextField(), BorderLayout.NORTH);
		urlDisplay.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				setURL(urlDisplay.getText());
			}
		});
		add(viewer = new Viewer());
		add(controlPanel = new JPanel(), BorderLayout.SOUTH);
		controlPanel.add(updateVideoButton = new JButton("Update"));

		updateVideoButton.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent e) {
				try {
					String url = null;
					if (rstpCheckbox.isSelected()) {
						url = camera.getImageRstlURL(
								(ImageFormat) formatSelector.getSelectedItem(),
								(ResolutionConfig) resolutionSelector
										.getSelectedItem(),
								(RotationConfig) rotationSelector
										.getSelectedItem(),
										new VideoConfig( ((Integer)fpsSelector.getSelectedItem()).intValue() ));
					} else {
						url = camera.getImageHttpURL(
								(ImageFormat) formatSelector.getSelectedItem(),
								(ResolutionConfig) resolutionSelector
										.getSelectedItem(),
								(RotationConfig) rotationSelector
										.getSelectedItem(),
										new VideoConfig( ((Integer)fpsSelector.getSelectedItem()).intValue() ) );
					}
					setURL(url);
				} catch (RuntimeException e1) {
					stop();
					urlDisplay.setText("");
					throw e1;
				}
			}
		});

		formatSelector = new JComboBox(ImageFormat.getSupportedFormats(camera)/*new ImageFormat[]{}*/);
		controlPanel.add(formatSelector);

		controlPanel.add(rstpCheckbox = new JCheckBox("Use RSTP"));

		resolutionSelector = new JComboBox(Util.addNull(ResolutionConfig
				.getSupported(camera)/*new ResolutionConfig[]{}*/));
		controlPanel.add(resolutionSelector);

		rotationSelector = new JComboBox(Util.addNull(RotationConfig
				.getSupported(camera)/*new RotationConfig[]{}*/));
		controlPanel.add(rotationSelector);

		fpsSelector = new JComboBox(new Integer[]{1, 10, 30});
		controlPanel.add(fpsSelector);

		stop();
	}

	public void setURL(String url) {
		System.out.println("Set video url to " + url);
		urlDisplay.setText(url);
		viewer.setURL(url);
	}

	public void stop() {
		viewer.setURL(null);
	}
}
