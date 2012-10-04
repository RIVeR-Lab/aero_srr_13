package axiscamera.image;

import java.util.HashMap;
import java.util.Map;

import axiscamera.AxisCamera;
import axiscamera.AxisCameraException;

public class ImageFormat {
	public static final ImageFormat JPEG = new ImageFormat("jpeg",
			"jpg/image.jpg", null);
	public static final ImageFormat MJPEG = new ImageFormat("mjpeg",
			"mjpg/video.mjpg", "jpeg");
	public static final ImageFormat H264 = new ImageFormat("h264", null,
			"h264");
	public static final ImageFormat BITMAP = new ImageFormat("bitmap",
			"bitmap/image.bmp", null);//TODO bmp does not display properly in vlcj

	private String name;
	private String httpCommand;
	private String rtspType;

	public ImageFormat(String name, String httpCommand, String rtspType) {
		this.name = name;
		this.httpCommand = httpCommand;
		this.rtspType = rtspType;
	}

	public String toString() {
		return name;
	}

	public String getImageHttpURL(AxisCamera camera, ImageConfig... properties) {
		if (httpCommand == null)
			throw new AxisCameraException("HTTP not supported for image type: "
					+ name);
		Map<String, String> args = new HashMap<String, String>();
		args.put("camera", Integer.toString(camera.getCameraNumber()));
		for (ImageConfig property : properties)
			if (property != null)
				property.applyHttp(args);
		return camera.buildURLString("http", httpCommand, args);
	}

	public String getImageRstlURL(AxisCamera camera, ImageConfig... properties) {
		if (rtspType == null)
			throw new AxisCameraException("RSTL not supported for image type: "
					+ name);
		Map<String, String> args = new HashMap<String, String>();
		args.put("camera", Integer.toString(camera.getCameraNumber()));
		for (ImageConfig property : properties)
			if (property != null)
				property.applyRstp(args);
		args.put("videocodec", rtspType);
		return camera.buildURLString("rtsp", "axis-media/media.amp", args);
	}

	public static ImageFormat[] getSupportedFormats(AxisCamera camera) {
		String[] supportedFormatNames = camera
				.listParam("Properties.Image.Format")
				.get("Properties.Image.Format").split(",");
		ImageFormat[] supportedFormats = new ImageFormat[supportedFormatNames.length];
		for (int i = 0; i < supportedFormatNames.length; ++i) {
			String formatName = supportedFormatNames[i];
			if (formatName.equals("jpeg"))
				supportedFormats[i] = JPEG;
			else if (formatName.equals("mjpeg"))
				supportedFormats[i] = MJPEG;
			else if (formatName.equals("h264"))
				supportedFormats[i] = H264;
			else if (formatName.equals("bitmap"))
				supportedFormats[i] = BITMAP;
			else
				supportedFormats[i] = new ImageFormat(formatName, null, null);
		}
		return supportedFormats;
	}

}
