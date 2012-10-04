package axiscamera;

import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLConnection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import axiscamera.image.ImageConfig;
import axiscamera.image.ImageFormat;
import axiscamera.util.HttpDevice;

public class AxisCamera extends HttpDevice {
	public static final String ERROR_PREFIX = "# Error: ";
	public static final String REQUEST_FAILED_PREFIX = "# Request Failed: ";

	public static final int DEFAULT_CAMERA_NUMBER = 1;

	private int cameraNumber;
	private String username;
	private String password;

	public AxisCamera(String url, String username, String password) throws MalformedURLException {
		this(url, DEFAULT_CAMERA_NUMBER, username, password);
	}

	public AxisCamera(String url, int cameraNumber, String username, String password) {
		super(url, username, password);
		this.cameraNumber = cameraNumber;
		this.username = username;
		this.password = password;
	}

	public int getCameraNumber() {
		return cameraNumber;
	}

	public String getUsername(){
		return username;
	}
	public String getPassword(){
		return password;
	}

	public URL buildCameraURL(String command, Map<String, String> queryArgs) {
		try {
			return buildURL(command, queryArgs);
		} catch (MalformedURLException e) {
			throw new AxisCameraException("Could not make URL", e);
		}
	}

	public URL buildCameraCgiURL(String command, Map<String, String> queryArgs) {
		return buildCameraURL("axis-cgi/" + command, queryArgs);
	}
	
	public URLConnection makeCgiRequest(String command, Map<String, String> queryArgs){
		try {
			return makeRequest(buildCameraCgiURL(command, queryArgs));
		} catch (IOException e) {
			throw new AxisCameraException("Error making request", e);
		}
	}
	
	public String makeStringCgiRequest(String command, Map<String, String> queryArgs){
		try {
			return makeStringRequest(buildCameraCgiURL(command, queryArgs));
		} catch (IOException e) {
			throw new AxisCameraException("Error making request", e);
		}
	}

	// PARAMETERS
	public Map<String, String> paramAction(String action,
			Map<String, String> params) {
		Map<String, String> queryArgs = new HashMap<String, String>();
		queryArgs.put("action", action);
		queryArgs.putAll(params);
		return parseParamBody(makeStringCgiRequest("param.cgi", queryArgs));
	}

	public Map<String, String> updateParam(String key, String value) {
		return updateParam(Collections.singletonMap(key, value));
	}

	public Map<String, String> updateParam(Map<String, String> keyValues) {
		return paramAction("update", keyValues);
	}

	public Map<String, String> listParam(String group) {
		return paramAction("list", Collections.singletonMap("group", group));
	}
	public static Map<String, String> parseParamBody(String body) {
		if (body == null)
			return null;
		// TODO errors may not be first thing in response
		if (body.startsWith(ERROR_PREFIX))
			throw new AxisCameraResponseException(body.substring(ERROR_PREFIX
					.length()));
		if (body.startsWith(REQUEST_FAILED_PREFIX))
			throw new AxisCameraResponseException(
					body.substring(REQUEST_FAILED_PREFIX.length()));

		Map<String, String> data = new HashMap<String, String>();
		String[] lines = body.split("\\r?\\n");
		for (String line : lines) {
			String[] lineSplit = line.split("\\s*=\\s*", 2);
			if (lineSplit.length == 0)
				continue;
			if (lineSplit.length == 1)
				data.put(lineSplit[0], "");
			else
				data.put(lineSplit[0], lineSplit[1]);
		}
		return data;
	}
	
	
	
	
	

	public String serverReport() {
		return makeStringCgiRequest("serverreport.cgi", null);
	}

	public String getImageHttpURL(ImageFormat format, ImageConfig... properties){
		return format.getImageHttpURL(this, properties);
	}
	
	public String getImageRstlURL(ImageFormat format, ImageConfig... properties){
		return format.getImageRstlURL(this, properties);
	}
}
