package axiscamera.util;

import java.io.IOException;
import java.io.InputStream;
import java.io.StringWriter;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLConnection;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.commons.codec.binary.Base64;
import org.apache.commons.io.IOUtils;

import axiscamera.AxisCameraException;
import axiscamera.AxisCameraResponseException;

public class HttpDevice {

	private final String root;
	private final String username;
	private final String password;

	public HttpDevice(String root, String username, String password) {
		this.root = root;
		this.username = username;
		this.password = password;
	}

	public String buildURLString(String protocal, String command, Map<String, String> args) {
		String queryString = "";
		boolean firstArg = true;
		if (args != null) {
			queryString = "?";
			for (Entry<String, String> entry : args.entrySet()) {
				if (!firstArg)
					queryString += "&";
				queryString += entry.getKey() + "=" + entry.getValue();
				firstArg = false;
			}
		}
		
		String userPass = "";
		if(username!=null && password!=null)
			userPass = username+":"+password+"@";

		return protocal+"://"+userPass + root + "/" + command  + queryString;
	}

	public URL buildURL(String command, Map<String, String> args)
			throws MalformedURLException {
		return new URL(buildURLString("http", command, args));
	}

	public URLConnection makeRequest(URL url) throws IOException {
		System.out.println("Making request to " + url);
		URLConnection connection = url.openConnection();
		String authString = Base64
				.encodeBase64URLSafeString((username + ":" + password)
						.getBytes());
		connection.setRequestProperty("Authorization", "Basic " + authString);
		connection.connect();
		try {
			if(((HttpURLConnection)connection).getResponseCode()>=300)
				throw new AxisCameraResponseException("Error retrieving image. Server returned HTTP status: "+((HttpURLConnection)connection).getResponseCode());
		} catch (IOException e) {
			throw new AxisCameraException("Error retrieving image", e);
		}
		return connection;
	}

	public String makeStringRequest(URL url) throws IOException {
		StringWriter writer = new StringWriter();
		URLConnection connection = makeRequest(url);
		InputStream is = connection.getInputStream();
		IOUtils.copy(is, writer, connection.getHeaderField("content-encoding"));
		is.close();
		return writer.toString();
	}

	public static boolean isIPAddress(String ipAddress) {
		return ipAddress
				.matches("[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}\\.[0-9]{1,3}");
	}

}
