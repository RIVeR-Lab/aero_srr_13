package axiscamera.ptz;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import axiscamera.AxisCamera;

public class PTZ {
	public static void controlPTZ(AxisCamera camera, Map<String, String> params) {
		Map<String, String> args = new HashMap<String, String>();
		args.put("camera", Integer.toString(camera.getCameraNumber()));
		args.putAll(params);
		camera.makeCgiRequest("com/ptz.cgi", args);
	}

	/*
	 * home up down left right upleft upright downleft downright stop
	 */
	public static void move(AxisCamera camera, String command) {
		controlPTZ(camera, Collections.singletonMap("move", command));
	}

	//1-9999
	public static void zoom(AxisCamera camera, int zoom){
		controlPTZ(camera, Collections.singletonMap("zoom", Integer.toString(zoom)));
	}
	//-9999 - 9999
	public static void jogZoom(AxisCamera camera, int zoom){
		controlPTZ(camera, Collections.singletonMap("rzoom", Integer.toString(zoom)));
	}
}
