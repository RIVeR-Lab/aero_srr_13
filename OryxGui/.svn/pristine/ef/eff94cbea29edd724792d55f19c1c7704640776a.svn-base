package gui;

import org.ros.message.sensor_msgs.Image;

import com.googlecode.javacpp.BytePointer;
import com.googlecode.javacv.cpp.opencv_core.CvMat;
import static com.googlecode.javacv.cpp.opencv_core.*;
import com.googlecode.javacv.cpp.opencv_core.IplImage;

import java.awt.SystemColor;
import java.awt.image.BufferedImage;
import java.awt.image.ComponentSampleModel;
import java.awt.image.DataBuffer;
import java.awt.image.DataBufferByte;
import java.awt.image.Raster;
import java.awt.image.SampleModel;
import java.nio.ByteBuffer;



/**
 * Static class for converting ROS Image messages to JavaCV and Java BufferedImages
 * @author joe
 *
 */
public class JavaCVBridge {
	
	public static Image cvToImageMessage(IplImage img, String encoding){
	
		CvMat header = new CvMat(), cvm;
		
		Image dest = new Image();
		int[] coi = null;
		cvm = cvGetMat(img, header, coi, 0);
		if (encoding ==null) encoding = "passthrough";		
		dest.encoding=encoding;
		dest.width = cvm.cols();
		dest.height=cvm.rows();
		dest.step=cvm.step();
		
		ByteBuffer buff = img.getByteBuffer();
		byte[] b = new byte[buff.remaining()];
		if(buff.hasArray())System.out.println("Buffer has Array");
		
		buff.get(b,0,b.length);
		dest.data= b;		
		return dest;
	}
	
	public static BufferedImage messageToBufferedImage(Image imgMsg){
		int width = (int) imgMsg.width;
		int height = (int) imgMsg.height;		
		DataBuffer buffer = new DataBufferByte(imgMsg.data, width*height);
        SampleModel sampleModel = new ComponentSampleModel(DataBuffer.TYPE_BYTE, width, height, 3, width*3, new int[]{2,1,0});
        Raster raster = Raster.createRaster(sampleModel, buffer, null);
        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
        image.setData(raster);
        return image;
		
		
	}
	
	
	public static IplImage imageMessageToCv(Image imgMsg, int encoding){
		CvMat cvmHeader = new CvMat();
		IplImage returnImage = new IplImage();
		BytePointer bp=	new BytePointer(imgMsg.data);	
		 cvInitMatHeader(cvmHeader,(int) imgMsg.height,(int) imgMsg.width, encoding, bp, (int)imgMsg.step);
		 cvGetImage(cvmHeader, returnImage);
		 return returnImage;
	}
	
}
