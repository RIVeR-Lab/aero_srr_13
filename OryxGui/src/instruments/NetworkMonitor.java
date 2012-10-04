package instruments;
import java.awt.*;
import java.io.*;
import java.text.DecimalFormat;
import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;

import javax.swing.*;


public class NetworkMonitor extends JPanel implements Runnable {

    private ArrayList<String> lines;
    double timeStep = .5;
    int buffer = 160;
    double prevBytesReceived =0,prevBytesSent=0;
    ArrayBlockingQueue<Double> receivedQueue;
    
    public NetworkMonitor() {
        lines = new ArrayList<String>();
        receivedQueue = new ArrayBlockingQueue<Double>(buffer);
        while(receivedQueue.remainingCapacity()>0){
        	receivedQueue.add(new Double(0.0));
        }
        setPreferredSize(new Dimension(232,200));
        setMaximumSize(new Dimension(232,200));
        new Thread(this).start();
    }

    @Override
    public void run() {
        try {
            while (true) {
                String sContents = readFile();
                StringTokenizer tok = new StringTokenizer(sContents, "\n");
                lines = new ArrayList<String>();
                int x = 0;

                while (tok.hasMoreTokens()) {
                    String line = tok.nextToken();
                    if (x++ < 2) {//skip headers
                        continue;
                    }
                    lines.add(line);
                }
                repaint();
                Thread.sleep((long) (1000*timeStep));//half a second
            }
        } catch (Exception exc) {
            exc.printStackTrace();
        }
    }

    @Override
    public void paint(Graphics gr) {
        super.paint(gr);
        Graphics2D g = (Graphics2D) gr;
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setFont(new Font(Font.DIALOG_INPUT, Font.PLAIN, 14));
        try {
        	if(lines.size() == 4)
        		//If an external wireless modem is available, use it
        		drawLine(g,lines.get(3));
        	else
        		//Otherwise, use eth0
        		drawLine(g,lines.get(1));
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
     
  
    }

    /**
     * This method reads the file containing network information and puts it in a single string
     * @return String containing network bandwidth information
     */
    private static final String readFile() {
        try {
            File file = new File("/proc/net/dev");
            StringBuilder sb = new StringBuilder();
            byte[] bytes = new byte[1024];
            int read = 0;
            InputStream in = new FileInputStream(file);
            while ((read = in.read(bytes)) != -1) {
                sb.append(new String(bytes, 0, read));
            }
            in.close();
            return sb.toString();
        } catch (Exception exc) {
            exc.printStackTrace();
        }
        return null;
    }

    /**
     * Draws a line indicating how much bandwidth has been used by the network interface over period of time
     * @param g A Graphics object
     * @param line String containing the bandwidth information for the interface
     * @throws InterruptedException
     */
    private void drawLine(Graphics2D g, String line) throws InterruptedException {
    	//Split the string into separate words
        String[] words = line.split("\\s+");
        String sInterface = words[words[0].length() > 0 ? 0 : 1];
        int index = sInterface.indexOf(':');
        boolean jump = index != sInterface.length()-1;
        //Find the location corresponding to TX/RX bandwidth in bytes
        String receivedBytes = jump ? sInterface.substring(index+1) : words[2];
        String sentBytes = jump ? words[9] : words[10];
    
        double numBytesReceived=Double.parseDouble(receivedBytes);
        double numBytesTransmit=Double.parseDouble(sentBytes);
        
        DecimalFormat decimalFormat = new DecimalFormat( "#########.00" );
        
        double receivedRate = (new Double(decimalFormat.format((numBytesReceived - prevBytesReceived)/timeStep/1024))).doubleValue();
        double transmitRate = (new Double(decimalFormat.format((numBytesTransmit - prevBytesSent)/timeStep/1024))).doubleValue();
        
        prevBytesSent=numBytesTransmit;
        prevBytesReceived=numBytesReceived;
       
        receivedBytes = Double.toString(receivedRate)+" kB/s";
        sentBytes = Double.toString(transmitRate)+" kB/s";
        
        if(receivedQueue.remainingCapacity() ==0){
        	receivedQueue.take();
        }
        receivedQueue.add(receivedRate);
       Object[] history = receivedQueue.toArray();
       int i=0, highestVal=1;
       
       //Find highest value
       for (Object object : history) {
    	   int value = (int)((Double)object).doubleValue()*1024;
    	   if(value > 10000000) value = 0;
    	   if(value > highestVal){
    		   highestVal = value;
    	   }
		   
       }
    
       double xScale = ((double)(this.getWidth()))/((double)(buffer));
       double yScale = findScale(highestVal/1024);
       boolean first = true;
       int prevVal = 0;
       
       float dash[] = { 10.0f };
       Stroke oldStroke = g.getStroke();
       g.setStroke(new BasicStroke(1.0f, BasicStroke.CAP_BUTT,
        BasicStroke.JOIN_MITER, 10.0f, dash, 0.0f));
       
       //Draw and label grids
       for(int j=0; j< 5; j++){
    	   g.drawString(new String(j*highestVal/1024/5 + "kB/s"), 0, (this.getHeight()-j*(this.getHeight()/5)));
    	   g.drawLine(0, (this.getHeight()-j*(this.getHeight()/5)), this.getWidth(), (this.getHeight()-j*(this.getHeight()/5)));
       }
   		   
   	g.setClip(60, 0, this.getWidth(), this.getHeight());	   
   	
   	
	for(int j=0; j< 8; j++){
   			g.drawLine((j*this.getWidth())/8+2, 0, (j*this.getWidth())/8+2,this.getHeight());
   		}
    g.setStroke(oldStroke);
    g.setColor(Color.RED);
       // Draw the line indicating bandwidth
       for (Object object : history) {
    	   int value = (int)((Double)object).doubleValue()*1024;
    	   if(first){
    		   prevVal=value;
    		   first= false;
    	   }
   		   g.drawLine((int)(i*xScale), (this.getHeight()-(int)(prevVal/yScale)), (int)((i+1)*xScale),(this.getHeight()-(int)(value/yScale)));
   		   i++;

   		   prevVal = value;
       }
       
    }
    
    /**
     * This finds a scaler value to automatically scale the Y axis to best display bandwidth
     * @param val The highest bandwidth in the bandwidth history
     * @return The scale for the Y axis
     */
    public int findScale(int val){
    	int scale = 5;
    	if(val>4)scale = 10;
    	if(val>8)scale = 20;
    	if(val>16)scale = 40;
    	if(val>32)scale = 80;
    	if(val>64)scale = 160;
    	if(val>128)scale = 320;
    	if(val>256)scale = 640;
    	if(val>512)scale = 1280;
    	if(val>1024)scale = 2560;
    	if(val>2048)scale = 5120;
    	return scale*1024/this.getHeight();
    }
}  