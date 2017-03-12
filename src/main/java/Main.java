import edu.wpi.first.wpilibj.networktables.*;
import edu.wpi.first.wpilibj.tables.*;
import org.opencv.core.Rect;
import java.util.ArrayList;
import java.util.Comparator;
import edu.wpi.cscore.*;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;

public class Main {
  public static void main(String[] args) {
	int LOGITECH_RES_WIDTH = 720;
	int LOGITECH_HORIZ_ANGLE = 45;
	int UPPER_TAPE_WIDTH = 4;
	double LOGITECH_FOCAL_LENGTH = 0.15748;
	int TOWER_HEIGHT = 88;
	  
    NetworkTable.setClientMode();
    NetworkTable.setTeam(1306);
    NetworkTable.initialize();
    NetworkTable table = NetworkTable.getTable("1306");
    
    // This is the network port you want to stream the raw received image to
    // By rules, this has to be between 1180 and 1190, so 1185 is a good choice
    int streamPort = 1185;

    // This stores our reference to our mjpeg server for streaming the input image
    MjpegServer inputStream = new MjpegServer("MJPEG Server", streamPort);

    // This gets the image from a USB camera 
    // Usually this will be on device 0, but there are other overloads
    // that can be used
    UsbCamera camera = setUsbCamera(0, inputStream);
    // Set the resolution for our camera, since this is over USB
    camera.setResolution(1280,720);

    // This creates a CvSink for us to use. This grabs images from our selected camera, 
    // and will allow us to use those images in opencv
    CvSink imageSink = new CvSink("CV Image Grabber");
    imageSink.setSource(camera);

    // This creates a CvSource to use. This will take in a Mat image that has had OpenCV operations
    // operations 
    CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 1280, 720, 30);
    MjpegServer cvStream = new MjpegServer("CV Image Stream", 1186);
    cvStream.setSource(imageSource);

    // All Mats and Lists should be stored outside the loop to avoid allocations
    // as they are expensive to create
    Mat inputImage = new Mat();
    //Mat hsv = new Mat();
	//Mat image_process;
	Pipeline pipeline = new Pipeline();
	
	//ArrayList<MatOfPoint> final_contours;
	ArrayList<Rect> bounding_box;
	
    boolean hasBBOX = false;
	double yaw, dist, angle;
	
    while (true) {
	long frameTime = imageSink.grabFrame(inputImage);
        if (frameTime == 0) continue;
		
	//image_process=inputImage.t();
	//final_contours=processImage();
	bounding_box=getBoundingBox(processImage());
	
	// Sorting (We want the top tape, not the bottom one
	if (bounding_box.size() > 0) {
		java.util.Collections.sort(bounding_box, new Comparator<Rect>() {
			@Override
			public int compare(Rect bbox1, Rect bbox2)
			{
		    	return  Double.compare(bbox1.y,bbox2.y);
			}
	    	});
		//yaw=getYaw(bounding_box.get(0));
		//dist=getDistance(bounding_box.get(0));
		//angle=getAngle(getDistance(bounding_box.get(0)));
		table.putNumber("yaw", getYaw(bounding_box.get(0)));
		table.putNumber("dist", getDistance(bounding_box.get(0)));
		table.putNumber("angle", getAngle(getDistance(bounding_box.get(0))));
		//table.putNumber("bboxx", bounding_box.get(0).x);
		//table.putNumber("bboxy", bounding_box.get(1).y);
		table.putBoolean("seeTarget", true);
		System.out.println("YAW:" + getYaw(bounding_box.get(0)));
		
	} else {
		table.putBoolean("seeTarget", false);
		System.out.println("No bbox------------------------------------------------------------No bbox");
	}
	imageSource.putFrame(inputImage.t());
	//System.out.println("YAW: " + yaw);
	//System.out.println(dist);
	//System.out.println(angle);
      //System.out.println(table.putNumber("dist", dist));
    }
  }
  private static UsbCamera setUsbCamera(int cameraId, MjpegServer server) {
    // This gets the image from a USB camera 
    // Usually this will be on device 0, but there are other overloads
    // that can be used
    UsbCamera camera = new UsbCamera("CoprocessorCamera", cameraId);
    server.setSource(camera);
    return camera;
  }

	//ArrayList<MatOfPoint> final_contours; // Contours that GRIP gives at the end
	public ArrayList<MatOfPoint> processImage() {
		pipeline.process(inputImage.t());
		//final_contours = pipeline.filterContoursOutput(); // Get GRIP output
		return pipeline.filterContoursOutput();
	}

	ArrayList<Rect> bbox= new ArrayList<Rect>();
	public ArrayList<Rect> getBoundingBox(ArrayList<MatOfPoint> contours) {
		for (int i = 0; i < contours.size(); i++) {
			bbox.add(Imgproc.boundingRect(contours.get(i)));
		}
		return bbox;
	}

	public double getYaw(Rect upper_tape) {
		System.out.println("Upper tape X:" + upper_tape.x + "Upper tape Y:" + upper_tape.y + "imgCenter:" + (LOGITECH_RES_WIDTH/2));
		return Math.toDegrees(Math.atan((upper_tape.x - (LOGITECH_RES_WIDTH/2)) / LOGITECH_FOC_LENGTH_PX));
	}

	public double getDistance(Rect upper_tape) {
		//double apparent_width=upper_tape.width;
		//double horiz_distance= (UPPER_TAPE_WIDTH * LOGITECH_FOCAL_LENGTH) / apparent_width;
		return (UPPER_TAPE_WIDTH * LOGITECH_FOCAL_LENGTH) / upper_tape.width;

	}
	public double getAngle(double horiz_dist) {
		return (Math.toDegrees(Math.atan(TOWER_HEIGHT/horiz_dist)));
	}
}
