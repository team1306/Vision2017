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
    Mat hsv = new Mat();
	
    boolean hasBBOX = false;
	
    while (true) {
	long frameTime = imageSink.grabFrame(inputImage);
        if (frameTime == 0) continue;
	VisionData data=new VisionData(inputImage.t());
	ArrayList<MatOfPoint> final_contours;
	ArrayList<Rect> bounding_box;
	double yaw, dist, angle;
	final_contours=data.processImage();
	bounding_box=data.getBoundingBox(final_contours);
	// Sorting (We want the top tape, not the bottom one
	if (bounding_box.size() > 0) {
		java.util.Collections.sort(bounding_box, new Comparator<Rect>() {
			@Override
			public int compare(Rect bbox1, Rect bbox2)
			{
		    	return  Double.compare(bbox1.y,bbox2.y);
			}
	    	});
		yaw=data.getYaw1(bounding_box.get(0));
		dist=data.getDistance(bounding_box.get(0));
		angle=data.getAngle(dist);
		table.putNumber("yaw", yaw);
		table.putNumber("dist", dist);
		table.putNumber("angle", angle);
		//table.putNumber("bboxx", bounding_box.get(0).x);
		//table.putNumber("bboxy", bounding_box.get(1).y);
		table.putBoolean("seeTarget", true);
		System.out.println("YAW:" + yaw);
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
	int LOGITECH_RES_WIDTH = 720;
	int LOGITECH_HORIZ_ANGLE = 45;
	int UPPER_TAPE_WIDTH = 4;
	double LOGITECH_FOCAL_LENGTH = 0.15748;
	int TOWER_HEIGHT = 88;

	Pipeline pipeline; // This goes to the GRIP pipeline that does all the work
	Mat image_process; // Input image

	public ArrayList<MatOfPoint> processImage() {
		ArrayList<MatOfPoint> final_contours; // Contours that GRIP gives at the end
		pipeline.process(image_process);
		final_contours = pipeline.filterContoursOutput(); // Get GRIP output
		return final_contours;
	}

	public ArrayList<Rect> getBoundingBox(ArrayList<MatOfPoint> contours) {
		ArrayList<Rect> bbox= new ArrayList<Rect>(); //Bbox stuff

		for (int i = 0; i < contours.size(); i++) {
			bbox.add(Imgproc.boundingRect(contours.get(i)));
		}
		return bbox;
	}

	public double getYaw(Rect upper_tape) {
		double yaw;
		//This takes the width of the image in pixels and divides it by the horizontal angle,
		//giving the number of degrees per pixel
		double deg_per_pixel= LOGITECH_RES_WIDTH / LOGITECH_HORIZ_ANGLE;
		double center_col = (LOGITECH_RES_WIDTH - 1) / 2; //Center column of image, -1 is to account for 0 indexing

		int contour_x = upper_tape.x;//column of center of contour (tape)
		yaw = (contour_x - center_col) * deg_per_pixel;
		//(contour_x - center_col) is the horizontal distance between 
		//the center of the image and the center of the target in pixels, so 
		//you multiply that by the degrees/pixel conversion factor to get degrees

		return yaw;
	}

	public double getDistance(Rect upper_tape) {
		double apparent_width=upper_tape.width;
		double horiz_distance= (UPPER_TAPE_WIDTH * LOGITECH_FOCAL_LENGTH) / apparent_width;
		return horiz_distance;

	}
	public double getAngle(double horiz_dist) {
		return (Math.toDegrees(Math.atan(TOWER_HEIGHT/horiz_dist)));
	}
}
