package root;
import java.util.ArrayList;

/**
 * A frame contains all pixel data for a single instant of time.
 * @author Ryan Feldhausen
 *
 */
public class Frame {
	/**
	 * A nested array containing the pixel data for the frame.
	 * This is structured as the outer array = x. Inner array = y.
	 * <br> Origin is bottom left
	 * <br>(y)
	 * <br>|
	 * <br>|
	 * <br>o-----(x)
	 */
	private ArrayList<ArrayList<Pixel>> pixelGrid;
	
	/**
	 * Create a frame given a nested array of pixels
	 * @param pixelGrid The nested array ArrayList(x)ArrayList(y)Pixel
	 */
	public Frame(ArrayList<ArrayList<Pixel>> pixelGrid){
		this.pixelGrid = pixelGrid;
	}
	
	/**
	 * Returns the pixel at coordinate x, y.
	 * The bottom left of the screen is (0,0). Y up. X right.
	 * @param x int The horizontal position of the pixel (increases right)
	 * @param y int The vertical position of the pixel (increases up)
	 * @return Pixel at the given location
	 */
	public Pixel getPixelAt(int x, int y){
		return pixelGrid.get(x).get(y);
	}
	
	/**
	 * This function will return the width of a frame, or the resolution in the x axis.
	 * @return int the width
	 */
	public int getFrameWidth(){
		return pixelGrid.size();
	}
	
	/**
	 * This function will return the height of a frame, or the resolution of the y axis.
	 * @return int the height
	 */
	public int getFrameHeight(){
		return pixelGrid.get(0).size();
	}
	
}
