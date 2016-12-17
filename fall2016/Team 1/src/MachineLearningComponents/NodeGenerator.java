package MachineLearningComponents;

import java.util.ArrayList;

import BoundingClasses.BoundedIntFrameHeight;
import BoundingClasses.BoundedIntFrameWidth;
import MachineLearningFramework.AbstractMethod;
import MachineLearningFramework.ReflectionHelper;
import root.Frame;
import root.Pixel;

/**
 * NodeGenerator will generate nodes using any number of human created
 * methods. Examples could be "square"
 * 
 * @author Ryan Feldhausen
 *
 */
public class NodeGenerator {
	private static final String reflectionPrefix = "construct";
	private ReflectionHelper nodeGeneratorReflectionHelper = null;
	private int reflectionMethodCount;
	private Frame frame;

	public NodeGenerator(Frame frame) {
		this.frame = frame;

		ReflectionHelper nodeGeneratorReflectionHelper = new ReflectionHelper(this, reflectionPrefix);

		this.nodeGeneratorReflectionHelper = nodeGeneratorReflectionHelper;
		this.reflectionMethodCount = nodeGeneratorReflectionHelper.getMethodCount();
	}

	public int getReflectionMethodCount() {
		return reflectionMethodCount;
	}

	public AbstractMethod characterizeMethodX(int x) {
		AbstractMethod characterizedMethod = nodeGeneratorReflectionHelper.characterizeMethodX(x);
		return characterizedMethod;
	}

	// ----------------------------------------------------------
	// ------------MACHINE LEARNING METHODS----------------------
	// ----------------------------------------------------------

	public Node constructNodeRectangle(BoundedIntFrameWidth x1, BoundedIntFrameWidth x2, BoundedIntFrameHeight y1,
			BoundedIntFrameHeight y2) {
		// Obey the minimum bound
		if (x1.value < BoundedIntFrameWidth.minimum || x2.value < BoundedIntFrameWidth.minimum
				|| y1.value < BoundedIntFrameHeight.minimum || y2.value < BoundedIntFrameHeight.minimum) {
			return null;
			// Obey the maximum bound
		} else if (x1.value > BoundedIntFrameWidth.maximum || x2.value > BoundedIntFrameWidth.maximum
				|| y1.value > BoundedIntFrameHeight.maximum || y2.value > BoundedIntFrameHeight.maximum) {
			return null;
			// Properly constrained.
		} else {
			BoundedIntFrameWidth lowerX;
			BoundedIntFrameWidth upperX;
			BoundedIntFrameHeight lowerY;
			BoundedIntFrameHeight upperY;
			if (x1.value < x2.value) {
				lowerX = x1;
				upperX = x2;
			} else {
				lowerX = x2;
				upperX = x1;
			}

			if (y1.value < y2.value) {
				lowerY = y1;
				upperY = y2;
			} else {
				lowerY = y2;
				upperY = y1;
			}

			ArrayList<Pixel> pixelsInNode = new ArrayList<Pixel>();
			for (int x = lowerX.value; x < upperX.value; x++) {
				for (int y = lowerY.value; y < upperY.value; y++) {
					pixelsInNode.add(frame.getPixelAt(x, y));
				}
			}
			Node returnNode;
			try {
				returnNode = new Node(pixelsInNode);
				return returnNode;
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				return null;
			}
		}
	}

	/*
	 * Removed for the present future
	 * public Node constructNodeCircle(BoundedIntFrameWidth x,
	 * BoundedIntFrameHeight y, BoundedIntFrameWidth radius) {
	 * if (x.value < BoundedIntFrameWidth.minimum || y.value <
	 * BoundedIntFrameHeight.minimum) {
	 * return null;
	 * } else if (x.value > BoundedIntFrameWidth.maximum || y.value >
	 * BoundedIntFrameHeight.maximum) {
	 * return null;
	 * }
	 * // TODO fill this in
	 * ArrayList<Pixel> pixelsInNode = new ArrayList<Pixel>();
	 * pixelsInNode.add(frame.getPixelAt(100, 100));
	 * Node newNode;
	 * try {
	 * newNode = new Node(pixelsInNode);
	 * return newNode;
	 * } catch (Exception e) {
	 * // TODO Auto-generated catch block
	 * return null;
	 * }
	 * }
	 */
}
