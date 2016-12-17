package root;

import java.util.ArrayList;
import java.util.Random;

import MachineLearningFramework.Organism;
import MachineLearningFramework.Python;

/**
 * @version 0.18
 * @author Ryan Feldhausen
 *
 */
public class Main {
	public static final int x_resolution = 800;
	public static final int y_resolution = 600;
	public static final boolean print_test_output = false;

	/**
	 * Main function, this program is designed to run indefinitely.
	 * A normal shutdown should allow a normal startup to resume progress.
	 * 
	 * @param args
	 *            Not used. Leave Empty.
	 * @throws Exception
	 *             Pixel Component Out of Bounds
	 */
	public static void main(String[] args) throws Exception {
		// Tests!
		runTests(print_test_output);

		// Generate a given number of organisms
		Random rng = new Random();
		ArrayList<Organism> organisms = new ArrayList<Organism>();
		for (int x = 0; x < 1000; x++) {
			Organism newOrganism = new Organism(rng.nextInt(10000));
			newOrganism.define();
			organisms.add(newOrganism);
		}

		System.out.println("1000 Organisms Generated");

		// Start up the jython instance
		Python jythonInstance = new Python();
		String frameInit = "import random\n" + "Width = 800\n" + "Height = 600\n" + "row = []\n"
				+ "for x in range(0, Width):\n" + "    column = []\n" + "    for y in range(0, Height):\n"
				+ "        r = random.randint(0, 255)\n" + "        g = random.randint(0, 255)\n"
				+ "        b = random.randint(0, 255)\n" + "        column.append(Pixel(r, g, b, 255))\n"
				+ "    row.append(column)\n" + "frame = Frame(row)\n";

		jythonInstance.runCode(frameInit);

		System.out.println("Frame Initialized");

		for (int x = 0; x < organisms.size(); x++) {
			jythonInstance.runCode(organisms.get(x).getPython());
			jythonInstance.runCode("print(str(c2) + '\t' + str(c3))");
		}

		System.out.println("Finished");

	}

	/**
	 * runTests will run a number of simple "does it crash" tests.
	 * 
	 * @param printOutput
	 *            Show debugging print statements if true
	 * @throws Exception
	 */
	public static void runTests(boolean printOutput) throws Exception {
		Test testInstance = new Test();
		testInstance.testNode(printOutput);
		testInstance.testOperation(printOutput);
		testInstance.testNodeGenerator(printOutput);
		testInstance.testOrganism(printOutput);
	}
}
