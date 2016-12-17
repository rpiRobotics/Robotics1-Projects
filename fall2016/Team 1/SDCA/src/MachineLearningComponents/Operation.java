package MachineLearningComponents;

import MachineLearningFramework.AbstractMethod;
import MachineLearningFramework.ReflectionHelper;

/**
 * An operation is any action done to a two elements returning a variable.
 * For the purposes of this program. Input = two node/variables (1 of each is
 * fine)
 * Output = a variable.
 * 
 * @author Ryan Feldhausen
 *
 */
public class Operation {
	/**
	 * prefix of the methods that the genetic algorithm will call.
	 */
	private static final String reflectionPrefix = "act";
	/**
	 * This class abstracts away the maintenance for adding/removing operations
	 * for the
	 * genetic algorithm.
	 */
	private ReflectionHelper operationReflectionHelper;

	private int reflectionMethodCount;

	public Operation() {
		ReflectionHelper operationReflectionHelper = new ReflectionHelper(this, reflectionPrefix);
		int reflectionMethodCount = operationReflectionHelper.getMethodCount();

		this.operationReflectionHelper = operationReflectionHelper;
		this.reflectionMethodCount = reflectionMethodCount;
	}

	public int getReflectionMethodCount() {
		return reflectionMethodCount;
	}

	public AbstractMethod characterizeMethodX(int x) {
		AbstractMethod characterizedMethod = operationReflectionHelper.characterizeMethodX(x);
		return characterizedMethod;
	}

	// ----------------------------------------------------------
	// ------------MACHINE LEARNING METHODS----------------------
	// ----------------------------------------------------------

	public int actAdd(int a, int b) {
		return a + b;
	}

	public int actSubtract(int a, int b) {
		return a - b;
	}

	public int actMultiply(int a, int b) {
		return a * b;
	}

	public int actDivide(int a, int b) {
		return a / b;
	}

	public int actRemainder(int a, int b) {
		return a % b;
	}

	public int actALessThanB(int a, int b) {
		if (a < b) {
			return 1;
		} else {
			return 0;
		}
	}

	public int actAGreaterThanB(int a, int b) {
		if (a > b) {
			return 1;
		} else {
			return 0;
		}
	}

	public int actLcd(int a, int b) {
		return (a * b / actGcd(a, b));
	}

	public int actGcd(int a, int b) {
		if (a == 0) {
			return b;
		}
		while (b != 0) {
			if (a > b) {
				a = a - b;
			} else {
				b = b - a;
			}
		}
		return a;
	}
}
