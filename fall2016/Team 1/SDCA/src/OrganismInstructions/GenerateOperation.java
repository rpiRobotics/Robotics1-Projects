package OrganismInstructions;

import MachineLearningComponents.Operation;
import MachineLearningFramework.AbstractMethod;

public class GenerateOperation extends GenericInstruction {
	String retVal;
	String paramA;
	String paramB;
	int operationMachineIdentifier;
	int methodId;

	public GenerateOperation(int identifier, String retVal, String paramA, String paramB,
			int operationMachineIdentifier, int methodId) {
		this.identifier = identifier;
		this.retVal = retVal;
		this.paramA = paramA;
		this.paramB = paramB;
		this.operationMachineIdentifier = operationMachineIdentifier;
		this.methodId = methodId;
	}

	@Override
	public String getPythonInstruction() {
		Operation tempOperation = new Operation();
		AbstractMethod methodX = tempOperation.characterizeMethodX(methodId);
		String methodName = methodX.method.getName();
		// y = o.m(a,b)
		String y = retVal;
		String o = "i" + String.valueOf(operationMachineIdentifier);
		String m = methodName;
		String a = paramA;
		String b = paramB;
		return y + " = " + o + "." + m + "(" + a + "," + b + ")\n";
	}

}
