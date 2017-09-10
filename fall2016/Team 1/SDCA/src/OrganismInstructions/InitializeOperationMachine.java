package OrganismInstructions;

public class InitializeOperationMachine extends GenericInstruction {
	private int identifier;

	public InitializeOperationMachine(int identifier) {
		this.identifier = identifier;
	}

	public String getPythonInstruction() {
		// x = Operation()
		String x = "i" + String.valueOf(identifier);
		return x + " = Operation()\n";
	}

}
