package OrganismInstructions;

public abstract class GenericInstruction {
	int identifier;

	abstract public String getPythonInstruction();

	public int getIdentifier() {
		return identifier;
	}
}
