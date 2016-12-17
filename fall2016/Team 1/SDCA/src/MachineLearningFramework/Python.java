package MachineLearningFramework;

import org.python.util.PythonInterpreter;

public class Python {
	private static final String python_folder = "/python/";
	private static final String python_import_prefix = "import sys\n" + "sys.path.append('" + python_folder + "')\n";
	private static String imports = "";
	private PythonInterpreter py = null;

	public Python() {
		this.py = new PythonInterpreter();
		addImport("Frame");
		addImport("Main");
		addImport("Node");
		addImport("NodeGenerator");
		addImport("Pixel");
		addImport("BoundedIntFrameHeight");
		addImport("BoundedIntFrameWidth");
		addImport("Operation");
		runImports();
	}

	public void addImport(String name) {
		imports = imports + "from " + name + " import " + name + "\n";
	}

	public void runCode(String code) {
		System.out.print(code);
		// py.exec(python_import_prefix + imports + code);
		// py.exec(imports);
		py.exec(code);
	}

	public void runImports() {
		// System.out.print(imports);
		py.exec(python_import_prefix + imports);
	}
}
