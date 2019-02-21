package example;// Copyright 2010-2018 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Minimal example to call the GLOP solver.
// [START program]
import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;

/** Minimal Linear Programming example to showcase calling the solver.*/
public class SimpleLpProgram {
  static {
    System.loadLibrary("jniortools");
  }

  public static void main(String[] args) throws Exception {
    // Create the linear solver with the GLOP backend.
    MPSolver solver =
        new MPSolver("SimpleLpProgram", MPSolver.OptimizationProblemType.GLOP_LINEAR_PROGRAMMING);

    // Create the variables x and y.
    MPVariable x = solver.makeNumVar(0.0, 1.0, "x");
    MPVariable y = solver.makeNumVar(0.0, 2.0, "y");

    // Create a linear constraint, 0 <= x + y <= 2.
    MPConstraint ct = solver.makeConstraint(0.0, 2.0, "ct");
    ct.setCoefficient(x, 1);
    ct.setCoefficient(y, 1);

    // Create the objective function, 3 * x + y.
    MPObjective objective = solver.objective();
    objective.setCoefficient(x, 3);
    objective.setCoefficient(y, 1);
    objective.setMaximization();

    // Call the solver and display the results.
    solver.solve();
    System.out.println("Solution:");
    System.out.println("x = " + x.solutionValue());
    System.out.println("y = " + y.solutionValue());
  }
}
// [END program]
