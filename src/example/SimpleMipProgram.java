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

// Minimal example to call the MIP solver.
// [START program]
import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;

/** Minimal Mixed Integer Programming example to showcase calling the solver. */
public class SimpleMipProgram {
  static {
    System.loadLibrary("jniortools");
  }

  public static void main(String[] args) throws Exception {
    // [START solver]
    // Create the linear solver with the CBC backend.
    MPSolver solver = new MPSolver(
        "SimpleMipProgram", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
    // [END solver]

    // [START variables]
    double infinity = MPSolver.infinity();
    // x and y are integer non-negative variables.
    MPVariable x = solver.makeIntVar(0.0, infinity, "x");
    MPVariable y = solver.makeIntVar(0.0, infinity, "y");

    System.out.println("Number of variables = " + solver.numVariables());
    // [END variables]

    // [START constraints]
    // x + 7 * y <= 17.5.
    MPConstraint c0 = solver.makeConstraint(-infinity, 17.5, "c0");
    c0.setCoefficient(x, 1);
    c0.setCoefficient(y, 7);

    // x <= 3.5.
    MPConstraint c1 = solver.makeConstraint(-infinity, 3.5, "c1");
    c1.setCoefficient(x, 1);
    c1.setCoefficient(y, 0);

    System.out.println("Number of constraints = " + solver.numConstraints());
    // [END constraints]

    // [START objective]
    // Maximize x + 10 * y.
    MPObjective objective = solver.objective();
    objective.setCoefficient(x, 1);
    objective.setCoefficient(y, 10);
    objective.setMaximization();
    // [END objective]

    // [START solve]
    final MPSolver.ResultStatus resultStatus = solver.solve();
    // Check that the problem has an optimal solution.
    if (resultStatus != MPSolver.ResultStatus.OPTIMAL) {
      System.err.println("The problem does not have an optimal solution!");
      return;
    }
    // Verify that the solution satisfies all constraints (when using solvers
    // others than GLOP_LINEAR_PROGRAMMING, this is highly recommended!).
    if (!solver.verifySolution(/*tolerance=*/1e-7, /*log_errors=*/true)) {
      System.err.println("The solution returned by the solver violated the"
          + " problem constraints by at least 1e-7");
      return;
    }
    // [END solve]

    // [START print_solution]
    System.out.println("Solution:");
    System.out.println("Objective value = " + objective.value());
    System.out.println("x = " + x.solutionValue());
    System.out.println("y = " + y.solutionValue());
    // [END print_solution]

    // [START advanced]
    System.out.println("\nAdvanced usage:");
    System.out.println("Problem solved in " + solver.wallTime() + " milliseconds");
    System.out.println("Problem solved in " + solver.iterations() + " iterations");
    System.out.println("Problem solved in " + solver.nodes() + " branch-and-bound nodes");
    // [END advanced]
  }
}
// [END program]
