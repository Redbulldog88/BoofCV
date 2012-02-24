/*
 * Copyright (c) 2011-2012, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package boofcv.numerics.optimization.impl;

import boofcv.numerics.optimization.OptimizationException;
import org.ejml.alg.dense.linsol.LinearSolver;
import org.ejml.alg.dense.mult.VectorVectorMult;
import org.ejml.data.DenseMatrix64F;

/**
 * <p>
 * Modification of {@link LevenbergDampened} which incorporates the insight of Marquardt.  The insight
 * was to use the function's curvature information to increase dampening along directions with
 * a larger gradient.  In practice this method seems to do better on nearly singular systems.
 * </p>
 *
 * <p>
 * The step 'x' is computed using the following formula:
 * [J(k)'*J(k) + &mu;*diag(J(k)'*J(k))]x = -g = -J'*f<br>
 * where J is the Jacobian, &mu; is the damping coefficient, g is the gradient,
 * f is the functions output.
 * </p>
 *
 * <p>
 * The linear solver it uses is specified in the constructor.  Cholesky based solver will be the fastest
 * but can fail if the J(x)'J(x) matrix is nearly singular.  In those situations a pseudo inverse type
 * solver should be used, which is immune to that type of problem but much more expensive.
 * </p>
 *
 * @author Peter Abeles
 */
public class LevenbergMarquardtDampened extends LevenbergDampened {

	/**
	 * Specifies termination condition and linear solver.  Selection of the linear solver an effect
	 * speed and robustness.
	 *
	 * @param solver		   Linear solver. Cholesky or pseudo-inverse are recommended.
	 * @param initialDampParam Initial value of the dampening parameter.  Tune.. try 1e-3;
	 * @param absoluteErrorTol Absolute convergence test.
	 * @param relativeErrorTol Relative convergence test based on function magnitude.
	 */
	public LevenbergMarquardtDampened(LinearSolver<DenseMatrix64F> solver, double initialDampParam, double absoluteErrorTol, double relativeErrorTol) {
		super(solver, initialDampParam, absoluteErrorTol, relativeErrorTol);
	}

	/**
	 * Solves the linear system to find the change in x
	 */
	@Override
	protected boolean solveForXDelta() {
		// add dampening parameter
		for( int i = 0; i < N; i++ ) {
			int index = B.getIndex(i,i);
			B.data[index] = (1+dampParam)*Bdiag.data[i];
		}

		// compute the change in step.
		if( !solver.setA(B) ) {
			throw new OptimizationException("Singularity encountered.  Try a more robust solver line pseudo inverse");
		}
		// solve for change in x
		solver.solve(g, xdelta);

		return true;
	}

	/**
	 * compute the change predicted by the model
	 *
	 * m_k(0) - m_k(p_k) = -g_k'*p - 0.5*p'*B*p
	 * (J'*J+mu*diag(J'*J))*p = -J'*r = -g
	 *
	 * @return predicted reduction
	 */
	@Override
	protected double predictedReduction( DenseMatrix64F p, double mu ) {

		double p_dot_g = VectorVectorMult.innerProd(p,g);
		double p_JJ_p = 0;
		for( int i = 0; i < N; i++ )
			p_JJ_p += p.data[i]*Bdiag.data[i]*p.data[i];

		// The variable g is really the negative of g
		return 0.5*(p_dot_g + mu*p_JJ_p);

	}
}
