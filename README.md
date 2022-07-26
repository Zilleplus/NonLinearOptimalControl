# Simple Optimal Control framework

## What is this?
- A simple implementation of non-linear optimal control.
- Self contained -> uses only stl, no external libs.
- It has it's own automatic backward differentiation implemenation.
- It does not contain a non-linear solver, those you have to find somewhere else.

## How to use?
1. Define behavior/constraints
2. Solve the problem using your own non-linear solver.

Using the tests/optimalControlTests/cart.cpp as example:

### Continous behavior of the system

The behavior of a 2D pointmass with 
- mass m
- x-position: $x_0$
- y-position: $x_1$
- x-velocity: input $u_0$
- y-velocity: input $u_1$

$$
f = 
\begin{cases}
\dot{x_0} & = x_2 \\
\dot{x_1} & = x_3 \\
\dot{x_2} & = u_0/m \\
\dot{x_3} & = u_1/m \\
\end{cases}
$$

Translated in code:
```
// The system behavior :
// dot_x0 = x_2
// dot_x1 = x_3
// dot_x2 = u_0 / m
// dot_x3 = u_1 / m
class CartBehavior : public optimalControl::ContiniousSystemBehavior
{
	virtual multi_expr dot(multi_expr& current_state, multi_expr& current_input) override {
		using namespace runtimeAd;
		const double m = 1;
		auto constant_m = runtimeAd::Constant(m);

		multi_expr next_state;
		next_state.push_back(current_state[2]);
		next_state.push_back(current_state[3]);
		next_state.push_back(current_input[0] / constant_m);
		next_state.push_back(current_input[1] / constant_m);

		return next_state;
	}
};
```

### Convert to Discrete system
Either create a class that inherits from optimalControl::DiscreteSystemBehavior and implement it manually.
Or integrate the continous system using an integrator:
```
double step_size = 1e-1;
auto discrete_system = optimalControl::IntegrateWithExplicitEuler(cont_system, step_size);
```

### Create optimal control problem
Use the single shot definition to define the optimal control problem.
Set the state constraint as a weighted L2Norm (4 states in this case, so 4 weights are required)
```
optimalControl::SingleShot::InitParams params;
params.num_of_input = 2; // 2D motoro speed
params.num_of_states = 4; // 2D position + speed of body
params.num_of_steps_horizon = 10;
params.system = &discrete_system;
optimalControl::WeightedL2Constraint state_constraint({ 10.0, 10.0, 0.0, 0.0 });
params.state_constraint = state_constraint;

```

### set the run-time variables on the optimal control problem.
The start state, and reference state are mutable. Set these to the desired values.
```
const int s = problem.GradientSize();

// Start at origin at stand still.
std::vector<double> init_state = { 0,0, 0,0 };
problem.SetInitState(init_state);

// End at (10, 10) at standing still.
std::vector<double> ref_state = { 10, 10 , 0, 0 };
problem.SetRefState(ref_state);
```

Create the vector to save the results.
```
std::vector<double> current_input_horizon(s);
for (int i = 0; i < std::size(current_input_horizon); ++i)
{
	current_input_horizon[i] = 0;
}
```

### Solve the non-linear problem
You can find the gradient/cost function by calling problem.CostGradient(input, output\_gradient).
Minimize the costs of this function to solve the optimal control problem.
```
std::vector<double> output_gradient(s);
const double current_cost = problem.CostGradient(current_input_horizon, output_gradient);
```

### Try it yourself
test/cart.cpp contains an implementation solving the 2D point mass problem using a gradient descent.