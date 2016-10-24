package deliberate;

/* import table */

import logist.agent.Agent;
import logist.behavior.DeliberativeBehavior;
import logist.plan.Plan;
import logist.simulation.Vehicle;
import logist.task.Task;
import logist.task.TaskDistribution;
import logist.task.TaskSet;
import logist.topology.Topology;
import logist.topology.Topology.City;

/**
 * An optimal planner for one vehicle.
 */
@SuppressWarnings("unused")
public class DeliberativeTemplate implements DeliberativeBehavior {

	/* Environment */
	private Topology topology;
	private TaskDistribution td;
	/* the properties of the agent */
	private Agent agent;
	/* the planning class */
	private Algorithm algorithm;
    // the tasks that the agent is carrying (useful when new plan needs to be generated)
    private TaskSet carrying;

	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {

        this.topology = topology;
		this.td = td;
		this.agent = agent;

		// initialize the planner
		String algorithmName = agent.readProperty("algorithm", String.class, "ASTAR");

        // Throws IllegalArgumentException if algorithm is unknown
		this.algorithm = Algorithm.valueOf(algorithmName.toUpperCase());


	}


	@Override
	public Plan plan(Vehicle vehicle, TaskSet tasks) {

        return carrying==null? new Solver(vehicle, tasks).execute(algorithm):
                new Solver(vehicle, tasks, carrying).execute(algorithm);
	}


	
	private Plan naivePlan(Vehicle vehicle, TaskSet tasks) {
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);

		for (Task task : tasks) {
			// move: current city => pickup location
			for (City city : current.pathTo(task.pickupCity))
				plan.appendMove(city);

			plan.appendPickup(task);

			// move: pickup location => delivery location
			for (City city : task.path())
				plan.appendMove(city);

			plan.appendDelivery(task);

			// set current city
			current = task.deliveryCity;
		}
		return plan;
	}

    @Override
	public void planCancelled(TaskSet carriedTasks) {

        if (!carriedTasks.isEmpty()) {
			// This cannot happen for this simple agent, but typically
			// you will need to consider the carriedTasks when the next
			// plan is computed.
            carrying = carriedTasks;
		}
	}

    enum Algorithm {BFS, ASTAR}
}
