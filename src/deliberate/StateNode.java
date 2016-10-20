package deliberate;

import logist.task.Task;
import logist.task.TaskSet;
import logist.topology.Topology.City;

import java.util.HashSet;
import java.util.Set;

class StateNode {
    private City currentCity;
    private TaskSet avalaibleTasks, takenTasks;
    private Transition parentTransition;
    private double currentWeight;

    public StateNode(City currentCity, TaskSet avalaibleTasks, TaskSet takenTasks, Transition parentTransition, double currentWeight) {
        this.currentCity = currentCity;
        this.avalaibleTasks = avalaibleTasks;
        this.takenTasks = takenTasks;
        this.parentTransition = parentTransition;
        this.currentWeight = currentWeight;
    }

    public static StateNode initialNode(City current, TaskSet avalaible) {
        return new StateNode(current, avalaible, TaskSet.noneOf(avalaible), null, 0);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        StateNode stateNode = (StateNode) o;

        if (!currentCity.equals(stateNode.currentCity)) return false;
        if (!avalaibleTasks.equals(stateNode.avalaibleTasks)) return false;
        if (!takenTasks.equals(stateNode.takenTasks)) return false;
        return parentTransition != null ? parentTransition.equals(stateNode.parentTransition) : stateNode.parentTransition == null;

    }

    @Override
    public int hashCode() {
        int result = currentCity.hashCode();
        result = 31 * result + avalaibleTasks.hashCode();
        result = 31 * result + takenTasks.hashCode();
        result = 31 * result + (parentTransition != null ? parentTransition.hashCode() : 0);
        return result;
    }

    /**
     * Generate all the successor of a given state
     */
    private Set<StateNode> generateSuccessors(final double maximumLoad) {
        Set<StateNode> result = new HashSet<>();
        for (Transition.Type actionType : Transition.Type.values()) {
            switch (actionType) {

                case MOVE:
                    Set<City> destinationCities = generateCitiesOfInterest();
                    for (City d : destinationCities) {
                        Transition trans = new Transition(actionType, currentCity, d, null);
                        result.add(new StateNode(d, avalaibleTasks, takenTasks, trans, currentWeight));
                    }
                    break;

                case PICKUP:
                    for (Task t : avalaibleTasks) {
                        double potentialWeight = currentWeight + t.weight;

                        // agent can only pickup tasks in its current city and if its weight is lower than maximum load
                        if (t.pickupCity == currentCity && potentialWeight <= maximumLoad) {
                            Transition trans = new Transition(actionType, currentCity, null, t);

                            TaskSet newAvailableTasks = TaskSet.copyOf(avalaibleTasks);
                            TaskSet newTakenTasks = TaskSet.copyOf(takenTasks);
                            newAvailableTasks.remove(t);
                            newTakenTasks.remove(t);

                            result.add(new StateNode(currentCity, newAvailableTasks, newTakenTasks, trans, currentWeight + t.weight));
                        }
                    }
                    break;

                case DELIVER:
                    for (Task t : takenTasks) {
                        // agent can only deliver tasks in its current city
                        if (t.deliveryCity == currentCity) {
                            Transition trans = new Transition(actionType, currentCity, null, t);

                            TaskSet newTakenTasks = TaskSet.copyOf(takenTasks);
                            // task is delivered so we can get rid of it
                            newTakenTasks.remove(t);

                            result.add(new StateNode(currentCity, avalaibleTasks, newTakenTasks, trans, currentWeight - t.weight));
                        }
                    }
                    break;
            }
        }
        return result;
    }

    /**
     * Generate the set of cities where the agent can either deliver or pickup a task
     */
    private Set<City> generateCitiesOfInterest() {
        Set<City> result = new HashSet<>();
        for (Task t : avalaibleTasks) {
            result.add(t.pickupCity);
        }
        for (Task t : takenTasks) {
            result.add(t.deliveryCity);
        }

        return result;
    }
}

class Transition {
    private Type type;
    private City destination;
    private Task task;
    private double cost;

    public Transition(Type type, City current, City destination, Task task) {
        this.type = type;
        this.destination = destination;
        this.task = task;
        this.cost = destination != null ? current.distanceTo(destination) : 0;
    }

    enum Type {MOVE, PICKUP, DELIVER}
}
