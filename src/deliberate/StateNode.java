package deliberate;

import logist.plan.Action;
import logist.plan.Plan;
import logist.simulation.Vehicle;
import logist.task.Task;
import logist.task.TaskSet;
import logist.topology.Topology.City;

import java.util.*;





class Solver {


    private StateNode initialNode;

    private Vehicle vehicle;



    public Solver(
            Vehicle vehicle,
            TaskSet tasks){

        this.initialNode = StateNode.initialNode(vehicle.getCurrentCity(),tasks);
        this.vehicle = vehicle;
    }


    private Plan convertToPlan(Transition transition){

        List<Action> actions = new ArrayList<>();

        do {
            actions.add(transition.toAction());
        }while( (transition = transition.getPredecessor()) != null );

        Collections.reverse(actions);

        return new Plan(this.initialNode.getVehicleCity(),actions);
    }



    public Plan execute(DeliberativeTemplate.Algorithm algo){

        LinkedList<Transition> paths = new LinkedList<>();
        paths.addAll(initialNode.generateSuccessors(null,this.vehicle.capacity(),this.vehicle.costPerKm()));

        while(!paths.isEmpty()){

            Transition currentTransition = paths.poll();

            if(currentTransition.getState().isGoalNode()){
                return convertToPlan(currentTransition);
            }

            paths.addAll(
                    currentTransition
                            .getState()
                            .generateSuccessors(currentTransition,this.vehicle.capacity(),this.vehicle.costPerKm())
            );


            if(algo == DeliberativeTemplate.Algorithm.ASTAR){
                Collections.sort(paths);
            }

        }

        return null;
    }


}




class StateNode {


    private City currentCity;
    private TaskSet availableTasks, takenTasks;
    private double currentWeight;


    public StateNode(
            City currentCity,
            TaskSet avalaibleTasks,
            TaskSet takenTasks,
            double currentWeight) {

        this.currentCity = currentCity;
        this.availableTasks = avalaibleTasks;
        this.takenTasks = takenTasks;
        this.currentWeight = currentWeight;

        System.out.println(this.takenTasks.size());
        System.out.println(this.availableTasks.size());

    }


    public static StateNode initialNode(City current, TaskSet avalaible) {
        return new StateNode(current, avalaible, TaskSet.noneOf(avalaible), 0);
    }


    /**
     * @return the vehicle's city in the current state
     */
    public City getVehicleCity(){
        return currentCity;
    }


    /**
     * @return all tasks not already deliver/loaded_in_the_vehicle
     */
    public TaskSet getAvailableTasks(){
        return availableTasks;
    }

    /**
     * @return all tasks loaded in the vehicle
     */
    public TaskSet getTakenTasks(){
        return takenTasks;
    }

    /**
     * @return the total weight in the vehicle
     */
    public double getVehicleWeight(){
        return currentWeight;
    }

    /**
     * @return true if the state is a goal node
     */
    public boolean isGoalNode(){
        return this.availableTasks.isEmpty() && this.takenTasks.isEmpty();
    }

    /**
     * Generate the set of cities where the agent can pickup objects
     */
    public Set<City> interestingMove(double maximumLoad) {

        Set<City> result = new HashSet<>();
        for (Task t : availableTasks) {
            if(t.weight + currentWeight <= maximumLoad){
                result.add(t.pickupCity);
            }
        }

        return result;
    }


    /**
     * Generate the next possible transitions from the current state
     * or a deliver action singleton if any
     */
    public Set<Transition> generateSuccessors(Transition parent, final double maximumLoad, double costPerKm) {

        Set<Transition> result = new HashSet<>();

        double currentCost = parent != null ? parent.getCost() : 0;

        // DELIVER ____________________________________________
        for (Task t : this.getTakenTasks()) {

            // agent can only deliver tasks in its current city
            if (t.deliveryCity == this.getVehicleCity()) {

                TaskSet newTakenTasks = TaskSet.copyOf(this.getTakenTasks());
                newTakenTasks.remove(t);

                StateNode toState = new StateNode(
                        this.getVehicleCity(),
                        this.getAvailableTasks(),
                        newTakenTasks,
                        this.getVehicleWeight()-t.weight);

                double cost = currentCost;

                result.add(new Transition(toState, parent, Transition.Type.DELIVER, t, cost));

                // in case we are able to deliver something
                // this action will always be taken (0 cost + one passed subgoal)
                return result;
            }
        }


        // PICKUP ____________________________________________
        for (Task t : this.getAvailableTasks()) {
            if(t.pickupCity == this.getVehicleCity() && t.weight + this.getVehicleWeight() <= maximumLoad){

                TaskSet newAvailableTasks = TaskSet.copyOf(this.getAvailableTasks());
                TaskSet newTakenTasks = TaskSet.copyOf(this.getTakenTasks());
                newAvailableTasks.remove(t);
                newTakenTasks.add(t);

                StateNode toState = new StateNode(
                        this.getVehicleCity(),
                        newAvailableTasks,
                        newTakenTasks,
                        this.getVehicleWeight()+t.weight);

                double cost = currentCost;

                result.add(new Transition(toState, parent, Transition.Type.PICKUP, t, cost));
            }
        }


        // MOVE ______________________________________________
        for (City nextVehicleCity : this.interestingMove(maximumLoad)) {

            StateNode toState = new StateNode(
                    nextVehicleCity,
                    this.getAvailableTasks(),
                    this.getTakenTasks(),
                    this.getVehicleWeight());

            double cost = currentCost + this.getVehicleCity().distanceTo(toState.getVehicleCity())*costPerKm;

            result.add(new Transition(toState, parent, Transition.Type.MOVE, null, cost));
        }



        return result;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        StateNode stateNode = (StateNode) o;

        if (!currentCity.equals(stateNode.currentCity)) return false;
        if (!availableTasks.equals(stateNode.availableTasks)) return false;
        if (!takenTasks.equals(stateNode.takenTasks)) return false;
        return true;

    }

    @Override
    public int hashCode() {
        int result = currentCity.hashCode();
        result = 31 * result + availableTasks.hashCode();
        result = 107 * result + takenTasks.hashCode();
        return result;
    }

}






class Transition implements Comparable<Transition> {

    // three possible action
    enum Type {MOVE, PICKUP, DELIVER}

    // linkedList/tree fashion (one known predecessor / many unknown sucessors)
    // i.e : we can rewind from a state to initialState
    private Transition predecessor;

    //track the current state
    private StateNode state;

    // track the last move
    private Type type;

    // current cost
    private double cost;

    private Task task = null;



    public Transition(StateNode state, Transition predecessor, Type type, Task task, double cost){
        this.state = state;
        this.predecessor = predecessor;
        this.type = type;
        this.cost = cost;
        this.task = task;
    }



    /**
     * @return the transition taken just before this one
     */
    public Transition getPredecessor(){
        return predecessor;
    }

    /**
     * @return the snapshot of the world
     */
    public StateNode getState(){
        return state;
    }

    /**
     * @return the type of action executed during this transition
     */
    public Type getType(){
        return type;
    }

    /**
     * @return the task of the transition or null
     */
    public Task getTask(){
        return task;
    }

    /**
     * @return the cost of all the transitions taken to arrives in this.getState
     */
    public double getCost(){
        return cost;
    }


    /**
     * @return the coresponding action of the transition
     */
    public Action toAction(){

        switch(this.getType()){
            case PICKUP: return new Action.Pickup(this.getTask());
            case MOVE : return new Action.Move(this.getState().getVehicleCity());
            case DELIVER: return new Action.Delivery(this.getTask());
            default : return null;
        }

    }

    /**
     * @return the heuristic cost estimation to the goal state (under estimate)
     */
    public double heuristic(){
        return -1;
    }


    @Override
    public int compareTo(Transition that) {
        double thatCost = that.cost + that.heuristic();
        double thisCost = this.cost + this.heuristic();
        return thisCost > thatCost ? 1 :
                thisCost == thatCost ? 0 :
                -1;
    }

    @Override
    public String toString(){
        return "(" + this.getState().getVehicleCity() + "/" +this.getType()+")";
    }

}

