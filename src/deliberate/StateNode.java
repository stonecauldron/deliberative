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

    public Solver(Vehicle vehicle, TaskSet all, TaskSet carried) {
        TaskSet remainingTasks = TaskSet.intersectComplement(all, carried);
        this.initialNode = new StateNode(vehicle.getCurrentCity(), remainingTasks, carried, carried.weightSum());
        this.vehicle = vehicle;
    }


    private List<Action> shortestPathMove(City fromCity, City toCity){

        List<Action> plan = new ArrayList<>();

        for(City c : fromCity.pathTo(toCity)){
            plan.add(new Action.Move(c));
        }

        return plan;
    }

    private Plan convertToPlan(Transition transition){


        // retrieve path of transition from the goal state
        List<Transition> transitions = new ArrayList<>();
        do {
            transitions.add(transition);
        }while((transition = transition.getPredecessor()) != null);
        // reverse to get the transition in natural order
        Collections.reverse(transitions);


        // now compile the all plan
        List<Action> plan = new ArrayList<>();
        City lastCity = initialNode.getVehicleCity();

        for(Transition t : transitions){

            switch(t.getType()){
                case MOVE :
                    plan.addAll(shortestPathMove(lastCity,t.getState().getVehicleCity()));
                    break;
                case PICKUP:
                    plan.add(new Action.Pickup(t.getTask()));
                    break;
                case DELIVER:
                    plan.add(new Action.Delivery(t.getTask()));
                    break;
            }

            lastCity = t.getState().getVehicleCity();
        }


        return new Plan(this.initialNode.getVehicleCity(),plan);
    }

    

    public synchronized Plan execute(DeliberativeTemplate.Algorithm algo){

        LinkedList<Transition> paths = new LinkedList<>();
        paths.addAll(initialNode.generateSuccessors(null,this.vehicle.capacity()));

        while(!paths.isEmpty()){

            Transition currentTransition = null;

            //_______________________________________________ NEXT TRANSITION
            if(algo == DeliberativeTemplate.Algorithm.ASTAR){

                /**
                 * instead of sorting the all list we take min heuristic
                 * and we remove the transition from the queue
                 * ==> speedup x1.4
                 */
                double minEstimatedCost = Double.MAX_VALUE;
                int i = 0, minEstimatedId = 0;

                for(Transition t : paths){

                    if(t.getTotalTravel() + t.getState().heuristic() < minEstimatedCost){
                        minEstimatedCost = t.getTotalTravel() + t.getState().heuristic();
                        minEstimatedId = i;
                        currentTransition = t;
                    }

                    i++;
                }

                paths.remove(minEstimatedId);

            }
            else {
                currentTransition = paths.poll();
            }



            if(currentTransition.getState().isGoalNode()){
                return convertToPlan(currentTransition);
            }

            paths.addAll(
                    currentTransition
                            .getState()
                            .generateSuccessors(currentTransition,this.vehicle.capacity())
            );


        }

        throw new IllegalStateException("unsolvable");
    }


}



class StateNode {


    private City currentCity;
    private TaskSet availableTasks, takenTasks;
    private double currentWeight;
    private Double heuristic = null;


    public StateNode(
            City currentCity,
            TaskSet availableTasks,
            TaskSet takenTasks,
            double currentWeight) {

        this.currentCity = currentCity;
        this.availableTasks = availableTasks;
        this.takenTasks = takenTasks;
        this.currentWeight = currentWeight;

    }


    public static StateNode initialNode(City current, TaskSet avalaible) {
        return new StateNode(current, avalaible, TaskSet.noneOf(avalaible), 0);
    }


    /**
     * @return the heuristic cost estimation to the goal state (under estimate)
     *
     */
    public double heuristic(){

        if(this.heuristic != null){
            return this.heuristic;
        }

        Set<CityEdge> edges = new HashSet<>();
        for(Task t : this.getAvailableTasks()){

            City prevCity = t.pickupCity;

            for(City c : t.pickupCity.pathTo(t.deliveryCity)){
                edges.add(new CityEdge(prevCity, c));
                prevCity = c;
            }

        }

        this.heuristic = 0.0;
        for(CityEdge c : edges){
            this.heuristic += c.distance();
        }


        return this.heuristic;
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
        for (Task t : this.getAvailableTasks()) {
            if(t.weight + currentWeight <= maximumLoad){
                result.add(t.pickupCity);
            }
        }

        for(Task t : this.getTakenTasks()){
            result.add(t.deliveryCity);
        }

        result.remove(this.getVehicleCity());

        return result;
    }


    /**
     * Generate the next possible transitions from the current state
     * or a deliver action singleton if any
     */
    public Set<Transition> generateSuccessors(Transition parent, final double maximumLoad) {

        Set<Transition> result = new HashSet<>();

        double alreadyTraveled = parent != null ? parent.getTotalTravel() : 0;

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

                double newTotal = alreadyTraveled;

                result.add(new Transition(toState, parent, Transition.Type.DELIVER, t, newTotal));

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

                double newTotal = alreadyTraveled;

                result.add(new Transition(toState, parent, Transition.Type.PICKUP, t, newTotal));
            }
        }

        // avoid dumb action : move twice => useless
        if(parent != null && parent.getType() == Transition.Type.MOVE){
            return result;
        }


        // MOVE ______________________________________________
        for (City nextVehicleCity : this.interestingMove(maximumLoad)) {

            StateNode toState = new StateNode(
                    nextVehicleCity,
                    this.getAvailableTasks(),
                    this.getTakenTasks(),
                    this.getVehicleWeight());

            double newTotal = alreadyTraveled + this.getVehicleCity().distanceTo(toState.getVehicleCity());

            result.add(new Transition(toState, parent, Transition.Type.MOVE, null, newTotal));
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

    // km already cross
    private double totalTravel;

    private Task task = null;



    public Transition(
            StateNode state,
            Transition predecessor,
            Type type,
            Task task,
            double totalTravel){

        this.state = state;
        this.predecessor = predecessor;
        this.type = type;
        this.totalTravel = totalTravel;
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
     * @return the total km traveled to arrives in this.getState in this path
     */
    public double getTotalTravel(){
        return totalTravel;
    }



    @Override
    public int compareTo(Transition that) {
        double thatCost = that.totalTravel + that.getState().heuristic();
        double thisCost = this.totalTravel + this.getState().heuristic();
        return thisCost > thatCost ? 1 :
                thisCost == thatCost ? 0 :
                        -1;
    }

    @Override
    public String toString(){
        return "(" + this.getState().getVehicleCity() + "/" +this.getType()+")";
    }

    @Override
    public boolean equals(Object that) {
        return that instanceof Transition
                && ((Transition)that).getType() == this.getType()
                && ((Transition)that).getState() == this.getState()
                && ((Transition)that).getTask() == this.getTask();

    }

    @Override
    public int hashCode() {
        return this.type.ordinal() + 31*this.state.hashCode();
    }

}


/**
 * tuple for heuristic
 */
class CityEdge {

    private City from, to;

    public CityEdge(City from, City to){
        this.from = from;
        this.to = to;
    }

    public double distance(){
        return from.distanceTo(to);
    }

    @Override
    public int hashCode(){
        return from.hashCode() + 31*to.hashCode();
    }

    @Override
    public boolean equals(Object that){
        return that instanceof CityEdge &&
                ((CityEdge)that).from == this.from &&
                ((CityEdge)that).to == this.to;
    }

}

