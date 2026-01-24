public class Edge {
    // Fields
    private Long sourceId;
    private long destinationId;
    private double distance;


    private Graph.HighwayType highwayType;
    /**
     * Constructor: takes source node ID, destination node ID, and the distance
     */
    public Edge(long sourceId, long destinationId, double distance) {
        this.sourceId = sourceId;
        this.destinationId = destinationId;
        this.distance = distance;
    }

    public Graph.HighwayType getHighwayType() {
        return highwayType;
    }

    public void setHighwayType(Graph.HighwayType highwayType) {
        this.highwayType = highwayType;
    }

    /**
     * Copy constructor
     */
    public Edge(Edge other) {
        this.sourceId = other.sourceId;
        this.destinationId = other.destinationId;
        this.distance = other.distance;
    }

    // Getters
    public Long getSourceId() {
        return sourceId;
    }

    public long getDestinationId() {
        return destinationId;
    }

    public double getDistance() {
        return distance;
    }

    // Setters
    public void setSourceId(Long sourceId) {
        this.sourceId = sourceId;
    }

    public void setDestinationId(long destinationId) {
        this.destinationId = destinationId;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    @Override
    public String toString() {

        return ("[" + getSourceId() + " -> " + getDestinationId() + "] " + getDistance());
    }
}
