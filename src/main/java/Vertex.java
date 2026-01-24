import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Vertex {

    private final long id;
    private final double latitude;
    private final double longitude;

    private final List<Edge> edges;  // outgoing edges from this vertex

    /**
     * Constructor with id, latitude, and longitude
     */
    public Vertex(long id, double latitude, double longitude) {
        this.id = id;
        this.latitude = latitude;
        this.longitude = longitude;
        this.edges = new ArrayList<>();
    }

    /**
     * Copy constructor
     * Performs a shallow copy of the edges list
     */
    public Vertex(Vertex other) {
        this.id = other.id;
        this.latitude = other.latitude;
        this.longitude = other.longitude;
        this.edges = new ArrayList<>(other.edges); // shallow copy
    }

    /**
     * Adds an outgoing edge
     */
    public void addEdge(Edge edge) {
        if (edge == null) {
            throw new IllegalArgumentException("Edge cannot be null");
        }
        edges.add(edge);
    }

    /**
     * Removes a specific edge. Returns true if it was found and removed
     */
    public boolean removeEdge(Edge edge) {
        return edges.remove(edge);
    }

    /**
     * Returns an unmodifiable list of edges to prevent external modification
     */
    public List<Edge> getEdges() {
        return edges;
    }

    // Getters
    public long getId() {
        return id;
    }

    public double getLatitude() {
        return latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    @Override
    public String toString() {
        return String.format("[%d] ", id);
    }
}
