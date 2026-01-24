import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.util.*;

public class Graph {
    private static final double R = 6378137;

    /** Enum of the highway type.
     *  Each type has a factor that is used to calculate the weight of the edges
     *  W(edge) = HighwayType(factor) * Haversine
     */
    public enum HighwayType {
        MOTORWAY(0.5),
        TRUNK(0.5),
        PRIMARY(0.75),
        SECONDARY(0.75),
        TERTIARY(1.0),
        RESIDENTIAL(1.0),
        LIVING_STREET(1.25),
        UNCLASSIFIED(1.25),
        SERVICE(1.5),
        TRACK(1.5);

        private final double factor;

        HighwayType(double factor) {
            this.factor = factor;
        }

        public double getFactor() {
            return factor;
        }
    }


    //List of vertices in the graph
    private List<Vertex> vertices;
    //A map that stores the ID of a vertex(Node) and it's index in the list of vertices
    private Map<Long, Integer> map;


    public Graph() {
        this.vertices = new ArrayList<>();
        this.map = new HashMap<>();
    }

    /**
     * Convenience constructor: builds a graph directly from an OSM XML file.
     * It will:
     *  1) Parse all <node> elements and create Vertex objects.
     *  2) Parse all <way> elements, filter by highway tag, and create Edge objects.
     *  3) Remove all isolated vertices (with no incoming and no outgoing edges).
     *
     * @param osmFilePath path to the OSM XML file on disk
     */
    public Graph(String osmFilePath) {
        this(); // ensure vertices/map are initialized

        try {
            loadFromOSM(osmFilePath);
        } catch (Exception e) {
            // You can handle this differently if you want (log, rethrow checked, etc.)
            throw new RuntimeException("Failed to build graph from OSM file: " + osmFilePath, e);
        }
    }

    private void loadFromOSM(String osmFilePath) throws Exception {
        File file = new File(osmFilePath);
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        DocumentBuilder builder = factory.newDocumentBuilder();

        Document doc = builder.parse(file);
        doc.getDocumentElement().normalize();

        NodeList nodeList = doc.getElementsByTagName("node");
        int nodeCount = nodeList.getLength();

        this.vertices = new ArrayList<>(nodeCount);
        this.map = new HashMap<>((int)(nodeCount / 0.75f) + 1);

        parseNodes(doc);

        parseWays(doc);

        removeIsolatedVertices();

        System.out.println("Vertices after load: " + vertices.size());
    }


    /**
     * Parses all node elements in the OSM document and creates Vertex objects.
     * Only the attributes id, lat and lon are used; any other node information is ignored.
     */
    private void parseNodes(Document doc) {

        Element root = doc.getDocumentElement();

        for (Node child = root.getFirstChild(); child != null; child = child.getNextSibling()) {
            if (child.getNodeType() != Node.ELEMENT_NODE) {
                continue;
            }

            Element elem = (Element) child;
            if (!"node".equals(elem.getTagName())) {
                continue; // skip <way>, <relation>, etc.
            }

            long id = Long.parseLong(elem.getAttribute("id"));
            double lat = Double.parseDouble(elem.getAttribute("lat"));
            double lon = Double.parseDouble(elem.getAttribute("lon"));

            Vertex v = new Vertex(id, lat, lon);
            addVertex(v);
        }
    }



    /**
     * Parses all <way> elements in the OSM document and creates Edge objects
     * only for those that are valid highways (based on the "highway" tag).
     * Direction is determined by the "oneway" tag:
     *   - "yes" or "1" => one-way (forward only)
     *   - "no" or missing => two-way (edges in both directions)
     */
    private void parseWays(Document doc) {
        NodeList wayList = doc.getElementsByTagName("way");
        System.out.println("Number of <way> elements: " + wayList.getLength());

        for (int i = 0; i < wayList.getLength(); i++) {
            Node wayNode = wayList.item(i);
            if (wayNode.getNodeType() != Node.ELEMENT_NODE) {
                continue;
            }

            Element wayElem = (Element) wayNode;

            // Local storage for this way
            List<Long> nodeRefs = new ArrayList<>();
            String highwayValue = null;
            String onewayValue = null;

            // Iterate over the children of <way>: <nd> and <tag>
            NodeList children = wayElem.getChildNodes();
            for (int j = 0; j < children.getLength(); j++) {
                Node child = children.item(j);
                if (child.getNodeType() != Node.ELEMENT_NODE) {
                    continue;
                }

                Element childElem = (Element) child;
                String tagName = childElem.getTagName();

                if (tagName.equals("nd")) {
                    // <nd ref="..."> gives us node references in sequence
                    String refStr = childElem.getAttribute("ref");
                    long refId = Long.parseLong(refStr);
                    nodeRefs.add(refId);
                } else if (tagName.equals("tag")) {
                    // <tag k="..." v="...">
                    String k = childElem.getAttribute("k");
                    String v = childElem.getAttribute("v");

                    if ("highway".equals(k)) {
                        highwayValue = v;
                    } else if ("oneway".equals(k)) {
                        onewayValue = v;
                    }
                }
            }

            // If there's no highway tag, this way is not a road so ignore it
            if (highwayValue == null) {
                continue;
            }

            // Convert the highway string to our enum, if it fails ignore this way
            HighwayType type = highwayTypeFromString(highwayValue);
            if (type == null) {
                continue;
            }

            // Determine whether this way is one-way or two-way
            boolean isOneWay = isOneWay(onewayValue);

            // Now create edges between consecutive node references
            for (int k = 0; k < nodeRefs.size() - 1; k++) {
                long fromId = nodeRefs.get(k);
                long toId   = nodeRefs.get(k + 1);

                // Both endpoints must exist in the graph
                Integer fromIndexObj = map.get(fromId);
                Integer toIndexObj   = map.get(toId);
                if (fromIndexObj == null || toIndexObj == null) {
                    // Some OSM files might reference nodes that are not in the file, just skip them
                    continue;
                }

                Vertex fromVertex = vertices.get(fromIndexObj);
                Vertex toVertex   = vertices.get(toIndexObj);

                // Forward edge fromId -> toId
                Edge forward = new Edge(fromId, toId, 0.0);
                forward.setHighwayType(type);
                double forwardWeight = computeFinalWeight(forward);
                forward.setDistance(forwardWeight);
                fromVertex.getEdges().add(forward);

                // If the way is not strictly one-way, add the reverse edge to model a two-way road
                if (!isOneWay) {
                    Edge backward = new Edge(toId, fromId, 0.0);
                    backward.setHighwayType(type);
                    double backwardWeight = computeFinalWeight(backward);
                    backward.setDistance(backwardWeight);
                    toVertex.getEdges().add(backward);
                }
            }
        }
    }

    /**
     * Maps the "highway" string from OSM (e.g. "residential", "primary", "living_street")
     * to the corresponding HighwayType enum value. If the value is not supported,
     * this method returns null.
     */
    private HighwayType highwayTypeFromString(String highwayValue) {
        if (highwayValue == null) {
            return null;
        }

        // eg:
        //  "residential"   -> "RESIDENTIAL"
        //  "living_street" -> "LIVING_STREET"
        String normalized = highwayValue.toUpperCase(Locale.ROOT);


        try {
            return HighwayType.valueOf(normalized);
        } catch (IllegalArgumentException ex) {
            // Not one of our supported types
            return null;
        }
    }

    /**
     * Determines whether the given oneway string represents a one-way street.
     * According to the assignment:
     *  "yes" or "1" => one-way (true)
     *  "no" or missing => two-way (false)
     */
    private boolean isOneWay(String onewayValue) {
        if (onewayValue == null) {
            // Default when oneway is missing is two-way
            return false;
        }

        String v = onewayValue.trim().toLowerCase(Locale.ROOT);
        return v.equals("yes") || v.equals("1");

        // "no", "0", or any other value → treat as two-way
    }

    /**
     * Removes all vertices that are completely isolated
     * This should be called once after the entire graph has been built from the OSM file
     */
    private void removeIsolatedVertices() {
        int n = vertices.size();
        int[] inDegree = new int[n];

        // Build in-degree counts in one pass
        for (Vertex v : vertices) {
            for (Edge e : v.getEdges()) {
                Integer destIndex = map.get(e.getDestinationId());
                if (destIndex != null) {
                    inDegree[destIndex]++;
                }
            }
        }

        // Collect isolated vertices
        List<Long> toDelete = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            Vertex v = vertices.get(i);
            if (v.getEdges().isEmpty() && inDegree[i] == 0) {
                toDelete.add(v.getId());
            }
        }

        // Now delete
        for (Long id : toDelete) {
            deleteIsolatedVertex(id);
        }
    }
    //took so long to delete all the isolated vertices with my delete
    private void deleteIsolatedVertex(long vertexID) {
        Integer indexObj = map.get(vertexID);
        if (indexObj == null) return;
        int indexToRemove = indexObj;
        int lastIndex = vertices.size() - 1;

        if (indexToRemove != lastIndex) {
            Vertex lastVertex = vertices.get(lastIndex);
            vertices.set(indexToRemove, lastVertex);
            map.put(lastVertex.getId(), indexToRemove);
        }

        vertices.remove(lastIndex);
        map.remove(vertexID);
    }


    public void addVertex(Vertex vertex) {
        if (vertex == null) {
            throw new IllegalArgumentException("Vertex cant be null.");
        }

        long id = vertex.getId();

        if (map.containsKey(id)) {
            return;
        }

        int index = vertices.size();
        vertices.add(vertex);
        map.put(id, index);
    }



    public void deleteVertex(long vertexID) {
        if (!map.containsKey(vertexID)) {
            System.out.println("Vertex doesn't exist!");
            return;
        }

        // remove all edges that have this vertex as DESTINATION
        for (Vertex v : vertices) {
            v.getEdges().removeIf(e -> e.getDestinationId() == vertexID);

        }

        int indexToRemove = map.get(vertexID);
        int lastIndex = vertices.size() - 1;

        if (indexToRemove != lastIndex) {
            // vertex to delete is not the last one → swap
            Vertex lastVertex = vertices.get(lastIndex);

            // move last vertex into the "hole"
            vertices.set(indexToRemove, lastVertex);

            // update its index in the map
            map.put(lastVertex.getId(), indexToRemove);
        }

        // remove last element (now either the deleted vertex or the moved one)
        vertices.remove(lastIndex);

        // remove the deleted vertex from the map
        map.remove(vertexID);

    }


    public List<Vertex> Dijkstra(long start, long end) {
        int n = vertices.size();
        List<Vertex> path = new ArrayList<>();

        Integer startIndexObj = map.get(start);
        Integer endIndexObj   = map.get(end);

        if (startIndexObj == null || endIndexObj == null) {
            return path; // empty path
        }

        int startIndex = startIndexObj;
        int endIndex   = endIndexObj;

        boolean[] isVisited = new boolean[n];
        double[] distance = new double[n];
        int[] parents = new int[n];

        for(int i = 0; i < n; i++) {
            distance[i] = Double.POSITIVE_INFINITY;
            parents[i] = -1;
            isVisited[i] = false;
        }

        distance[map.get(start)] = 0;

        PriorityQueue<SearchNode> queue = new PriorityQueue<>();
        queue.offer(new SearchNode(startIndex, 0, 0));

        while(!queue.isEmpty()) {
            SearchNode current = queue.poll();
            int u = current.index;

            // If this entry is stale (older distance), skip it
            if (current.gScore > distance[u]) {
                continue;
            }

            if (isVisited[u]) {
                continue;
            }

            isVisited[u] = true;

            // If we reach the destination we are interested in, we don't care about the other paths to other nodes:)
            if (u == endIndex) {
                break;
            }
            //Relaxation
            for(Edge e : vertices.get(u).getEdges()) {
                int v = map.get(e.getDestinationId());
                if(isVisited[v]) {
                    continue;
                }
                double newDistance = distance[u] + e.getDistance();
                if(distance[v] > newDistance) {
                    distance[v] = newDistance;
                    parents[v] = u;
                    queue.offer(new SearchNode(v, newDistance, newDistance));
                }
                if(distance[v] == newDistance) {
                    parents[v] = vertices.get(parents[v]).getId() > vertices.get(u).getId() ? u : parents[v];
                }
            }
        }

        if(distance[endIndex] == Double.POSITIVE_INFINITY) {
            return path;
        }

        int currentIndex = endIndex;
        while(currentIndex != -1) {
            path.add(vertices.get(currentIndex));
            currentIndex = parents[currentIndex];
        }

        Collections.reverse(path);
        return path;
    }



    public List<Long> BFS(long start) {
        int n = vertices.size();
        List<Long> bfsPath = new ArrayList<>();
        Queue<Integer> queue = new ArrayDeque<>();

        Integer startIndexObj = map.get(start);
        if (startIndexObj == null) {
            return Collections.emptyList();
        }
        int startIndex = startIndexObj;
        boolean[] isVisited = new boolean[n];
        queue.offer(startIndex);
        isVisited[startIndex] = true;

        while(!queue.isEmpty()) {
            int currentIndex = queue.poll();
            bfsPath.add(vertices.get(currentIndex).getId());
            List<Edge> vertexEdges = new ArrayList<>(vertices.get(currentIndex).getEdges());
            vertexEdges.sort(Comparator.comparingLong(Edge::getDestinationId));

            for(Edge e : vertexEdges) {
                int v = map.get(e.getDestinationId());
                if(isVisited[v]) {
                    continue;
                }
                isVisited[v] = true;
                queue.offer(v);
            }
        }
        return bfsPath;
    }

    public List<Long> DFS(long start) {
        int n = vertices.size();
        List<Long> dfsPath = new ArrayList<>();
        Stack<Integer> stack = new Stack<>();

        Integer startIndexObj = map.get(start);
        if (startIndexObj == null) {
            return Collections.emptyList();
        }
        int startIndex = startIndexObj;
        boolean[] isVisited = new boolean[n];
        stack.push(startIndex);
        isVisited[startIndex] = true;

        while(!stack.isEmpty()) {
            int currentIndex = stack.pop();
            dfsPath.add(vertices.get(currentIndex).getId());

            List<Edge> vertexEdges = new ArrayList<>(vertices.get(currentIndex).getEdges());
            vertexEdges.sort(Comparator.comparingLong(Edge::getDestinationId).reversed());

            for(Edge e : vertexEdges) {
                int v = map.get(e.getDestinationId());
                if(isVisited[v]) {
                    continue;
                }
                isVisited[v] = true;
                stack.push(v);
            }
        }
        return dfsPath;
    }

    public void compress() {
        boolean changed;

        // Build the "Incoming Edge Map" LOCALLY.
        // This takes O(V+E) time once.
        // Key = Vertex ID, Value = List of IDs pointing to it.
        Map<Long, List<Long>> incomingMap = new HashMap<>();

        for (Vertex v : vertices) {
            for (Edge e : v.getEdges()) {
                incomingMap.computeIfAbsent(e.getDestinationId(), _ -> new ArrayList<>())
                        .add(v.getId());
            }
        }

        do {
            changed = false;
            List<Vertex> snapshot = new ArrayList<>(vertices);

            for (Vertex v : snapshot) {
                long id = v.getId();
                if (!map.containsKey(id)) continue; // Already deleted

                // Lookup is now O(1) using the local map
                int outDeg = v.getEdges().size();
                List<Long> incomingList = incomingMap.get(id);
                int inDeg = (incomingList == null) ? 0 : incomingList.size();

                if (outDeg == 1 && inDeg == 1) {
                    // Pass the map to the helper so it can stay updated
                    if (oneWayStreetReduction(v, incomingMap)) {
                        changed = true;
                    }
                }
                else if (outDeg == 2 && inDeg == 2) {
                    if (twoWayStreetReduction(v, incomingMap)) {
                        changed = true;
                    }
                }
            }
        } while (changed);
    }

    /**
     * Collapses a one-way intermediate vertex V between two distinct vertices U and W:
     * <p>
     *    U -> V -> W   becomes   U -> W
     * <p>
     * V must have exactly 1 incoming and 1 outgoing edge
     * <p>
     * This method:
     *   1. Finds the unique incoming edge U -> V.
     *   2. Uses the existing outgoing edge V -> W.
     *   3. Replaces U -> V with U -> W, updating the distance.
     *   4. Deletes the intermediate vertex V from the graph.
     */
    // Add the Map parameter
    private boolean oneWayStreetReduction(Vertex middle, Map<Long, List<Long>> incomingMap) {
        long middleId = middle.getId();

        // ... existing checks (middle.getEdges().size() != 1, etc) ...

        Edge outgoing = middle.getEdges().getFirst();
        long destId = outgoing.getDestinationId(); // W

        // Find U (the node pointing to middle) using the map
        List<Long> sources = incomingMap.get(middleId);
        if (sources == null || sources.size() != 1) return false;

        long sourceId = sources.getFirst(); // U
        Vertex sourceVertex = vertices.get(map.get(sourceId));

        // Find the actual edge object U -> V
        Edge incoming = null;
        for(Edge e : sourceVertex.getEdges()) {
            if(e.getDestinationId() == middleId) {
                incoming = e;
                break;
            }
        }

        if (incoming == null || sourceId == destId) return false;

        // ... Calculate new weight ...
        double newDistance = incoming.getDistance() + outgoing.getDistance();

        // update the map, IMPORTANT

        // U no longer points to middle
        // We don't need to update incomingMap.get(middleId) because middle is being deleted anyway

        // U now points to W (destId)
        // Update the edge object
        incoming.setDestinationId(destId);
        incoming.setDistance(newDistance);

        // Update the map for W, add U to its incoming list
        incomingMap.computeIfAbsent(destId, _ -> new ArrayList<>()).add(sourceId);

        // Middle no longer points to W
        // Remove middle from W's incoming list
        List<Long> destIncoming = incomingMap.get(destId);
        if(destIncoming != null) {
            destIncoming.remove(middleId);
        }
        deleteVertex(middleId);
        return true;
    }


    /**
     * Collapses a two-way intermediate vertex V between two distinct vertices A and B:
     * A <-> V <-> B   becomes   A <-> B
     * <p>
     * More precisely:
     *   - V has exactly two outgoing edges: V -> A and V -> B
     *   - V has exactly two incoming edges: A -> V and B -> V
     *   - A and B are distinct vertices.
     * <p>
     * This method:
     *   1. Identifies the two neighbors A and B of V.
     *   2. Finds the incoming edges A -> V and B -> V.
     *   3. Uses the outgoing edges V -> A and V -> B.
     *   4. Replaces:
     *        A -> V -> B with A -> B
     *        B -> V -> A with B -> A
     *      updating the distances accordingly.
     *   5. Deletes V from the graph.
     */
    private boolean twoWayStreetReduction(Vertex middle, Map<Long, List<Long>> incomingMap) {
        long middleId = middle.getId();

        // Verify Outgoing Edges from Middle (V -> A, V -> B)
        if (middle.getEdges().size() != 2) {
            return false;
        }

        Edge out1 = middle.getEdges().get(0);
        Edge out2 = middle.getEdges().get(1);

        long neighborAId = out1.getDestinationId();
        long neighborBId = out2.getDestinationId();

        if (neighborAId == neighborBId) {
            return false; // Not a line but a multi edge loop
        }

        // Verify Incoming Edges to Middle (A -> V, B -> V) using the MAP
        List<Long> sources = incomingMap.get(middleId);
        if (sources == null || sources.size() != 2) {
            return false;
        }

        // Check if the sources are actually A and B
        boolean aPointsToMiddle = sources.contains(neighborAId);
        boolean bPointsToMiddle = sources.contains(neighborBId);

        if (!aPointsToMiddle || !bPointsToMiddle) {
            // This means V is connected to X and Y, but pointed to by Z and W.
            // Not a standard two-way street.
            return false;
        }

        // Get the actual Vertex objects for A and B
        Vertex neighborA = vertices.get(map.get(neighborAId));
        Vertex neighborB = vertices.get(map.get(neighborBId));

        // Find the specific Edge objects: A -> V and B -> V
        Edge inFromA = null;
        for (Edge e : neighborA.getEdges()) {
            if (e.getDestinationId() == middleId) {
                inFromA = e;
                break;
            }
        }

        Edge inFromB = null;
        for (Edge e : neighborB.getEdges()) {
            if (e.getDestinationId() == middleId) {
                inFromB = e;
                break;
            }
        }

        if (inFromA == null || inFromB == null) {
            return false; // Should satisfy by map check, but safety first :)))
        }

        // Determine which outgoing edge goes where
        // outToA is V -> A, outToB is V -> B
        Edge outToA = (out1.getDestinationId() == neighborAId) ? out1 : out2;
        Edge outToB = (out1.getDestinationId() == neighborBId) ? out1 : out2;

        // Calculate new distances
        // New A -> B = (A -> V) + (V -> B)
        double distAtoB = inFromA.getDistance() + outToB.getDistance();

        // New B -> A = (B -> V) + (V -> A)
        double distBtoA = inFromB.getDistance() + outToA.getDistance();

        // CRITICAL: Update the Structure and the Map

        // --- UPDATE A -> B ---
        inFromA.setDestinationId(neighborBId);
        inFromA.setDistance(distAtoB);

        // Update Map for B:
        // Remove Middle (V) from B's incoming list
        List<Long> incomingToB = incomingMap.get(neighborBId);
        if(incomingToB != null) incomingToB.remove(middleId);
        // Add A to B's incoming list
        incomingMap.computeIfAbsent(neighborBId, _ -> new ArrayList<>()).add(neighborAId);


        // --- UPDATE B -> A ---
        inFromB.setDestinationId(neighborAId);
        inFromB.setDistance(distBtoA);

        // Update Map for A:
        // Remove Middle (V) from A's incoming list
        List<Long> incomingToA = incomingMap.get(neighborAId);
        if(incomingToA != null) incomingToA.remove(middleId);
        // Add B to A's incoming list
        incomingMap.computeIfAbsent(neighborAId, _ -> new ArrayList<>()).add(neighborBId);


        // finally delete the middle vertex
        deleteVertex(middleId);

        return true;
    }

    /**
     * A* Star Search Algorithm
     * <p>
     *   It works just like Dijkstra's but instead of just taking in account the
     *      distance of the current node to the starting one it adds
     *      another score which is the distance (haversine) of the current node
     *      to the target node f(n) = h(n) + g(n)
     *      h(n) -> distance from the starting node
     *      g(n) -> distance to the finish node.
     * </p>
     */
    public List<Vertex> AStar(long startId, long endId) {
        Integer startIdx = map.get(startId);
        Integer endIdx = map.get(endId);

        if (startIdx == null || endIdx == null) return Collections.emptyList();

        int n = vertices.size();

        Vertex goalVertex = vertices.get(map.get(endId));

        // distance[i] stores the shortest actual distance found from 'start' to 'i'
        double[] distance = new double[n];
        int[] parents = new int[n];
        boolean[] visited = new boolean[n];

        Arrays.fill(distance, Double.POSITIVE_INFINITY);
        Arrays.fill(parents, -1);

        distance[startIdx] = 0.0;

        // Initial score = 0 distance + estimated distance to goal
        double initialScore = calculateHeuristic(vertices.get(startIdx), goalVertex);

        PriorityQueue<SearchNode> pq = new PriorityQueue<>();
        pq.offer(new SearchNode(startIdx, 0.0, initialScore));

        while (!pq.isEmpty()) {
            SearchNode current = pq.poll();
            int u = current.index;

            // If we reached the goal index, we are done
            if (u == endIdx) {
                break;
            }

            // Skip if we've already found a better path for this node
            if (visited[u]) {
                continue;
            }
            visited[u] = true;

            for (Edge edge : vertices.get(u).getEdges()) {
                Integer v = map.get(edge.getDestinationId());
                if (v == null || visited[v]) {
                    continue;
                }

                // Calculate the new distance from start to neighbor 'v'
                double newDistance = distance[u] + edge.getDistance();

                if (newDistance < distance[v]) {
                    distance[v] = newDistance;
                    parents[v] = u;

                    // The A* Score: Actual distance + Heuristic
                    double h = calculateHeuristic(vertices.get(v), goalVertex);
                    double score = newDistance + h;

                    pq.offer(new SearchNode(v, newDistance, score));
                }
            }
        }

        return reconstructPath(parents, endIdx);
    }

    /**
     * Heuristic function is the straight-line distance between two points
     * This is acceptable because the shortest path between two points is a line
     */
    private double calculateHeuristic(Vertex v1, Vertex v2) {
        return calculateRawHaversine(v1.getLatitude(), v1.getLongitude(),
                v2.getLatitude(), v2.getLongitude());
    }

    private List<Vertex> reconstructPath(int[] parents, int endIdx) {
        List<Vertex> path = new ArrayList<>();
        int curr = endIdx;
        while (curr != -1) {
            path.add(vertices.get(curr));
            curr = parents[curr];
        }
        Collections.reverse(path);
        return path.isEmpty() || parents[endIdx] == -1 && vertices.get(endIdx).getId() != vertices.getFirst().getId() ? new ArrayList<>() : path;
    }

    private double calculateRawHaversine(double lat1, double lon1, double lat2, double lon2) {
        double latitude1 = lat1 * (Math.PI / 180);
        double latitude2 = lat2 * (Math.PI / 180);
        double longitude1 = lon1 * (Math.PI / 180);
        double longitude2 = lon2 * (Math.PI / 180);

        double latitudeDiff = latitude1 - latitude2;
        double longitudeDiff = longitude1 - longitude2;

        double a = Math.pow(Math.sin(latitudeDiff / 2), 2)
                + Math.cos(latitude1) * Math.cos(latitude2)
                * Math.pow(Math.sin(longitudeDiff / 2), 2);

        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        return R * c;
    }

    private double computeHaversineDistance(Edge edge) {
        Vertex src = vertices.get(map.get(edge.getSourceId()));
        Vertex dst = vertices.get(map.get(edge.getDestinationId()));

        // Use the new helper!
        return calculateRawHaversine(src.getLatitude(), src.getLongitude(),
                dst.getLatitude(), dst.getLongitude());
    }


    private double computeFinalWeight(Edge edge) {
        return computeHaversineDistance(edge) *  edge.getHighwayType().getFactor();
    }

    @Override
    public String toString() {
        return vertices.toString();
    }

    public int getVertexCount() {
        return vertices.size();
    }

    public Vertex getVertex(int index) {
        return vertices.get(index);
    }

    private record SearchNode(int index, double gScore, double fScore) implements Comparable<SearchNode> {
        @Override
        public int compareTo(SearchNode other) {
            return Double.compare(this.fScore, other.fScore);
        }
    }
}
