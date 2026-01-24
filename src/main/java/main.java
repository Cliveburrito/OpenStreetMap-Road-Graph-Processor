import java.io.File;
import java.util.List;
import java.util.Scanner;
import java.awt.Desktop;
import java.net.URI;

public class main {
    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);
        Graph graph = new Graph();

        while(true) {
            String menu = """
                    \n-i <filepath> : Import Graph from <filepath>
                    -c : Compact Graph
                    -p <sid> <eid> : Estimate the shortest path between start
                                     node with <sid> and end node with <eid>
                    -a <sid> <eid> : Estimate the shortest path between start
                                     node with <sid> and end node with <eid> via A* heuristic
                    -b <sid> : Print bfs starting from node with <sid>
                    -d <sid> : Print dfs starting from node with <sid>
                    -r : Generate two random node IDs
                    -q : Exit without memory leaks
                    Enter your choice:
                    """;
            System.out.println(menu);

            String input = scanner.nextLine().trim();
            String[] tokens = input.split("\\s+");

            switch(tokens[0]) {
                case "-i" -> {
                    if (tokens.length != 2) {
                        System.out.println("Invalid input. -i <filename>.");
                        continue;
                    }

                    String path = tokens[1];

                    File f = new File(path);
                    if (!f.exists() || !f.isFile() || !f.canRead()) {
                        System.out.println();
                        System.out.println("Unable to open file: " + path);
                        System.out.println();
                        continue;
                    }

                    try {
                        graph = new Graph(path);  // ή Graph.fromOSM(path)
                        System.out.println("Graph OK");
                    } catch (Exception e) {
                        System.out.println();
                        System.out.println("Invalid format for file: " + path);
                        System.out.println();
                    }
                }
                case "-c" -> {
                    if(tokens.length != 1) {
                        System.out.println("Invalid input");
                        continue;
                    }
                    graph.compress();
                    System.out.println("Compact OK");
                }
                case "-p" -> {
                    if(tokens.length != 3) {
                        System.out.println("Invalid input");
                        continue;
                    }
                    long sourceID = Long.parseLong(tokens[1]);
                    long destinationID = Long.parseLong(tokens[2]);
                    long startTime = System.nanoTime();
                    List<Vertex> shortestPath = graph.Dijkstra(sourceID, destinationID);
                    long endTime = System.nanoTime();

                    if (shortestPath.isEmpty()) {
                        System.out.println("No path found.");
                        continue;
                    }

                    System.out.printf("Dijkstra found path in %.2f ms\n", (endTime - startTime) / 1_000_000.0);

                    openMapInBrowser(shortestPath);
                }

                case "-b" -> {
                    if (tokens.length != 2) {
                        System.out.println("Invalid input. -i <sid>.");
                        continue;
                    }
                    List<Long> list = graph.BFS(Long.parseLong(tokens[1]));
                    if(list.isEmpty()) {
                        System.out.println("No Breadth First Search from this NODE :(");
                    }
                    for(Long l : list) {
                        System.out.println(l);
                    }
                }

                case "-a" -> {
                    if (tokens.length != 3) {
                        System.out.println("Invalid input. Use: -a <sid> <eid>");
                        continue;
                    }
                    long sourceID = Long.parseLong(tokens[1]);
                    long destinationID = Long.parseLong(tokens[2]);

                    // Benchmark Start
                    long startTime = System.nanoTime();
                    List<Vertex> shortestPath = graph.AStar(sourceID, destinationID);
                    long endTime = System.nanoTime();

                    if (shortestPath.isEmpty()) {
                        System.out.println("No path found via A*.");
                        continue;
                    }

                    System.out.printf("A* found path in %.2f ms\n", (endTime - startTime) / 1_000_000.0);

                    openMapInBrowser(shortestPath);
                }

                case "-d" -> {
                    if (tokens.length != 2) {
                        System.out.println("Invalid input. -i <sid>.");
                        continue;
                    }
                    List<Long> list = graph.DFS(Long.parseLong(tokens[1]));
                    if(list.isEmpty()) {
                        System.out.println("No Depth First Search from this NODE :(");
                    }
                    for(Long l : list) {
                        System.out.println(l);
                    }
                }
                case "-q" -> {
                    System.out.println("Goodbye!!");
                    return;
                }
                case "-r" -> {
                    if (graph.getVertexCount() == 0) {
                        System.out.println("Graph is empty. Load a file first (-i).");
                        continue;
                    }

                    java.util.Random dice = new java.util.Random();
                    int maxIndex = graph.getVertexCount(); // size of the list

                    int index1 = dice.nextInt(maxIndex);
                    int index2 = dice.nextInt(maxIndex);

                    Vertex v1 = graph.getVertex(index1);
                    Vertex v2 = graph.getVertex(index2);

                    System.out.println("Random Start Node: " + v1.getId() + " (Lat: " + v1.getLatitude() + ")");
                    System.out.println("Random End Node:   " + v2.getId() + " (Lat: " + v2.getLatitude() + ")");
                    System.out.println("------------------------------------------------");
                    System.out.println("Test Command: -p " + v1.getId() + " " + v2.getId());
                }
                default -> System.out.println("Are you NUTS?:??? wtf are you typing!");
            }
        }
    }

    private static void openMapInBrowser(List<Vertex> shortestPath) {
        StringBuilder sb = new StringBuilder();
        sb.append("https://www.google.com/maps/dir");

        int step = Math.max(1, shortestPath.size() / 20);
        for (int i = 0; i < shortestPath.size(); i++) {
            if (i == 0 || i == shortestPath.size() - 1 || i % step == 0) {
                Vertex v = shortestPath.get(i);
                sb.append("/").append(v.getLatitude()).append(",").append(v.getLongitude());
            }
        }

        String url = sb.toString();
        if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
            try {
                Desktop.getDesktop().browse(new URI(url));
                System.out.println("Map opened in browser.");
            } catch (Exception e) {
                System.out.println("Failed to open browser.");
            }
        } else {
            System.out.println("URL: " + url);
        }
    }
}
