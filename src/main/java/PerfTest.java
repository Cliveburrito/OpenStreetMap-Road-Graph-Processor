import java.util.List;

public class PerfTest {
    public static void main(String[] args) {
        int N = 50000;
        long start = System.currentTimeMillis();

        List<Vertex> vertices = new java.util.ArrayList<>(N);
        java.util.Map<Long, Integer> map = new java.util.HashMap<>((int)(N / 0.75f) + 1);

        for (int i = 0; i < N; i++) {
            Vertex v = new Vertex(i, 39.0 + i * 1e-6, 22.0 + i * 1e-6);
            vertices.add(v);
            map.put(v.getId(), i);
        }

        long end = System.currentTimeMillis();
        System.out.println("Inserted " + N + " vertices in " + (end - start) + " ms");
    }
}
