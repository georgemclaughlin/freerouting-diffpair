package app.freerouting.datastructures;

import static org.junit.jupiter.api.Assertions.assertEquals;

import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Point;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;

class PlanarDelaunayTriangulationDeterminismTest {

  private record StoredPoint(IntPoint point) implements PlanarDelaunayTriangulation.Storable {
    @Override
    public Point[] get_triangulation_corners() {
      return new Point[] {point};
    }
  }

  @Test
  void concurrentTriangulationsUseIndependentFixedSeedGenerators() throws Exception {
    int taskCount = 24;
    ExecutorService executor = Executors.newFixedThreadPool(taskCount);
    CountDownLatch ready = new CountDownLatch(taskCount);
    CountDownLatch start = new CountDownLatch(1);
    try {
      List<CompletableFuture<String>> futures = new ArrayList<>();
      for (int task = 0; task < taskCount; task++) {
        futures.add(CompletableFuture.supplyAsync(() -> {
          ready.countDown();
          try {
            start.await();
          } catch (InterruptedException exception) {
            Thread.currentThread().interrupt();
            throw new IllegalStateException(exception);
          }
          return triangulationSignature();
        }, executor));
      }
      ready.await();
      start.countDown();
      Set<String> signatures = new HashSet<>();
      for (CompletableFuture<String> future : futures) {
        signatures.add(future.join());
      }
      assertEquals(1, signatures.size());
    } finally {
      executor.shutdownNow();
    }
  }

  private String triangulationSignature() {
    List<PlanarDelaunayTriangulation.Storable> points = new ArrayList<>();
    for (int x = 0; x < 7; x++) {
      for (int y = 0; y < 7; y++) {
        points.add(new StoredPoint(new IntPoint(x * 1000, y * 1000)));
      }
    }
    return new PlanarDelaunayTriangulation(points).get_edge_lines().stream()
        .map(edge -> canonicalEdge((IntPoint) edge.start_point, (IntPoint) edge.end_point))
        .sorted()
        .collect(Collectors.joining("|"));
  }

  private String canonicalEdge(IntPoint first, IntPoint second) {
    String a = first.x + "," + first.y;
    String b = second.x + "," + second.y;
    return a.compareTo(b) <= 0 ? a + ":" + b : b + ":" + a;
  }
}
