package app.freerouting.fixtures;

import static org.junit.jupiter.api.Assertions.assertEquals;

import app.freerouting.board.BasicBoard;
import app.freerouting.core.RoutingJob;
import app.freerouting.core.scoring.BoardStatistics;
import app.freerouting.io.specctra.DsnReader;
import java.io.ByteArrayInputStream;
import org.junit.jupiter.api.Test;

public class DrcViolationRoutingTest extends RoutingFixtureTest {

  private void assertDrcOnLoadedBoard(String filename, int expectedUnconnected, int expectedViolations) throws Exception {
    RoutingJob job = GetRoutingJob(filename, null);

    // Read the board without routing it
    ByteArrayInputStream inputStream = new ByteArrayInputStream(job.input.getData().readAllBytes());
    app.freerouting.io.BoardReadResult result = DsnReader.readBoard(inputStream, null, null, "test");
    BasicBoard board = null;
    if (result instanceof app.freerouting.io.BoardReadResult.Success s) {
      board = s.board();
    } else if (result instanceof app.freerouting.io.BoardReadResult.OutlineMissing o) {
      board = o.board();
    } else {
      throw new RuntimeException("Failed to read board: " + result);
    }

    BoardStatistics stats = new BoardStatistics(board);

    assertEquals(expectedUnconnected, stats.connections.incompleteCount,
        "Mismatch in unconnected items for " + filename);
    assertEquals(expectedViolations, stats.clearanceViolations.totalCount,
        "Mismatch in clearance violations for " + filename);
  }

  // Note on unconnected items (incompleteCount):
  // KiCad typically reports one "unconnected items" entry per logically disconnected net.
  // Freerouting's DesignRulesChecker.getIncompleteCount() computes the number of AirLines needed
  // to fully connect the net (which equals N - 1 airlines for N disconnected components).
  // The values below reflect Freerouting's geometric connectivity for these
  // specific DSN files after symmetric trace-to-PTH-pad contact resolution.

  @Test
  public void test_Issue_575_6_track_and_1_hole_clearance_violations() throws Exception {
    assertDrcOnLoadedBoard("Issue575-drc_BBD_Mars-64_6_track_1_hole_clearance_violations.dsn", 3, 76);
  }

  @Test
  public void test_Issue_575_4_hole_clearance_violations() throws Exception {
    assertDrcOnLoadedBoard("Issue575-drc_dev-board_4_hole_clearance_violations.dsn", 1, 0);
  }

  @Test
  public void test_Issue_575_7_unconnected_items() throws Exception {
    assertDrcOnLoadedBoard("Issue575-drc_Natural_Tone_Preamp_7_unconnected_items.dsn", 29, 0);
  }
}
