package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.Freerouting;
import app.freerouting.board.FixedState;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.Unit;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Point;
import app.freerouting.io.BoardReadResult;
import app.freerouting.io.specctra.DsnReader;
import app.freerouting.rules.Net;
import app.freerouting.settings.GlobalSettings;
import java.io.ByteArrayInputStream;
import java.nio.charset.StandardCharsets;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DifferentialPairAutorouterTest {

  private static final String TWO_NET_PAIR_DSN =
      "(pcb diff-pair-route-test\n"
          + "  (parser (string_quote \"))\n"
          + "  (resolution um 10)\n"
          + "  (unit um)\n"
          + "  (structure\n"
          + "    (layer F.Cu (type signal) (property (index 0)))\n"
          + "    (layer B.Cu (type signal) (property (index 1)))\n"
          + "    (boundary\n"
          + "      (path pcb 0  0 0  50000 0  50000 30000  0 30000  0 0)\n"
          + "    )\n"
          + "  )\n"
          + "  (network\n"
          + "    (net USB_D+)\n"
          + "    (net USB_D-)\n"
          + "    (pair (nets USB_D+ USB_D-))\n"
          + "  )\n"
          + ")\n";

  @BeforeEach
  void setUp() {
    Freerouting.globalSettings = new GlobalSettings();
  }

  @Test
  void lengthensShorterDifferentialPairMemberUntilSkewIsWithinLimit() throws Exception {
    RoutingBoard board = loadBoard(TWO_NET_PAIR_DSN);
    Net positive = board.rules.nets.get("USB_D+", 1);
    Net negative = board.rules.nets.get("USB_D-", 1);
    assertNotNull(positive);
    assertNotNull(negative);

    int layer = 0;
    int halfWidth = board.rules.get_trace_half_width(positive.net_number, layer);
    int clearanceClass = positive.get_class().get_trace_clearance_class();

    board.insert_trace(new Point[] {
        new IntPoint(50_000, 100_000),
        new IntPoint(150_000, 100_000)
    }, layer, halfWidth, new int[] { positive.net_number }, clearanceClass, FixedState.UNFIXED);
    board.insert_trace(new Point[] {
        new IntPoint(50_000, 200_000),
        new IntPoint(130_000, 200_000)
    }, layer, halfWidth, new int[] { negative.net_number }, clearanceClass, FixedState.UNFIXED);

    double oneMm = board.communication.get_resolution(Unit.MM);
    double beforeSkew = Math.abs(positive.get_trace_length() - negative.get_trace_length());
    assertTrue(beforeSkew > oneMm);

    int edits = new DifferentialPairAutorouter(null, board, 1.0).run();

    double afterSkew = Math.abs(positive.get_trace_length() - negative.get_trace_length());
    assertTrue(edits > 0);
    assertTrue(afterSkew <= oneMm, "expected skew <= 1 mm, got " + (afterSkew / oneMm) + " mm");
    assertEquals(0, new DesignRulesChecker(board, null).getAllClearanceViolations().size());
  }

  private static RoutingBoard loadBoard(String p_dsn) throws Exception {
    BoardReadResult result = DsnReader.readBoard(
        new ByteArrayInputStream(p_dsn.getBytes(StandardCharsets.UTF_8)), null, null);
    return switch (result) {
      case BoardReadResult.Success success -> (RoutingBoard) success.board();
      case BoardReadResult.OutlineMissing outlineMissing -> (RoutingBoard) outlineMissing.board();
      case BoardReadResult.ParseError parseError ->
          throw new IllegalStateException(parseError.location() + ": " + parseError.detail());
      case BoardReadResult.IoError ioError -> throw ioError.cause();
    };
  }
}
