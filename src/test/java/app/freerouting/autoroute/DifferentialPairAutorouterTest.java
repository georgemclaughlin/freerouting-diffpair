package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.Freerouting;
import app.freerouting.board.FixedState;
import app.freerouting.board.PolylineTrace;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.Unit;
import app.freerouting.core.RoutingJob;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Point;
import app.freerouting.io.BoardReadResult;
import app.freerouting.io.specctra.DsnReader;
import app.freerouting.rules.Net;
import app.freerouting.settings.GlobalSettings;
import app.freerouting.settings.RouterIntentSettings;
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

  @Test
  void routeAsCoupledPairUsesPairAwareMeanderInsteadOfSkippingSkewTuning() throws Exception {
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
    }, layer, halfWidth, new int[] { positive.net_number }, clearanceClass, FixedState.SHOVE_FIXED);
    board.insert_trace(new Point[] {
        new IntPoint(50_000, 104_000),
        new IntPoint(130_000, 104_000)
    }, layer, halfWidth, new int[] { negative.net_number }, clearanceClass, FixedState.SHOVE_FIXED);

    double oneMm = board.communication.get_resolution(Unit.MM);
    double beforeSkew = Math.abs(positive.get_trace_length() - negative.get_trace_length());
    int beforeCornerCount = cornerCount(board, negative.net_number);
    assertTrue(beforeSkew > oneMm);

    RoutingJob job = new RoutingJob();
    job.board = board;
    job.routerSettings.intent = new RouterIntentSettings();
    RouterIntentSettings.DifferentialPairIntent pair = new RouterIntentSettings.DifferentialPairIntent();
    pair.positiveNet = "USB_D+";
    pair.negativeNet = "USB_D-";
    pair.routeAsCoupledPair = true;
    pair.maxSkewMm = 1.0;
    job.routerSettings.intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] { pair };

    int edits = new DifferentialPairAutorouter(job, board, 1.0).run();

    double afterSkew = Math.abs(positive.get_trace_length() - negative.get_trace_length());
    assertTrue(edits > 0);
    assertTrue(afterSkew < beforeSkew, "expected route_as_coupled_pair tuning to improve skew");
    assertTrue(afterSkew <= oneMm + (oneMm * 0.01), "expected skew <= 1.01 mm, got " + (afterSkew / oneMm) + " mm");
    assertTrue(
        maxTraceTurnDegrees(board, negative.net_number) <= 60.0,
        "expected pair-aware meander without sharp 90-degree turns");
    assertTrue(cornerCount(board, negative.net_number) > beforeCornerCount, "expected explicit angular meander geometry");
    assertEquals(0, new DesignRulesChecker(board, null).getAllClearanceViolations().size());
  }

  @Test
  void flowThroughBumpPrimitiveRejectsSharpReturnsAndAcceptsRoundedPlateau() throws Exception {
    RoutingBoard board = loadBoard(TWO_NET_PAIR_DSN);
    double oneMm = board.communication.get_resolution(Unit.MM);
    DifferentialPairAutorouter autorouter = new DifferentialPairAutorouter(null, board, 1.0);
    FloatPoint from = new FloatPoint(10.0 * oneMm, 10.0 * oneMm);
    FloatPoint to = new FloatPoint(80.0 * oneMm, 10.0 * oneMm);
    double height = oneMm;
    double plateau = oneMm;
    double spacing = oneMm;
    double rawSpacing = 1.5 * oneMm;

    Point[] sharpReturn = new Point[] {
        from.round(),
        new IntPoint((int) Math.round(20.0 * oneMm), (int) Math.round(10.0 * oneMm)),
        new IntPoint((int) Math.round(20.0 * oneMm), (int) Math.round(11.0 * oneMm)),
        new IntPoint((int) Math.round(22.0 * oneMm), (int) Math.round(11.0 * oneMm)),
        new IntPoint((int) Math.round(22.0 * oneMm), (int) Math.round(10.0 * oneMm)),
        to.round()
    };
    assertFalse(autorouter.valid_flow_through_bump_shape_for_test(
        sharpReturn,
        from,
        to,
        0.0,
        1.0,
        1,
        height,
        plateau,
        spacing,
        2.0 * oneMm));

    Point[] rounded = autorouter.rounded_outward_bump_path(
        from,
        to,
        0.0,
        1.0,
        2,
        height,
        (3.5 * oneMm),
        plateau,
        rawSpacing,
        false);
    assertNotNull(rounded);
    assertTrue(rounded.length > 20, "expected sampled rounded corners");
    assertTrue(autorouter.valid_flow_through_bump_shape_for_test(
        rounded,
        from,
        to,
        0.0,
        1.0,
        2,
        height,
        plateau,
        spacing,
        2.0 * oneMm));
    assertTrue(maxTurnDegrees(rounded) <= 60.0, "expected no sharp 90-degree return");
  }

  @Test
  void flowThroughPrimitiveFitterProducesFixtureScaleCandidates() throws Exception {
    RoutingBoard board = loadBoard(TWO_NET_PAIR_DSN);
    double oneMm = board.communication.get_resolution(Unit.MM);
    DifferentialPairAutorouter autorouter = new DifferentialPairAutorouter(null, board, 1.0);
    FloatPoint from = new FloatPoint(10.0 * oneMm, 10.0 * oneMm);
    FloatPoint to = new FloatPoint(46.0 * oneMm, 10.0 * oneMm);
    Point[] rounded = autorouter.rounded_outward_bump_path(
        from,
        to,
        0.0,
        1.0,
        3,
        2.5 * oneMm,
        6.1 * oneMm,
        0.8 * oneMm,
        1.2 * oneMm,
        true);
    assertNotNull(rounded);
    assertTrue(autorouter.valid_flow_through_bump_shape_for_test(
        rounded,
        from,
        to,
        0.0,
        1.0,
        3,
        2.5 * oneMm,
        0.8 * oneMm,
        0.8 * oneMm,
        2.5 * oneMm));
    assertNotNull(autorouter.rounded_outward_bump_path(
        from,
        to,
        0.0,
        1.0,
        2,
        0.2 * oneMm,
        1.3 * oneMm,
        0.8 * oneMm,
        0.8 * oneMm,
        true));
    assertTrue(autorouter.max_fit_flow_through_bump_count_for_test(
        from,
        to,
        0.0,
        1.0,
        0.8 * oneMm,
        0.8 * oneMm,
        0.8 * oneMm,
        0.2 * oneMm,
        true) >= 2);

    int candidateCount = autorouter.flow_through_primitive_candidate_count_for_test(
        from,
        to,
        0.0,
        1.0,
        36.0 * oneMm,
        40.4 * oneMm,
        0.8 * oneMm,
        0.8 * oneMm,
        0.8 * oneMm,
        0.2 * oneMm,
        2.5 * oneMm,
        true);

    assertTrue(candidateCount > 0, "expected deterministic outward bump candidates");
  }

  private static double maxTraceTurnDegrees(RoutingBoard p_board, int p_net_no) {
    double result = 0.0;
    for (var trace : p_board.get_traces()) {
      if (!(trace instanceof PolylineTrace polylineTrace) || !polylineTrace.contains_net(p_net_no)) {
        continue;
      }
      Point[] corners = polylineTrace.polyline().corner_arr();
      for (int i = 1; i < corners.length - 1; i++) {
        result = Math.max(result, turnDegrees(corners[i - 1], corners[i], corners[i + 1]));
      }
    }
    return result;
  }

  private static int cornerCount(RoutingBoard p_board, int p_net_no) {
    int result = 0;
    for (var trace : p_board.get_traces()) {
      if (trace instanceof PolylineTrace polylineTrace && polylineTrace.contains_net(p_net_no)) {
        result += polylineTrace.polyline().corner_arr().length;
      }
    }
    return result;
  }

  private static double maxTurnDegrees(Point[] p_points) {
    double result = 0.0;
    for (int i = 1; i < p_points.length - 1; i++) {
      result = Math.max(result, turnDegrees(p_points[i - 1], p_points[i], p_points[i + 1]));
    }
    return result;
  }

  private static double turnDegrees(Point p_before, Point p_corner, Point p_after) {
    double ax = p_corner.to_float().x - p_before.to_float().x;
    double ay = p_corner.to_float().y - p_before.to_float().y;
    double bx = p_after.to_float().x - p_corner.to_float().x;
    double by = p_after.to_float().y - p_corner.to_float().y;
    double aLength = Math.sqrt(ax * ax + ay * ay);
    double bLength = Math.sqrt(bx * bx + by * by);
    if (aLength <= 0.0 || bLength <= 0.0) {
      return 0.0;
    }
    double cos = ((ax * bx) + (ay * by)) / (aLength * bLength);
    cos = Math.max(-1.0, Math.min(1.0, cos));
    return Math.toDegrees(Math.acos(cos));
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
