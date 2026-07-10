package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.Freerouting;
import app.freerouting.board.FixedState;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.Trace;
import app.freerouting.board.Unit;
import app.freerouting.geometry.planar.FloatLine;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;
import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Point;
import app.freerouting.io.BoardReadResult;
import app.freerouting.io.specctra.DsnReader;
import app.freerouting.rules.Net;
import app.freerouting.rules.ViaInfo;
import app.freerouting.settings.GlobalSettings;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.RouterSettings;
import java.io.ByteArrayInputStream;
import java.nio.charset.StandardCharsets;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BatchAutorouterRouterIntentTest {

  private static final String USB_D_PLUS = "USB_D+";
  private static final String USB_D_MINUS = "USB_D-";
  private static final String TWO_LAYER_PAIR_DSN =
      "(pcb diff-pair-sibling-layer-test\n"
          + "  (parser (string_quote \"))\n"
          + "  (resolution um 10)\n"
          + "  (unit um)\n"
          + "  (structure\n"
          + "    (layer F.Cu (type signal) (property (index 0)))\n"
          + "    (layer B.Cu (type signal) (property (index 1)))\n"
          + "    (boundary (path pcb 0  0 0  100000 0  100000 -60000  0 -60000  0 0))\n"
          + "    (via \"Via[0-1]_600:300_um\")\n"
          + "    (rule (width 500) (clearance 500) (clearance 100 (type smd_smd)))\n"
          + "  )\n"
          + "  (library\n"
          + "    (padstack Rect[T]Pad_1000x1000_um\n"
          + "      (shape (rect F.Cu -500 -500 500 500))\n"
          + "      (shape (rect B.Cu -500 -500 500 500))\n"
          + "      (attach off)\n"
          + "    )\n"
          + "    (padstack \"Via[0-1]_600:300_um\"\n"
          + "      (shape (circle F.Cu 600))\n"
          + "      (shape (circle B.Cu 600))\n"
          + "      (attach off)\n"
          + "    )\n"
          + "    (image Pad_1x1mm\n"
          + "      (pin Rect[T]Pad_1000x1000_um 1 0 0)\n"
          + "    )\n"
          + "  )\n"
          + "  (placement\n"
          + "    (component Pad_1x1mm\n"
          + "      (place P1 10000 -20000 front 0.000000)\n"
          + "      (place P2 90000 -20000 front 0.000000)\n"
          + "      (place N1 10000 -30000 front 0.000000)\n"
          + "      (place N2 90000 -30000 front 0.000000)\n"
          + "    )\n"
          + "  )\n"
          + "  (network\n"
          + "    (net USB_D+ (pins P1-1 P2-1))\n"
          + "    (net USB_D- (pins N1-1 N2-1))\n"
          + "    (class default \"\" USB_D+ USB_D-\n"
          + "      (rule (width 500) (clearance 500))\n"
          + "      (circuit (use_via \"Via[0-1]_600:300_um\") (use_layer F.Cu B.Cu))\n"
          + "    )\n"
          + "  )\n"
          + "  (wiring)\n"
          + ")\n";
  private static final String FOUR_LAYER_PAIR_DSN =
      "(pcb diff-pair-balanced-via-test\n"
          + "  (parser (string_quote \"))\n"
          + "  (resolution um 10)\n"
          + "  (unit um)\n"
          + "  (structure\n"
          + "    (layer F.Cu (type signal) (property (index 0)))\n"
          + "    (layer In1.Cu (type signal) (property (index 1)))\n"
          + "    (layer In2.Cu (type signal) (property (index 2)))\n"
          + "    (layer B.Cu (type signal) (property (index 3)))\n"
          + "    (boundary (path pcb 0  0 0  100000 0  100000 -60000  0 -60000  0 0))\n"
          + "    (via \"Via[0-3]_600:300_um\")\n"
          + "    (rule (width 500) (clearance 500) (clearance 100 (type smd_smd)))\n"
          + "  )\n"
          + "  (library\n"
          + "    (padstack Rect[T]Pad_1000x1000_um\n"
          + "      (shape (rect F.Cu -500 -500 500 500))\n"
          + "      (shape (rect In1.Cu -500 -500 500 500))\n"
          + "      (shape (rect In2.Cu -500 -500 500 500))\n"
          + "      (shape (rect B.Cu -500 -500 500 500))\n"
          + "      (attach off)\n"
          + "    )\n"
          + "    (padstack \"Via[0-3]_600:300_um\"\n"
          + "      (shape (circle F.Cu 600))\n"
          + "      (shape (circle In1.Cu 600))\n"
          + "      (shape (circle In2.Cu 600))\n"
          + "      (shape (circle B.Cu 600))\n"
          + "      (attach off)\n"
          + "    )\n"
          + "    (image Pad_1x1mm\n"
          + "      (pin Rect[T]Pad_1000x1000_um 1 0 0)\n"
          + "    )\n"
          + "  )\n"
          + "  (placement\n"
          + "    (component Pad_1x1mm\n"
          + "      (place P1 10000 -20000 front 0.000000)\n"
          + "      (place P2 90000 -20000 front 0.000000)\n"
          + "      (place N1 10000 -30000 front 0.000000)\n"
          + "      (place N2 90000 -30000 front 0.000000)\n"
          + "    )\n"
          + "  )\n"
          + "  (network\n"
          + "    (net USB_D+ (pins P1-1 P2-1))\n"
          + "    (net USB_D- (pins N1-1 N2-1))\n"
          + "    (class default \"\" USB_D+ USB_D-\n"
          + "      (rule (width 500) (clearance 500))\n"
          + "      (circuit (use_via \"Via[0-3]_600:300_um\") (use_layer F.Cu In1.Cu In2.Cu B.Cu))\n"
          + "    )\n"
          + "  )\n"
          + "  (wiring)\n"
          + ")\n";

  @BeforeEach
  void setUp() {
    Freerouting.globalSettings = new GlobalSettings();
  }

  @Test
  void traceCostsPreferLayerUsedByAlreadyRoutedDifferentialPairSibling() throws Exception {
    RoutingBoard board = loadBoard(TWO_LAYER_PAIR_DSN);
    Net positive = board.rules.nets.get(USB_D_PLUS, 1);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(positive);
    assertNotNull(negative);
    insertSiblingTrace(board, positive, 1);

    RouterSettings settings = routerSettings(board);
    BatchAutorouter router = new BatchAutorouter(
        null,
        board,
        settings,
        true,
        false,
        settings.get_start_ripup_costs(),
        500);

    AutorouteControl.ExpansionCostFactor[] costs = router.traceCostsForRouterIntent(negative.net_number);

    assertEquals(2, costs.length);
    assertTrue(costs[0].horizontal > costs[1].horizontal);
    assertTrue(costs[0].vertical > costs[1].vertical);
    assertEquals(3.0, costs[0].horizontal / costs[1].horizontal, 0.01);
    assertEquals(3.0, costs[0].vertical / costs[1].vertical, 0.01);
  }

  @Test
  void pairCorridorPenaltyIncreasesWithDistanceFromAlreadyRoutedSibling() throws Exception {
    RoutingBoard board = loadBoard(TWO_LAYER_PAIR_DSN);
    Net positive = board.rules.nets.get(USB_D_PLUS, 1);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(positive);
    assertNotNull(negative);
    Trace siblingTrace = insertSiblingTrace(board, positive, 1);

    RouterSettings settings = routerSettings(board);
    BatchAutorouter router = new BatchAutorouter(
        null,
        board,
        settings,
        true,
        false,
        settings.get_start_ripup_costs(),
        500);

    AutorouteControl control = new AutorouteControl(
        board,
        negative.net_number,
        settings,
        settings.get_via_costs(),
        router.traceCostsForRouterIntent(negative.net_number));
    IntBox[] corridors = router.routedDifferentialPairSiblingCorridors(
        control,
        settings.intent,
        USB_D_MINUS);
    control.setRouterIntentPairCorridors(corridors);

    assertEquals(1, corridors.length);
    IntBox legacyCorridor = siblingTrace.bounding_box().offset(siblingTrace.get_half_width() * 3.0);
    assertEquals(legacyCorridor.ll, corridors[0].ll);
    assertEquals(legacyCorridor.ur, corridors[0].ur);
    assertEquals(0.0, control.routerIntentPairCorridorPenalty(new FloatPoint(50_000, -20_000), 1));
    assertTrue(control.routerIntentPairCorridorPenalty(new FloatPoint(50_000, -50_000), 1) > 0.0);
  }

  @Test
  void coupledPairCenterlineGuidePenalizesPointsHiddenInsideDiagonalTraceBounds() throws Exception {
    RoutingBoard board = loadBoard(TWO_LAYER_PAIR_DSN);
    Net positive = board.rules.nets.get(USB_D_PLUS, 1);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(positive);
    assertNotNull(negative);
    insertTrace(
        board,
        positive,
        1,
        new Point[] {
            new IntPoint(10_000, -10_000),
            new IntPoint(90_000, -50_000)
        });

    RouterSettings settings = routerSettings(board, coupledDifferentialPairIntent(1.0, 0.1));
    BatchAutorouter router = new BatchAutorouter(
        null,
        board,
        settings,
        true,
        false,
        settings.get_start_ripup_costs(),
        500);
    AutorouteControl control = new AutorouteControl(
        board,
        negative.net_number,
        settings,
        settings.get_via_costs(),
        router.traceCostsForRouterIntent(negative.net_number));

    router.applyRouterIntentPairCorridors(control, settings.intent, USB_D_MINUS);

    FloatPoint pointInsideWholeTraceBounds = new FloatPoint(50_000, -10_000);
    assertEquals(1, control.router_intent_pair_centerline_guides.length);
    assertEquals(0.0, control.router_intent_pair_corridors[0].distance(pointInsideWholeTraceBounds));
    assertTrue(control.routerIntentPairCorridorPenalty(pointInsideWholeTraceBounds, 1) > 0.0);
    assertEquals(0.0, control.routerIntentPairCorridorPenalty(pointInsideWholeTraceBounds, 0));
  }

  @Test
  void coupledPairCenterlineBandUsesTargetGapAndBothTraceHalfWidths() throws Exception {
    RoutingBoard board = loadBoard(TWO_LAYER_PAIR_DSN);
    Net positive = board.rules.nets.get(USB_D_PLUS, 1);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(positive);
    assertNotNull(negative);
    Trace siblingTrace = insertSiblingTrace(board, positive, 1);

    double targetGapMm = 1.0;
    double toleranceMm = 0.2;
    RouterSettings settings = routerSettings(
        board,
        coupledDifferentialPairIntent(targetGapMm, toleranceMm));
    BatchAutorouter router = new BatchAutorouter(
        null,
        board,
        settings,
        true,
        false,
        settings.get_start_ripup_costs(),
        500);
    AutorouteControl control = new AutorouteControl(
        board,
        negative.net_number,
        settings,
        settings.get_via_costs(),
        router.traceCostsForRouterIntent(negative.net_number));

    router.applyRouterIntentPairCorridors(control, settings.intent, USB_D_MINUS);

    assertEquals(1, control.router_intent_pair_centerline_guides.length);
    AutorouteControl.PairCenterlineGuide guide = control.router_intent_pair_centerline_guides[0];
    double mmResolution = board.communication.get_resolution(Unit.MM);
    double expectedCenterSpacing = targetGapMm * mmResolution
        + control.trace_half_width[1]
        + siblingTrace.get_half_width();
    double expectedTolerance = toleranceMm * mmResolution;
    assertEquals(expectedCenterSpacing, guide.targetCenterSpacing, 1e-6);
    assertEquals(expectedTolerance, guide.tolerance, 1e-6);

    FloatPoint targetPoint = new FloatPoint(50_000, -20_000 + expectedCenterSpacing);
    FloatPoint insideBand = new FloatPoint(
        50_000,
        -20_000 + expectedCenterSpacing + expectedTolerance * 0.5);
    FloatPoint outsideBand = new FloatPoint(
        50_000,
        -20_000 + expectedCenterSpacing + expectedTolerance + 0.5 * mmResolution);
    FloatPoint tooClose = new FloatPoint(
        50_000,
        -20_000 + expectedCenterSpacing - expectedTolerance - 0.5 * mmResolution);
    FloatPoint beyondGuideEnd = new FloatPoint(
        95_000,
        -20_000 + expectedCenterSpacing + expectedTolerance + 0.5 * mmResolution);
    FloatPoint farOutsideBand = new FloatPoint(
        50_000,
        -20_000 + expectedCenterSpacing + expectedTolerance + 5.0 * mmResolution);
    assertEquals(0.0, guide.bandDeviation(targetPoint), 1e-6);
    assertEquals(0.0, guide.bandDeviation(insideBand), 1e-6);
    assertEquals(0.0, control.router_intent_pair_corridors[0].distance(targetPoint), 1e-6);
    assertEquals(0.0, control.router_intent_pair_corridors[0].distance(insideBand), 1e-6);
    assertTrue(control.router_intent_pair_corridors[0].distance(outsideBand) > 0.0);
    assertEquals(0.0, control.routerIntentPairCorridorPenalty(targetPoint, 1), 1e-6);
    assertEquals(0.0, control.routerIntentPairCorridorPenalty(insideBand, 1), 1e-6);
    assertTrue(control.routerIntentPairCorridorPenalty(outsideBand, 1) > 0.0);
    assertTrue(control.routerIntentPairCorridorPenalty(tooClose, 1) > 0.0);
    assertTrue(control.routerIntentPairCorridorPenalty(beyondGuideEnd, 1) > 0.0);
    assertTrue(
        control.routerIntentPairCorridorPenalty(farOutsideBand, 1)
            > control.routerIntentPairCorridorPenalty(outsideBand, 1));
    assertEquals(0.0, control.routerIntentPairCorridorPenalty(outsideBand, 0), 1e-6);
  }

  @Test
  void outOfSpanSiblingSegmentDoesNotMaskApplicableCenterlinePenalty() throws Exception {
    RoutingBoard board = loadBoard(TWO_LAYER_PAIR_DSN);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(negative);

    RouterSettings settings = routerSettings(board, coupledDifferentialPairIntent(1.0, 0.1));
    AutorouteControl control = new AutorouteControl(
        board,
        negative.net_number,
        settings,
        settings.get_via_costs(),
        settings.get_trace_cost_arr());
    control.setRouterIntentPairCenterlineGuides(new AutorouteControl.PairCenterlineGuide[] {
        new AutorouteControl.PairCenterlineGuide(
            new FloatLine(
                new FloatPoint(10_000, -20_000),
                new FloatPoint(90_000, -20_000)),
            1,
            10_000,
            1_000),
        new AutorouteControl.PairCenterlineGuide(
            new FloatLine(
                new FloatPoint(10_000, -40_000),
                new FloatPoint(20_000, -40_000)),
            1,
            10_000,
            1_000)
    });

    double averageLayerTraceCost = (
        control.trace_costs[1].horizontal + control.trace_costs[1].vertical) / 2.0;
    assertEquals(
        10_000 * averageLayerTraceCost * 2.5,
        control.routerIntentPairCorridorPenalty(new FloatPoint(50_000, -50_000), 1),
        1e-6);
  }

  @Test
  void centerlineAndCorridorExitPenaltiesAccumulate() throws Exception {
    RoutingBoard board = loadBoard(TWO_LAYER_PAIR_DSN);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(negative);

    RouterSettings settings = routerSettings(board, coupledDifferentialPairIntent(1.0, 0.1));
    AutorouteControl control = new AutorouteControl(
        board,
        negative.net_number,
        settings,
        settings.get_via_costs(),
        settings.get_trace_cost_arr());
    control.setRouterIntentPairCenterlineGuides(new AutorouteControl.PairCenterlineGuide[] {
        horizontalGuide(0, 0, 100_000, 10_000, 1_000)
    });
    control.setRouterIntentPairCorridors(
        new IntBox[] { new IntBox(0, -5_000, 100_000, 5_000) },
        new int[] { 1 },
        -1);

    FloatPoint point = new FloatPoint(50_000, 30_000);
    double averageLayerTraceCost = (
        control.trace_costs[1].horizontal + control.trace_costs[1].vertical) / 2.0;
    double centerlinePenalty = 10_000 * averageLayerTraceCost
        * RouterIntentRoutingPolicy.differentialPairCenterlineBandCostFactor(settings.intent, USB_D_MINUS);
    double corridorPenalty = 25_000 * averageLayerTraceCost
        * RouterIntentRoutingPolicy.differentialPairCorridorExitCostFactor(settings.intent, USB_D_MINUS);

    assertEquals(
        centerlinePenalty + corridorPenalty,
        control.routerIntentPairCorridorPenalty(point, 1),
        1e-6);
  }

  @Test
  void nearestParallelCenterlineWinsEvenWhenFarGuideHasLowerBandDeviation() throws Exception {
    RoutingBoard board = loadBoard(TWO_LAYER_PAIR_DSN);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(negative);

    RouterSettings settings = routerSettings(board, coupledDifferentialPairIntent(1.0, 0.1));
    AutorouteControl control = new AutorouteControl(
        board,
        negative.net_number,
        settings,
        settings.get_via_costs(),
        settings.get_trace_cost_arr());
    AutorouteControl.PairCenterlineGuide nearGuide = horizontalGuide(
        0, 0, 100_000, 10_000, 1_000);
    AutorouteControl.PairCenterlineGuide fartherGuide = horizontalGuide(
        0, 12_000, 100_000, 10_000, 1_000);
    control.setRouterIntentPairCenterlineGuides(new AutorouteControl.PairCenterlineGuide[] {
        nearGuide,
        fartherGuide
    });

    FloatPoint point = new FloatPoint(50_000, 2_000);
    assertEquals(2_000, nearGuide.applicableDistance(point), 1e-6);
    assertEquals(10_000, fartherGuide.applicableDistance(point), 1e-6);
    assertEquals(7_000, nearGuide.bandDeviation(point), 1e-6);
    assertEquals(0.0, fartherGuide.bandDeviation(point), 1e-6);
    assertTrue(control.routerIntentPairCorridorPenalty(point, 1) > 0.0);
  }

  @Test
  void nearestFoldedCenterlineSegmentWinsAcrossOverlappingSpans() throws Exception {
    RoutingBoard board = loadBoard(TWO_LAYER_PAIR_DSN);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(negative);

    RouterSettings settings = routerSettings(board, coupledDifferentialPairIntent(1.0, 0.1));
    AutorouteControl control = new AutorouteControl(
        board,
        negative.net_number,
        settings,
        settings.get_via_costs(),
        settings.get_trace_cost_arr());
    AutorouteControl.PairCenterlineGuide horizontal = horizontalGuide(
        0, 0, 100_000, 10_000, 1_000);
    AutorouteControl.PairCenterlineGuide vertical = new AutorouteControl.PairCenterlineGuide(
        new FloatLine(
            new FloatPoint(50_000, 0),
            new FloatPoint(50_000, 100_000)),
        1,
        10_000,
        1_000);
    control.setRouterIntentPairCenterlineGuides(new AutorouteControl.PairCenterlineGuide[] {
        horizontal,
        vertical
    });

    FloatPoint point = new FloatPoint(52_000, 10_000);
    assertEquals(10_000, horizontal.applicableDistance(point), 1e-6);
    assertEquals(2_000, vertical.applicableDistance(point), 1e-6);
    assertEquals(0.0, horizontal.bandDeviation(point), 1e-6);
    assertEquals(7_000, vertical.bandDeviation(point), 1e-6);
    assertTrue(control.routerIntentPairCorridorPenalty(point, 1) > 0.0);
  }

  @Test
  void centerlineBandDeviationSaturatesAtOneTargetSpacing() {
    AutorouteControl.PairCenterlineGuide guide = new AutorouteControl.PairCenterlineGuide(
        new FloatLine(
            new FloatPoint(10_000, -20_000),
            new FloatPoint(90_000, -20_000)),
        1,
        10_000,
        1_000);

    assertEquals(10_000, guide.boundedBandDeviation(new FloatPoint(50_000, -100_000)), 1e-6);
    assertEquals(0.0, guide.boundedBandDeviation(new FloatPoint(50_000, -30_000)), 1e-6);
    assertTrue(Double.isInfinite(guide.boundedBandDeviation(new FloatPoint(95_000, -100_000))));
  }

  @Test
  void pairCorridorReservationDiscountsForeignObstaclesOnlyInsideSiblingCorridor() throws Exception {
    RoutingBoard board = loadBoard(TWO_LAYER_PAIR_DSN);
    Net positive = board.rules.nets.get(USB_D_PLUS, 1);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(positive);
    assertNotNull(negative);
    Trace siblingTrace = insertSiblingTrace(board, positive, 1);
    Net foreign = board.rules.nets.add("VBUS", 1, false);
    Trace crossingCorridor = insertTrace(
        board,
        foreign,
        1,
        new Point[] {
            new IntPoint(50_000, -25_000),
            new IntPoint(50_000, -15_000)
        });
    Trace outsideCorridor = insertTrace(
        board,
        foreign,
        1,
        new Point[] {
            new IntPoint(50_000, -55_000),
            new IntPoint(90_000, -55_000)
        });

    RouterSettings settings = routerSettings(board);
    BatchAutorouter router = new BatchAutorouter(
        null,
        board,
        settings,
        true,
        false,
        settings.get_start_ripup_costs(),
        500);
    AutorouteControl control = new AutorouteControl(
        board,
        negative.net_number,
        settings,
        settings.get_via_costs(),
        router.traceCostsForRouterIntent(negative.net_number));

    router.applyRouterIntentPairCorridors(control, settings.intent, USB_D_MINUS);

    assertTrue(control.routerIntentPairCorridorRipupCostFactor(crossingCorridor) < 1.0);
    assertEquals(1.0, control.routerIntentPairCorridorRipupCostFactor(outsideCorridor));
    assertEquals(1.0, control.routerIntentPairCorridorRipupCostFactor(siblingTrace));
  }

  @Test
  void differentialPairSkewLimitPenalizesOverlongCandidatePathsBeforeMazeSearch() throws Exception {
    RoutingBoard board = loadBoard(TWO_LAYER_PAIR_DSN);
    Net positive = board.rules.nets.get(USB_D_PLUS, 1);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(positive);
    assertNotNull(negative);
    insertSiblingTrace(board, positive, 1);

    RouterSettings settings = routerSettings(board);
    BatchAutorouter router = new BatchAutorouter(
        null,
        board,
        settings,
        true,
        false,
        settings.get_start_ripup_costs(),
        500);
    AutorouteControl control = new AutorouteControl(
        board,
        negative.net_number,
        settings,
        settings.get_via_costs(),
        router.traceCostsForRouterIntent(negative.net_number));

    router.applyRouterIntentPairSkewLimit(control, settings.intent, USB_D_MINUS);

    double mmResolution = board.communication.get_resolution(Unit.MM);
    double siblingLength = positive.get_trace_length();
    assertEquals(0.0, control.routerIntentPairSkewPenalty(siblingLength + 0.5 * mmResolution, 1));
    assertTrue(control.routerIntentPairSkewPenalty(siblingLength + 2.0 * mmResolution, 1) > 0.0);
  }

  @Test
  void differentialPairIntentRaisesViaCostBeforeMazeSearch() throws Exception {
    RoutingBoard board = loadBoard(TWO_LAYER_PAIR_DSN);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(negative);

    RouterSettings baselineSettings = routerSettings(board, null);
    RouterSettings pairSettings = routerSettings(board, differentialPairIntent());
    AutorouteControl baselineControl = new AutorouteControl(
        board,
        negative.net_number,
        baselineSettings,
        baselineSettings.get_via_costs(),
        baselineSettings.get_trace_cost_arr());
    AutorouteControl pairControl = new AutorouteControl(
        board,
        negative.net_number,
        pairSettings,
        pairSettings.get_via_costs(),
        pairSettings.get_trace_cost_arr());

    assertEquals(1.5, pairControl.min_normal_via_cost / baselineControl.min_normal_via_cost, 0.01);
  }

  @Test
  void pairViaTransitionCostsPreferAlreadyRoutedSiblingTransition() throws Exception {
    RoutingBoard board = loadBoard(FOUR_LAYER_PAIR_DSN);
    Net positive = board.rules.nets.get(USB_D_PLUS, 1);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(positive);
    assertNotNull(negative);
    insertSiblingVia(board, positive);

    RouterSettings settings = routerSettings(board);
    BatchAutorouter router = new BatchAutorouter(
        null,
        board,
        settings,
        true,
        false,
        settings.get_start_ripup_costs(),
        500);
    AutorouteControl control = new AutorouteControl(
        board,
        negative.net_number,
        settings,
        settings.get_via_costs(),
        settings.get_trace_cost_arr());

    router.applyRouterIntentPairViaTransitionCosts(control, settings.intent, USB_D_MINUS);

    assertEquals(0, control.add_via_costs[0].to_layer[3]);
    assertEquals(0, control.add_via_costs[3].to_layer[0]);
    assertTrue(control.add_via_costs[0].to_layer[1] > 0);
    assertTrue(control.add_via_costs[1].to_layer[3] > 0);
  }

  private Trace insertSiblingTrace(RoutingBoard board, Net net, int layer) {
    return insertTrace(
        board,
        net,
        layer,
        new Point[] {
            new IntPoint(10_000, -20_000),
            new IntPoint(90_000, -20_000)
        });
  }

  private Trace insertTrace(RoutingBoard board, Net net, int layer, Point[] points) {
    int halfWidth = board.rules.get_trace_half_width(net.net_number, layer);
    int clearanceClass = net.get_class().get_trace_clearance_class();
    board.insert_trace(
        points,
        layer,
        halfWidth,
        new int[] { net.net_number },
        clearanceClass,
        FixedState.UNFIXED);
    return findTrace(board, net, layer, points[0], points[points.length - 1]);
  }

  private Trace findTrace(RoutingBoard board, Net net, int layer, Point start, Point end) {
    for (Trace trace : board.get_traces()) {
      if (!trace.contains_net(net.net_number) || trace.get_layer() != layer) {
        continue;
      }
      boolean sameDirection = start.equals(trace.first_corner()) && end.equals(trace.last_corner());
      boolean reverseDirection = end.equals(trace.first_corner()) && start.equals(trace.last_corner());
      if (sameDirection || reverseDirection) {
        return trace;
      }
    }
    throw new IllegalStateException("inserted trace was not found for net " + net.name);
  }

  private void insertSiblingVia(RoutingBoard board, Net net) {
    ViaInfo viaInfo = board.rules.get_default_via_rule().get_via(0);
    board.insert_via(
        viaInfo.get_padstack(),
        new IntPoint(50_000, -20_000),
        new int[] { net.net_number },
        viaInfo.get_clearance_class(),
        FixedState.UNFIXED,
        viaInfo.attach_smd_allowed());
  }

  private RouterSettings routerSettings(RoutingBoard board) {
    return routerSettings(board, differentialPairIntent());
  }

  private RouterSettings routerSettings(RoutingBoard board, RouterIntentSettings intent) {
    RouterSettings settings = new RouterSettings(board);
    settings.automatic_neckdown = false;
    settings.intent = intent;
    return settings;
  }

  private RouterIntentSettings differentialPairIntent() {
    RouterIntentSettings.DifferentialPairIntent pair = new RouterIntentSettings.DifferentialPairIntent();
    pair.id = "usb2_data";
    pair.positiveNet = USB_D_PLUS;
    pair.negativeNet = USB_D_MINUS;
    pair.priority = RouterIntentSettings.Priority.CRITICAL;
    pair.maxSkewMm = 1.0;

    RouterIntentSettings intent = new RouterIntentSettings();
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] { pair };
    return intent;
  }

  private RouterIntentSettings coupledDifferentialPairIntent(double targetGapMm, double toleranceMm) {
    RouterIntentSettings intent = differentialPairIntent();
    RouterIntentSettings.DifferentialPairIntent pair = intent.differentialPairs[0];
    pair.routeAsCoupledPair = true;
    pair.targetGapMm = targetGapMm;
    pair.gapToleranceMm = toleranceMm;
    return intent;
  }

  private static AutorouteControl.PairCenterlineGuide horizontalGuide(
      double startX,
      double y,
      double endX,
      double targetCenterSpacing,
      double tolerance) {
    return new AutorouteControl.PairCenterlineGuide(
        new FloatLine(
            new FloatPoint(startX, y),
            new FloatPoint(endX, y)),
        1,
        targetCenterSpacing,
        tolerance);
  }

  private static RoutingBoard loadBoard(String dsn) throws Exception {
    BoardReadResult result = DsnReader.readBoard(
        new ByteArrayInputStream(dsn.getBytes(StandardCharsets.UTF_8)), null, null);
    return switch (result) {
      case BoardReadResult.Success success -> (RoutingBoard) success.board();
      case BoardReadResult.OutlineMissing outlineMissing -> (RoutingBoard) outlineMissing.board();
      case BoardReadResult.ParseError parseError ->
          throw new IllegalStateException(parseError.location() + ": " + parseError.detail());
      case BoardReadResult.IoError ioError -> throw ioError.cause();
    };
  }
}
