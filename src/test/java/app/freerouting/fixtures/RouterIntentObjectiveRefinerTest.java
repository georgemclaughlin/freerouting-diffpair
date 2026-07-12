package app.freerouting.fixtures;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.board.Component;
import app.freerouting.board.ConductionArea;
import app.freerouting.board.FixedState;
import app.freerouting.board.Item;
import app.freerouting.board.Pin;
import app.freerouting.board.PolylineTrace;
import app.freerouting.board.Trace;
import app.freerouting.board.Unit;
import app.freerouting.board.Via;
import app.freerouting.autoroute.RouterIntentObjectiveRefiner;
import app.freerouting.core.Padstack;
import app.freerouting.core.RoutingJob;
import app.freerouting.core.RoutingJobState;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;
import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Point;
import app.freerouting.geometry.planar.Polyline;
import app.freerouting.logger.FRLogger;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.sources.TestingSettings;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

class RouterIntentObjectiveRefinerTest extends RoutingFixtureTest {

  @Test
  void acceptsBoundedLengthOrderCandidateUsingActualInsertedCopper() {
    FRLogger.getLogEntries().clear();
    RoutingJob job = routeFixture(
        "router-intent-refiner-length-positive.dsn",
        lengthIntent());
    markSyntheticRouteCopperUnfixed(job, "BUS_A", "BUS_B", "BUS_C");
    applyRefinement(job);

    assertEquals(RoutingJobState.COMPLETED, job.state);
    assertRoutingResult(job, "router-intent-refiner-length-positive.dsn")
        .maxDuration(Duration.ofSeconds(15))
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    assertTrue(lengthSkewMm(job, "BUS_A", "BUS_B", "BUS_C") <= 5.0);
    assertTrue(logContains("Router-intent length refinement 'spi_bus' accepted"));
  }

  @Test
  void rejectsLengthCandidateWhenScopedCopperIsNotUnfixed() {
    FRLogger.getLogEntries().clear();
    RoutingJob job = routeFixture(
        "router-intent-refiner-length-fixed-negative.dsn",
        lengthIntent());

    assertEquals(RoutingJobState.COMPLETED, job.state);
    assertTrue(lengthSkewMm(job, "BUS_A", "BUS_B", "BUS_C") > 5.0);
    assertTrue(hasFixedRouteCopper(job, "BUS_B"));
    assertFalse(logContains("Router-intent length refinement 'spi_bus' accepted"));
  }

  @Test
  void pairAwareLocateAcceptsCoupledPairCandidate() {
    FRLogger.getLogEntries().clear();
    RoutingJob job = routeFixture(
        "router-intent-refiner-pair-positive.dsn",
        pairIntent());
    markSyntheticRouteCopperUnfixed(job, "USB_D+", "USB_D-");
    applyRefinement(job);

    assertEquals(RoutingJobState.COMPLETED, job.state);
    assertRoutingResult(job, "router-intent-refiner-pair-positive.dsn")
        .maxDuration(Duration.ofSeconds(15))
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    assertTrue(lengthSkewMm(job, "USB_D+", "USB_D-") <= 1.0);
    assertTrue(logContains("Router-intent pair refinement 'usb_pair' accepted coupled ratio"));
  }

  @Test
  void pairAwareLocateAcceptsUsbScaleGapCandidate() {
    FRLogger.getLogEntries().clear();
    RoutingJob job = routeFixture(
        "router-intent-refiner-pair-usb-gap-positive.dsn",
        pairIntent(0.2, 0.18, 0.12));
    markSyntheticRouteCopperUnfixed(job, "USB_D+", "USB_D-");
    applyRefinement(job);

    assertEquals(RoutingJobState.COMPLETED, job.state);
    assertRoutingResult(job, "router-intent-refiner-pair-usb-gap-positive.dsn")
        .maxDuration(Duration.ofSeconds(15))
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    assertTrue(lengthSkewMm(job, "USB_D+", "USB_D-") <= 1.0);
    assertTrue(logContains("Router-intent pair refinement 'usb_pair' accepted coupled ratio"));
  }

  @Test
  void schedulerRefinesOrdinaryRouteWithoutPreauthoredWiring() {
    FRLogger.getLogEntries().clear();
    RouterIntentSettings schedulerIntent = pairIntent(0.2, 0.18, 0.12);
    schedulerIntent.differentialPairs[0].maxSkewMm = 4.0;

    RoutingJob job = routeFixture(
        "router-intent-refiner-pair-scheduler-positive.dsn",
        schedulerIntent);

    assertEquals(RoutingJobState.COMPLETED, job.state);
    assertRoutingResult(job, "router-intent-refiner-pair-scheduler-positive.dsn")
        .maxDuration(Duration.ofSeconds(15))
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    assertTrue(
        logContains("Router-intent pair refinement 'usb_pair' accepted coupled ratio"),
        "expected RunRoutingJob to accept the objective refinement:\n"
            + String.join("\n", FRLogger.getLogEntries().get()));
  }

  @Test
  void schedulerReconcilesRetainedTraceSplitBySameNetBranchBeforeRefinement() {
    FRLogger.getLogEntries().clear();
    RouterIntentSettings schedulerIntent = pairIntent(0.2, 0.18, 0.12);
    schedulerIntent.differentialPairs[0].maxSkewMm = 4.0;

    RoutingJob job = routeFixture(
        "router-intent-refiner-pair-scheduler-source-split.dsn",
        schedulerIntent);

    assertEquals(RoutingJobState.COMPLETED, job.state);
    assertRoutingResult(job, "router-intent-refiner-pair-scheduler-source-split.dsn")
        .maxDuration(Duration.ofSeconds(15))
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    assertTrue(
        logContains("Router-intent pair refinement 'usb_pair' accepted coupled ratio"),
        "expected refinement after reconciling the retained trace split:\n"
            + String.join("\n", FRLogger.getLogEntries().get()));
    assertFalse(logContains("retained source copper changed during routing"));
    assertTrue(
        fixedTracePiecesCoveringPinSpan(job, "KEEP", "K1", "K2") >= 2,
        "expected the original shove-fixed K1-K2 trace to remain exactly covered by split descendants");
  }

  @Test
  void captureRetainedCopperSupportsFixedNetlessConductionArea() {
    RouterIntentSettings schedulerIntent = pairIntent(0.2, 0.18, 0.12);
    schedulerIntent.differentialPairs[0].maxSkewMm = 4.0;
    RoutingJob job = routeFixture(
        "router-intent-refiner-pair-scheduler-source-split.dsn",
        schedulerIntent);

    ConductionArea netlessArea = job.board.insert_conduction_area(
        new IntBox(new IntPoint(2_000, -65_000), new IntPoint(8_000, -60_000)),
        0,
        new int[0],
        1,
        false,
        FixedState.USER_FIXED);
    assertNotNull(netlessArea);
    assertTrue(hasFixedNetlessConductionArea(job));
    RouterIntentObjectiveRefiner.RetainedCopper withNetlessArea = assertDoesNotThrow(
        () -> RouterIntentObjectiveRefiner.captureRetainedCopper(job.board),
        "immutable retained-copper capture must support netless conduction areas");
    assertTrue(withNetlessArea.itemIds().contains(netlessArea.get_id_no()));
  }

  @Test
  void reconciliationRejectsViaReplacementWithDifferentPadstackGeometry() {
    RouterIntentSettings schedulerIntent = pairIntent(0.2, 0.18, 0.12);
    schedulerIntent.differentialPairs[0].maxSkewMm = 4.0;
    RoutingJob job = routeFixture(
        "router-intent-refiner-pair-scheduler-source-split.dsn",
        schedulerIntent);

    Padstack originalPadstack = job.board.library.padstacks.get("Via[0-1]_600:300_um");
    Padstack replacementPadstack = job.board.library.padstacks.get("Via[0-1]_800:400_um");
    assertNotNull(originalPadstack);
    assertNotNull(replacementPadstack);
    Via original = job.board.insert_via(
        originalPadstack,
        new IntPoint(12_000, -60_000),
        new int[] {netNo(job, "KEEP")},
        1,
        FixedState.USER_FIXED,
        false);
    RouterIntentObjectiveRefiner.RetainedCopper retained =
        RouterIntentObjectiveRefiner.captureRetainedCopper(job.board);
    job.board.remove_item(original);
    Via replacement = job.board.insert_via(
        replacementPadstack,
        original.get_center(),
        new int[] {netNo(job, "KEEP")},
        original.clearance_class_no(),
        original.get_fixed_state(),
        original.attach_allowed);
    assertNotEquals(original.get_id_no(), replacement.get_id_no());
    assertNotEquals(original.get_padstack().name, replacement.get_padstack().name);
    assertEquals(original.get_center(), replacement.get_center());
    assertEquals(original.first_layer(), replacement.first_layer());
    assertEquals(original.last_layer(), replacement.last_layer());
    assertEquals(original.get_fixed_state(), replacement.get_fixed_state());
    assertEquals(original.clearance_class_no(), replacement.clearance_class_no());
    assertEquals(original.get_net_no(0), replacement.get_net_no(0));
    assertEquals(original.attach_allowed, replacement.attach_allowed);

    FRLogger.getLogEntries().clear();
    RouterIntentObjectiveRefiner.Result result = RouterIntentObjectiveRefiner.refine(job, retained);

    assertFalse(result.accepted());
    assertTrue(
        logContains("retained source copper changed during routing"),
        "same-span via replacement with a different padstack must not preserve source identity");
  }

  @Test
  void reconciliationPreservesOneToOneMultiplicityForIdenticalNonTraceCopper() {
    for (boolean deleteFirst : List.of(true, false)) {
      RouterIntentSettings schedulerIntent = pairIntent(0.2, 0.18, 0.12);
      schedulerIntent.differentialPairs[0].maxSkewMm = 4.0;
      RoutingJob job = routeFixture(
          "router-intent-refiner-pair-scheduler-source-split.dsn",
          schedulerIntent);
      Padstack padstack = job.board.library.padstacks.get("Via[0-1]_600:300_um");
      assertNotNull(padstack);
      IntPoint center = new IntPoint(14_000, -62_000);
      Via first = job.board.insert_via(
          padstack,
          center,
          new int[] {netNo(job, "KEEP")},
          1,
          FixedState.USER_FIXED,
          false);
      Via second = job.board.insert_via(
          padstack,
          center,
          new int[] {netNo(job, "KEEP")},
          1,
          FixedState.USER_FIXED,
          false);
      assertNotEquals(first.get_id_no(), second.get_id_no());
      RouterIntentObjectiveRefiner.RetainedCopper retained =
          RouterIntentObjectiveRefiner.captureRetainedCopper(job.board);
      job.board.remove_item(deleteFirst ? first : second);

      FRLogger.getLogEntries().clear();
      RouterIntentObjectiveRefiner.Result result = RouterIntentObjectiveRefiner.refine(job, retained);

      assertFalse(result.accepted());
      assertTrue(
          logContains("retained source copper changed during routing"),
          "one remaining via must not satisfy two identical retained snapshots when deleting "
              + (deleteFirst ? "the first" : "the second") + " item");
    }
  }

  @Test
  void reconciliationSelectsContainedTraceCoverRegardlessOfSupersetItemIdOrder() {
    for (boolean exactInsertedFirst : List.of(true, false)) {
      RouterIntentSettings schedulerIntent = pairIntent(0.2, 0.18, 0.12);
      schedulerIntent.differentialPairs[0].maxSkewMm = 4.0;
      RoutingJob job = routeFixture(
          "router-intent-refiner-pair-scheduler-source-split.dsn",
          schedulerIntent);
      RouterIntentObjectiveRefiner.RetainedCopper retained =
          RouterIntentObjectiveRefiner.captureRetainedCopper(job.board);

      Set<PolylineTrace> fixedKeep = fixedTracesOnPinSpan(job, "KEEP", "K1", "K2");
      PolylineTrace sourcePiece = fixedKeep.stream().findFirst().orElse(null);
      assertTrue(fixedKeep.size() >= 2);
      assertNotNull(sourcePiece);
      assertTrue(job.board.remove_items(new ArrayList<Item>(fixedKeep)));

      Point start = pinPoint(job, "K1");
      IntPoint retainedEnd = (IntPoint) pinPoint(job, "K2");
      Point extendedEnd = new IntPoint(retainedEnd.x + 5_000, retainedEnd.y);
      PolylineTrace exact;
      PolylineTrace superset;
      if (exactInsertedFirst) {
        exact = insertFixedKeepTrace(job, sourcePiece, start, retainedEnd);
        superset = insertFixedKeepTrace(job, sourcePiece, start, extendedEnd);
      } else {
        superset = insertFixedKeepTrace(job, sourcePiece, start, extendedEnd);
        exact = insertFixedKeepTrace(job, sourcePiece, start, retainedEnd);
      }
      assertEquals(exactInsertedFirst, exact.get_id_no() < superset.get_id_no());

      FRLogger.getLogEntries().clear();
      RouterIntentObjectiveRefiner.refine(job, retained);

      assertFalse(
          logContains("retained source copper changed during routing"),
          "an exact contained cover must win over a superset candidate regardless of item ids");
      assertTrue(
          logContains("ordinary route is incomplete"),
          "the post-reconciliation incomplete check proves reconciliation itself succeeded");
    }
  }

  @Test
  void reconciliationRejectsReplacementTraceExtendingBeyondCapturedSourceUnion() {
    RouterIntentSettings schedulerIntent = pairIntent(0.2, 0.18, 0.12);
    schedulerIntent.differentialPairs[0].maxSkewMm = 4.0;
    RoutingJob job = routeFixture(
        "router-intent-refiner-pair-scheduler-source-split.dsn",
        schedulerIntent);
    RouterIntentObjectiveRefiner.RetainedCopper retained =
        RouterIntentObjectiveRefiner.captureRetainedCopper(job.board);

    Set<PolylineTrace> fixedKeep = fixedTracesOnPinSpan(job, "KEEP", "K1", "K2");
    PolylineTrace sourcePiece = fixedKeep.stream().findFirst().orElse(null);
    assertTrue(fixedKeep.size() >= 2, "fixture must split the captured source trace before this check");
    assertNotNull(sourcePiece);
    assertTrue(job.board.remove_items(new ArrayList<Item>(fixedKeep)));

    IntPoint start = (IntPoint) pinPoint(job, "K1");
    IntPoint end = (IntPoint) pinPoint(job, "K2");
    Point extendedEnd = new IntPoint(end.x + 50_000, end.y);
    job.board.insert_trace_without_cleaning(
        new Polyline(new Point[] {start, extendedEnd}),
        sourcePiece.get_layer(),
        sourcePiece.get_half_width(),
        new int[] {netNo(job, "KEEP")},
        sourcePiece.clearance_class_no(),
        sourcePiece.get_fixed_state());

    FRLogger.getLogEntries().clear();
    RouterIntentObjectiveRefiner.Result result = RouterIntentObjectiveRefiner.refine(job, retained);

    assertFalse(result.accepted());
    assertTrue(
        logContains("retained source copper changed during routing"),
        "a superset replacement must not be absorbed into retained source ownership");
  }

  @Test
  void schedulerPreservesGuideForDivergentEndpointPitchAndThreeMillimeterUncoupledCap() {
    FRLogger.getLogEntries().clear();
    RouterIntentSettings intent = pairIntent(0.2, 0.18, 0.12);
    intent.differentialPairs[0].maxSkewMm = 4.0;
    intent.differentialPairs[0].maxUncoupledLengthMm = 3.0;

    RoutingJob job = routeFixture(
        "router-intent-refiner-pair-divergent-pitch-scheduler-positive.dsn",
        intent);

    assertEquals(RoutingJobState.COMPLETED, job.state);
    assertRoutingResult(job, "router-intent-refiner-pair-divergent-pitch-scheduler-positive.dsn")
        .maxDuration(Duration.ofSeconds(15))
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    assertTrue(
        logContains("Router-intent pair refinement 'usb_pair' accepted coupled ratio"),
        "expected RunRoutingJob to preserve the pair guide and satisfy max_uncoupled_length_mm=3.0:\n"
            + String.join("\n", FRLogger.getLogEntries().get()));
  }

  @Test
  void schedulerRejectsPairCandidateThatWouldDegradePassingLengthObjective() {
    FRLogger.getLogEntries().clear();
    RouterIntentSettings intent = pairIntent(0.2, 0.18, 0.12);
    intent.differentialPairs[0].maxSkewMm = 4.0;
    RouterIntentSettings.RouteLengthMatchIntent lengthMatch = new RouterIntentSettings.RouteLengthMatchIntent();
    lengthMatch.id = "usb_length";
    lengthMatch.nets = new String[] {"USB_D+", "USB_D-"};
    lengthMatch.priority = RouterIntentSettings.Priority.CRITICAL;
    lengthMatch.maxSkewMm = 0.1;
    intent.routeLengthMatches = new RouterIntentSettings.RouteLengthMatchIntent[] {lengthMatch};

    RoutingJob job = routeFixture(
        "router-intent-refiner-pair-scheduler-positive.dsn",
        intent);

    assertEquals(RoutingJobState.COMPLETED, job.state);
    assertTrue(lengthSkewMm(job, "USB_D+", "USB_D-") <= 0.1);
    assertFalse(logContains("Router-intent pair refinement 'usb_pair' accepted coupled ratio"));
    assertFalse(logContains("Accepted 1 bounded router-intent objective refinement candidate"));
  }

  @Test
  void rejectsUnresolvableDeclaredEndpointWithoutChangingBoard() {
    FRLogger.getLogEntries().clear();
    RouterIntentSettings intent = pairIntent();
    intent.differentialPairs[0].negativeTo = "MISSING.1";
    RoutingJob job = routeFixture(
        "router-intent-refiner-pair-endpoint-negative.dsn",
        intent);
    markSyntheticRouteCopperUnfixed(job, "USB_D+", "USB_D-");
    String beforeHash = job.board.get_hash();
    RouterIntentObjectiveRefiner.Result result = RouterIntentObjectiveRefiner.refine(job);

    assertEquals(RoutingJobState.COMPLETED, job.state);
    assertFalse(result.accepted());
    assertEquals(beforeHash, result.board().get_hash());
    assertFalse(logContains("Router-intent pair refinement 'usb_pair' accepted coupled ratio"));
  }

  @Test
  void failedGuidedInsertRollsBackValidEndpointCandidate() {
    RoutingJob job = routeFixture(
        "router-intent-refiner-pair-usb-gap-positive.dsn",
        pairIntent(0.2, 100.0, 0.0));
    markSyntheticRouteCopperUnfixed(job, "USB_D+", "USB_D-");
    String beforeHash = job.board.get_hash();

    RouterIntentObjectiveRefiner.Result result = RouterIntentObjectiveRefiner.refine(job);

    assertFalse(result.accepted());
    assertEquals(beforeHash, result.board().get_hash());
  }

  @Test
  void acceptedCandidatePreservesUnrelatedUnfixedCopper() {
    FRLogger.getLogEntries().clear();
    RoutingJob job = routeFixture(
        "router-intent-refiner-pair-unrelated-copper.dsn",
        pairIntent(0.2, 0.18, 0.12));
    markSyntheticRouteCopperUnfixed(job, "USB_D+", "USB_D-", "KEEP");
    String beforeKeep = routeCopperSignature(job, "KEEP");

    applyRefinement(job);

    assertEquals(beforeKeep, routeCopperSignature(job, "KEEP"));
    assertTrue(logContains("Router-intent pair refinement 'usb_pair' accepted coupled ratio"));
  }

  @Test
  void pairAwareLocateIsDeterministicForIdenticalInputs() {
    RoutingJob first = routeFixture(
        "router-intent-refiner-pair-usb-gap-positive.dsn",
        pairIntent(0.2, 0.18, 0.12));
    RoutingJob second = routeFixture(
        "router-intent-refiner-pair-usb-gap-positive.dsn",
        pairIntent(0.2, 0.18, 0.12));
    markSyntheticRouteCopperUnfixed(first, "USB_D+", "USB_D-");
    markSyntheticRouteCopperUnfixed(second, "USB_D+", "USB_D-");

    RouterIntentObjectiveRefiner.Result firstResult = RouterIntentObjectiveRefiner.refine(first);
    RouterIntentObjectiveRefiner.Result secondResult = RouterIntentObjectiveRefiner.refine(second);

    assertTrue(firstResult.accepted());
    assertTrue(secondResult.accepted());
    assertEquals(firstResult.board().get_hash(), secondResult.board().get_hash());
  }

  private RoutingJob routeFixture(String fixture, RouterIntentSettings intent) {
    TestingSettings settings = new TestingSettings();
    settings.setFanoutEnabled(false);
    settings.setOptimizerEnabled(false);
    settings.setMaxPasses(3);
    settings.setJobTimeoutString("00:00:20");
    RoutingJob job = GetRoutingJob(fixture, settings);
    job.routerSettings.intent = intent;
    return RunRoutingJob(job);
  }

  private RouterIntentSettings lengthIntent() {
    RouterIntentSettings.RouteLengthMatchIntent match = new RouterIntentSettings.RouteLengthMatchIntent();
    match.id = "spi_bus";
    match.nets = new String[] {"BUS_A", "BUS_B", "BUS_C"};
    match.priority = RouterIntentSettings.Priority.CRITICAL;
    match.maxSkewMm = 5.0;
    RouterIntentSettings intent = new RouterIntentSettings();
    intent.routeLengthMatches = new RouterIntentSettings.RouteLengthMatchIntent[] {match};
    return intent;
  }

  private RouterIntentSettings pairIntent() {
    return pairIntent(0.5, 1.5, 0.2);
  }

  private RouterIntentSettings pairIntent(double widthMm, double gapMm, double gapToleranceMm) {
    RouterIntentSettings.DifferentialPairIntent pair = new RouterIntentSettings.DifferentialPairIntent();
    pair.id = "usb_pair";
    pair.positiveNet = "USB_D+";
    pair.negativeNet = "USB_D-";
    pair.positiveFrom = "P1.1";
    pair.positiveTo = "P2.1";
    pair.negativeFrom = "N1.1";
    pair.negativeTo = "N2.1";
    pair.priority = RouterIntentSettings.Priority.CRITICAL;
    pair.allowedLayers = new String[] {"F.Cu"};
    pair.sameLayerRequired = true;
    pair.maxViasPerNet = 0;
    pair.matchedViaTransitionsRequired = true;
    pair.routeAsCoupledPair = true;
    pair.targetWidthMm = widthMm;
    pair.targetGapMm = gapMm;
    pair.gapToleranceMm = gapToleranceMm;
    pair.maxSkewMm = 1.0;
    pair.minParallelLengthRatio = 0.8;
    pair.requireParallelEvidence = true;
    RouterIntentSettings intent = new RouterIntentSettings();
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] {pair};
    return intent;
  }

  private boolean logContains(String text) {
    return Arrays.stream(FRLogger.getLogEntries().get()).anyMatch(entry -> entry.contains(text));
  }

  private boolean hasFixedRouteCopper(RoutingJob job, String netName) {
    int netNo = netNo(job, netName);
    for (Item item : job.board.get_connectable_items(netNo)) {
      if (item instanceof Trace && item.get_fixed_state() != FixedState.UNFIXED) {
        return true;
      }
    }
    return false;
  }

  private boolean hasFixedNetlessConductionArea(RoutingJob job) {
    for (Item item : job.board.get_items()) {
      if (item instanceof ConductionArea
          && item.net_count() == 0
          && item.get_fixed_state() != FixedState.UNFIXED) {
        return true;
      }
    }
    return false;
  }

  private PolylineTrace insertFixedKeepTrace(
      RoutingJob job,
      PolylineTrace source,
      Point start,
      Point end) {
    PolylineTrace inserted = job.board.insert_trace_without_cleaning(
        new Polyline(new Point[] {start, end}),
        source.get_layer(),
        source.get_half_width(),
        new int[] {netNo(job, "KEEP")},
        source.clearance_class_no(),
        source.get_fixed_state());
    assertNotNull(inserted);
    return inserted;
  }

  private void markSyntheticRouteCopperUnfixed(RoutingJob job, String... netNames) {
    for (String netName : netNames) {
      for (Item item : job.board.get_connectable_items(netNo(job, netName))) {
        if (item instanceof Trace || item instanceof Via) {
          item.set_fixed_state(FixedState.UNFIXED);
        }
      }
    }
  }

  private void applyRefinement(RoutingJob job) {
    RouterIntentObjectiveRefiner.Result result = RouterIntentObjectiveRefiner.refine(job);
    assertTrue(
        result.accepted(),
        "expected the synthetic generated-copper candidate to be accepted:\n"
            + String.join("\n", FRLogger.getLogEntries().get()));
    job.board = result.board();
  }

  private double lengthSkewMm(RoutingJob job, String... netNames) {
    double shortest = Double.POSITIVE_INFINITY;
    double longest = 0.0;
    for (String netName : netNames) {
      double length = routedLengthMm(job, netName);
      shortest = Math.min(shortest, length);
      longest = Math.max(longest, length);
    }
    return longest - shortest;
  }

  private double routedLengthMm(RoutingJob job, String netName) {
    Net net = job.board.rules.nets.get(netName, 1);
    return net.get_trace_length() / job.board.communication.get_resolution(Unit.MM);
  }

  private String routeCopperSignature(RoutingJob job, String netName) {
    StringBuilder result = new StringBuilder();
    for (Item item : job.board.get_connectable_items(netNo(job, netName))) {
      if (item instanceof PolylineTrace trace) {
        result.append(trace.get_id_no())
            .append(':')
            .append(trace.get_fixed_state())
            .append(':')
            .append(trace.get_layer())
            .append(':')
            .append(trace.get_half_width())
            .append(':')
            .append(Arrays.toString(trace.polyline().corner_approx_arr()))
            .append('\n');
      } else if (item instanceof Via via) {
        result.append(via.get_id_no())
            .append(':')
            .append(via.get_fixed_state())
            .append(':')
            .append(via.first_layer())
            .append('-')
            .append(via.last_layer())
            .append(':')
            .append(via.get_center())
            .append('\n');
      }
    }
    return result.toString();
  }

  private int fixedTracePiecesCoveringPinSpan(
      RoutingJob job,
      String netName,
      String startRef,
      String endRef) {
    FloatPoint start = pinCenter(job, startRef);
    FloatPoint end = pinCenter(job, endRef);
    double minX = Math.min(start.x, end.x);
    double maxX = Math.max(start.x, end.x);
    List<double[]> intervals = new ArrayList<>();
    for (Item item : job.board.get_connectable_items(netNo(job, netName))) {
      if (!(item instanceof PolylineTrace trace)
          || trace.get_fixed_state() == FixedState.UNFIXED) {
        continue;
      }
      FloatPoint[] corners = trace.polyline().corner_approx_arr();
      for (int index = 0; index < corners.length - 1; index++) {
        FloatPoint first = corners[index];
        FloatPoint second = corners[index + 1];
        if (Math.abs(first.y - start.y) > 1e-6 || Math.abs(second.y - start.y) > 1e-6) {
          continue;
        }
        double intervalStart = Math.max(minX, Math.min(first.x, second.x));
        double intervalEnd = Math.min(maxX, Math.max(first.x, second.x));
        if (intervalEnd > intervalStart) {
          intervals.add(new double[] {intervalStart, intervalEnd});
        }
      }
    }
    intervals.sort(Comparator.comparingDouble(interval -> interval[0]));
    double coveredThrough = minX;
    int contributingPieces = 0;
    for (double[] interval : intervals) {
      if (interval[0] > coveredThrough + 1e-6) {
        return 0;
      }
      if (interval[1] <= coveredThrough + 1e-6) {
        continue;
      }
      coveredThrough = interval[1];
      contributingPieces++;
      if (coveredThrough >= maxX - 1e-6) {
        return contributingPieces;
      }
    }
    return 0;
  }

  private Set<PolylineTrace> fixedTracesOnPinSpan(
      RoutingJob job,
      String netName,
      String startRef,
      String endRef) {
    FloatPoint start = pinCenter(job, startRef);
    FloatPoint end = pinCenter(job, endRef);
    Set<PolylineTrace> result = new LinkedHashSet<>();
    for (Item item : job.board.get_connectable_items(netNo(job, netName))) {
      if (!(item instanceof PolylineTrace trace)
          || trace.get_fixed_state() == FixedState.UNFIXED) {
        continue;
      }
      FloatPoint[] corners = trace.polyline().corner_approx_arr();
      if (Arrays.stream(corners).allMatch(corner ->
          Math.abs(corner.y - start.y) <= 1e-6
              && corner.x >= Math.min(start.x, end.x) - 1e-6
              && corner.x <= Math.max(start.x, end.x) + 1e-6)) {
        result.add(trace);
      }
    }
    return result;
  }

  private FloatPoint pinCenter(RoutingJob job, String ref) {
    return pinPoint(job, ref).to_float();
  }

  private Point pinPoint(RoutingJob job, String ref) {
    Component component = job.board.components.get(ref);
    for (Pin pin : job.board.get_component_pins(component.no)) {
      if ("1".equals(pin.name())) {
        return pin.get_center();
      }
    }
    throw new AssertionError("missing fixture pin " + ref + ".1");
  }

  private int netNo(RoutingJob job, String netName) {
    Net net = job.board.rules.nets.get(netName, 1);
    return net.net_number;
  }
}
