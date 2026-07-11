package app.freerouting.fixtures;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.board.FixedState;
import app.freerouting.board.Item;
import app.freerouting.board.PolylineTrace;
import app.freerouting.board.Trace;
import app.freerouting.board.Unit;
import app.freerouting.board.Via;
import app.freerouting.autoroute.RouterIntentObjectiveRefiner;
import app.freerouting.core.RoutingJob;
import app.freerouting.core.RoutingJobState;
import app.freerouting.logger.FRLogger;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.sources.TestingSettings;
import java.time.Duration;
import java.util.Arrays;
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

  private int netNo(RoutingJob job, String netName) {
    Net net = job.board.rules.nets.get(netName, 1);
    return net.net_number;
  }
}
