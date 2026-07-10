package app.freerouting.fixtures;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.board.Item;
import app.freerouting.board.Trace;
import app.freerouting.board.Unit;
import app.freerouting.board.Via;
import app.freerouting.core.RoutingJob;
import app.freerouting.core.RoutingJobState;
import app.freerouting.logger.FRLogger;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.sources.TestingSettings;
import app.freerouting.util.gson.GsonProvider;
import com.google.gson.JsonParser;
import java.time.Duration;
import java.util.Arrays;
import org.junit.jupiter.api.Test;

class RouterLadderRoutingTest extends RoutingFixtureTest {

  @Test
  void straightTwoPinNetRoutesWithoutViasOrDetour() {
    RoutingJob job = routeFixture("router-ladder-straight-single.dsn");

    assertEquals(RoutingJobState.COMPLETED, job.state);
    assertEquals(0, job.incompleteConnectionCount);
    assertRoutingResult(job, "router-ladder-straight-single.dsn")
        .maxDuration(Duration.ofSeconds(10))
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    assertEquals(0, viaCountOnNet(job, "NET_A"));
    assertTrue(
        routedLengthMm(job, "NET_A") <= 82.0,
        "straight NET_A should stay close to the 80 mm pad span, actual=" + routedLengthMm(job, "NET_A"));
  }

  @Test
  void straightParallelDifferentialPairRoutesWithLowSkew() {
    RoutingJob job = routeFixture("router-ladder-straight-diff-pair.dsn");

    assertRoutingResult(job, "router-ladder-straight-diff-pair.dsn")
        .maxDuration(Duration.ofSeconds(10))
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    assertEquals(0, viaCountOnNet(job, "USB_D+"));
    assertEquals(0, viaCountOnNet(job, "USB_D-"));

    double positiveLength = routedLengthMm(job, "USB_D+");
    double negativeLength = routedLengthMm(job, "USB_D-");
    assertTrue(
        Math.abs(positiveLength - negativeLength) <= 0.25,
        "straight pair skew should stay <= 0.25 mm, positive="
            + positiveLength
            + ", negative="
            + negativeLength);
  }

  @Test
  void hardBlockedTwoPinNetRemainsUnroutedWithoutViolations() {
    RoutingJob job = routeFixture("router-ladder-hard-blocked-single.dsn");

    assertEquals(RoutingJobState.INCOMPLETE, job.state);
    assertEquals(1, job.incompleteConnectionCount);
    assertNotNull(job.output);
    assertNotNull(job.output.getData());
    var jobReport = JsonParser.parseString(GsonProvider.GSON.toJson(job)).getAsJsonObject();
    assertEquals("INCOMPLETE", jobReport.get("state").getAsString());
    assertEquals(1, jobReport.get("incomplete_connection_count").getAsInt());
    assertRoutingResult(job, "router-ladder-hard-blocked-single.dsn")
        .maxDuration(Duration.ofSeconds(10))
        .exactIncompleteConnections(1)
        .exactClearanceViolations(0)
        .check();
    assertEquals(0.0, routedLengthMm(job, "NET_A"), 0.001);
  }

  @Test
  void offsetTwoPinNetRoutesWithBentPathWithoutVias() {
    RoutingJob job = routeFixture("router-ladder-offset-single.dsn");

    assertRoutingResult(job, "router-ladder-offset-single.dsn")
        .maxDuration(Duration.ofSeconds(10))
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();

    double routedLength = routedLengthMm(job, "NET_A");
    assertEquals(0, viaCountOnNet(job, "NET_A"));
    assertTrue(
        routedLength > 80.0 && routedLength < 100.0,
        "offset NET_A should route a bounded bent path, length=" + routedLength);
  }

  @Test
  void frontPadNetUsesViasWhenFrontLayerIsBlocked() {
    RoutingJob job = routeFixture("router-ladder-via-required-single.dsn");

    assertRoutingResult(job, "router-ladder-via-required-single.dsn")
        .maxDuration(Duration.ofSeconds(10))
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();
    assertTrue(viaCountOnNet(job, "NET_A") > 0, "NET_A should require vias to escape the F.Cu keepout.");
    assertTrue(
        routedLengthOnLayerMm(job, "NET_A", "B.Cu") > 0.0,
        "NET_A should route part of the connection on B.Cu.");
  }

  @Test
  void routeOrderRankControlsBatchQueueOrder() {
    FRLogger.getLogEntries().clear();

    RoutingJob job = routeFixture("router-ladder-route-order.dsn", routeOrderIntent());

    assertRoutingResult(job, "router-ladder-route-order.dsn")
        .maxDuration(Duration.ofSeconds(10))
        .exactIncompleteConnections(0)
        .exactClearanceViolations(0)
        .check();

    String routeOrderLog = Arrays.stream(FRLogger.getLogEntries().get())
        .filter(entry -> entry.contains("Router intent route order:"))
        .findFirst()
        .orElseThrow(() -> new AssertionError("missing Router intent route order log entry"));
    assertTrue(
        routeOrderLog.indexOf("HIGH(route_order_rank=300)")
            < routeOrderLog.indexOf("MEDIUM(route_order_rank=200)"),
        routeOrderLog);
    assertTrue(
        routeOrderLog.indexOf("MEDIUM(route_order_rank=200)")
            < routeOrderLog.indexOf("LOW(route_order_rank=100)"),
        routeOrderLog);
  }

  private RoutingJob routeFixture(String filename) {
    return routeFixture(filename, null);
  }

  private RoutingJob routeFixture(String filename, RouterIntentSettings intent) {
    TestingSettings settings = new TestingSettings();
    settings.setFanoutEnabled(false);
    settings.setOptimizerEnabled(false);
    settings.setMaxPasses(8);
    settings.setJobTimeoutString("00:00:20");

    RoutingJob job = GetRoutingJob(filename, settings);
    job.routerSettings.intent = intent;
    return RunRoutingJob(job);
  }

  private int viaCountOnNet(RoutingJob job, String netName) {
    int netNo = netNo(job, netName);
    int count = 0;
    for (Item item : job.board.get_connectable_items(netNo)) {
      if (item instanceof Via) {
        count++;
      }
    }
    return count;
  }

  private double routedLengthMm(RoutingJob job, String netName) {
    int netNo = netNo(job, netName);
    double length = 0.0;
    for (Item item : job.board.get_connectable_items(netNo)) {
      if (item instanceof Trace trace) {
        length += trace.get_length();
      }
    }
    return length / job.board.communication.get_resolution(Unit.MM);
  }

  private double routedLengthOnLayerMm(RoutingJob job, String netName, String layerName) {
    int netNo = netNo(job, netName);
    int layerNo = layerNo(job, layerName);
    double length = 0.0;
    for (Item item : job.board.get_connectable_items(netNo)) {
      if (item instanceof Trace trace && trace.get_layer() == layerNo) {
        length += trace.get_length();
      }
    }
    return length / job.board.communication.get_resolution(Unit.MM);
  }

  private int layerNo(RoutingJob job, String layerName) {
    for (int i = 0; i < job.board.layer_structure.arr.length; i++) {
      if (layerName.equals(job.board.layer_structure.arr[i].name)) {
        return i;
      }
    }
    throw new IllegalArgumentException("missing layer " + layerName);
  }

  private int netNo(RoutingJob job, String netName) {
    Net net = job.board.rules.nets.get(netName, 1);
    if (net == null) {
      throw new IllegalArgumentException("missing net " + netName);
    }
    return net.net_number;
  }

  private RouterIntentSettings routeOrderIntent() {
    RouterIntentSettings intent = new RouterIntentSettings();
    intent.netIntents = new RouterIntentSettings.NetIntent[] {
        netIntent("LOW", 100),
        netIntent("HIGH", 300),
        netIntent("MEDIUM", 200)
    };
    return intent;
  }

  private RouterIntentSettings.NetIntent netIntent(String netName, int routeOrderRank) {
    RouterIntentSettings.NetIntent intent = new RouterIntentSettings.NetIntent();
    intent.net = netName;
    intent.routeOrderRank = routeOrderRank;
    return intent;
  }
}
