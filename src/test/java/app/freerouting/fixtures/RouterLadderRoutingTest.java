package app.freerouting.fixtures;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.board.Item;
import app.freerouting.board.Trace;
import app.freerouting.board.Unit;
import app.freerouting.board.Via;
import app.freerouting.core.RoutingJob;
import app.freerouting.rules.Net;
import app.freerouting.settings.sources.TestingSettings;
import java.time.Duration;
import org.junit.jupiter.api.Test;

class RouterLadderRoutingTest extends RoutingFixtureTest {

  @Test
  void straightTwoPinNetRoutesWithoutViasOrDetour() {
    RoutingJob job = routeFixture("router-ladder-straight-single.dsn");

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

    assertRoutingResult(job, "router-ladder-hard-blocked-single.dsn")
        .maxDuration(Duration.ofSeconds(10))
        .exactIncompleteConnections(1)
        .exactClearanceViolations(0)
        .check();
    assertEquals(0.0, routedLengthMm(job, "NET_A"), 0.001);
  }

  private RoutingJob routeFixture(String filename) {
    TestingSettings settings = new TestingSettings();
    settings.setFanoutEnabled(false);
    settings.setOptimizerEnabled(false);
    settings.setMaxPasses(8);
    settings.setJobTimeoutString("00:00:20");

    RoutingJob job = GetRoutingJob(filename, settings);
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

  private int netNo(RoutingJob job, String netName) {
    Net net = job.board.rules.nets.get(netName, 1);
    if (net == null) {
      throw new IllegalArgumentException("missing net " + netName);
    }
    return net.net_number;
  }
}
