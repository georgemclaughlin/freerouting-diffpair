package app.freerouting.fixtures;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.board.Trace;
import app.freerouting.board.Via;
import app.freerouting.core.RoutingJob;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.sources.TestingSettings;
import java.time.Duration;
import org.junit.jupiter.api.Test;

class RouterIntentRoutingIntegrationTest extends RoutingFixtureTest {

  private static final String FIXTURE = "Issue269-min_fr_test/min_fr_test.dsn";
  private static final String STEERED_NET = "Net-(J1-Pin_1)";
  private static final String RIPUP_FIXTURE = "router-intent-ripup-cost.dsn";

  @Test
  void preferredLayerIntentChangesRoutedGeometry() {
    RoutingJob baseline = routeWithIntent(null);
    RoutingJob bottomPreferred = routeWithIntent(preferredLayerIntent(STEERED_NET, "B.Cu"));

    double baselineBottomLength = routedLengthOnLayer(baseline, STEERED_NET, "B.Cu");
    double intentBottomLength = routedLengthOnLayer(bottomPreferred, STEERED_NET, "B.Cu");

    assertTrue(
        intentBottomLength > baselineBottomLength,
        "expected B.Cu preferred-layer intent to increase routed B.Cu length for "
            + STEERED_NET
            + "; baseline="
            + baselineBottomLength
            + ", intent="
            + intentBottomLength);
  }

  @Test
  void protectedViaCostIntentChangesRoutedGeometry() {
    RoutingJob baseline = routeWithIntent(null);
    RoutingJob protectedNet = routeWithIntent(protectedNetIntent(STEERED_NET));

    int baselineViaCount = viaCountOnNet(baseline, STEERED_NET);
    int protectedViaCount = viaCountOnNet(protectedNet, STEERED_NET);
    double baselineLength = routedLength(baseline, STEERED_NET);
    double protectedLength = routedLength(protectedNet, STEERED_NET);

    assertTrue(
        protectedViaCount < baselineViaCount,
        "expected source-copper via-cost intent to reduce routed vias for "
            + STEERED_NET
            + "; baseline="
            + baselineViaCount
            + ", intent="
            + protectedViaCount);
    assertTrue(
        protectedLength > baselineLength,
        "expected via-cost intent to accept a longer no-via route for "
            + STEERED_NET
            + "; baseline="
            + baselineLength
            + ", intent="
            + protectedLength);
  }

  @Test
  void protectedRipupCostIntentPreservesObstacleGeometry() {
    RoutingJob baseline = routeWithIntent(RIPUP_FIXTURE, null, "CROSS", 1, 1);
    RoutingJob protectedKeep = routeWithIntent(RIPUP_FIXTURE, protectedNetIntent("KEEP"), "CROSS", 0, 1);

    assertEquals(1, incompleteCount(baseline, "KEEP"));
    assertEquals(0, incompleteCount(protectedKeep, "KEEP"));
    assertTrue(
        routedLength(protectedKeep, "KEEP") > routedLength(baseline, "KEEP"),
        "expected protected ripup intent to preserve the existing KEEP trace");
    assertTrue(
        routedLength(protectedKeep, "CROSS") > routedLength(baseline, "CROSS"),
        "expected protected ripup intent to route CROSS around KEEP instead of ripping through it");
  }

  private RoutingJob routeWithIntent(RouterIntentSettings intent) {
    return routeWithIntent(FIXTURE, intent, STEERED_NET, 1);
  }

  private RoutingJob routeWithIntent(String fixture, RouterIntentSettings intent, String completeNetName, int maxIncomplete) {
    return routeWithIntent(fixture, intent, completeNetName, maxIncomplete, null);
  }

  private RoutingJob routeWithIntent(
      String fixture,
      RouterIntentSettings intent,
      String completeNetName,
      int maxIncomplete,
      Integer startRipupCosts) {
    TestingSettings settings = new TestingSettings();
    settings.setMaxPasses(20);
    settings.setJobTimeoutString("00:00:30");
    settings.setOptimizerEnabled(false);

    RoutingJob job = GetRoutingJob(fixture, settings);
    if (startRipupCosts != null) {
      job.routerSettings.set_start_ripup_costs(startRipupCosts);
    }
    job.routerSettings.intent = intent;
    RunRoutingJob(job);
    assertRoutingResult(job, fixture)
        .maxDuration(Duration.ofSeconds(30))
        .maxIncompleteConnections(maxIncomplete)
        .check();
    assertEquals(0, incompleteCount(job, completeNetName));
    return job;
  }

  private RouterIntentSettings preferredLayerIntent(String netName, String layerName) {
    RouterIntentSettings.NetIntent netIntent = new RouterIntentSettings.NetIntent();
    netIntent.net = netName;
    netIntent.priority = RouterIntentSettings.Priority.CRITICAL;
    netIntent.scope = RouterIntentSettings.Scope.GLOBAL;
    netIntent.ripupProtection = RouterIntentSettings.RipupProtection.NONE;
    netIntent.preferredLayers = new String[] { layerName };

    RouterIntentSettings intent = new RouterIntentSettings();
    intent.netIntents = new RouterIntentSettings.NetIntent[] { netIntent };
    return intent;
  }

  private RouterIntentSettings protectedNetIntent(String netName) {
    RouterIntentSettings.NetIntent netIntent = new RouterIntentSettings.NetIntent();
    netIntent.net = netName;
    netIntent.priority = RouterIntentSettings.Priority.CRITICAL;
    netIntent.scope = RouterIntentSettings.Scope.GLOBAL;
    netIntent.ripupProtection = RouterIntentSettings.RipupProtection.SOURCE_COPPER;

    RouterIntentSettings intent = new RouterIntentSettings();
    intent.netIntents = new RouterIntentSettings.NetIntent[] { netIntent };
    return intent;
  }

  private double routedLengthOnLayer(RoutingJob job, String netName, String layerName) {
    Net net = job.board.rules.nets.get(netName, 1);
    int layer = layerIndex(job, layerName);
    double result = 0.0;
    for (Trace trace : job.board.get_traces()) {
      if (trace.contains_net(net.net_number) && trace.get_layer() == layer) {
        result += trace.get_length();
      }
    }
    return result;
  }

  private double routedLength(RoutingJob job, String netName) {
    Net net = job.board.rules.nets.get(netName, 1);
    double result = 0.0;
    for (Trace trace : job.board.get_traces()) {
      if (trace.contains_net(net.net_number)) {
        result += trace.get_length();
      }
    }
    return result;
  }

  private int viaCountOnNet(RoutingJob job, String netName) {
    Net net = job.board.rules.nets.get(netName, 1);
    int result = 0;
    for (Via via : job.board.get_vias()) {
      if (via.contains_net(net.net_number)) {
        result++;
      }
    }
    return result;
  }

  private int layerIndex(RoutingJob job, String layerName) {
    for (int i = 0; i < job.board.layer_structure.arr.length; i++) {
      if (layerName.equals(job.board.layer_structure.arr[i].name)) {
        return i;
      }
    }
    throw new IllegalArgumentException("unknown layer: " + layerName);
  }

  private int incompleteCount(RoutingJob job, String netName) {
    Net net = job.board.rules.nets.get(netName, 1);
    return new DesignRulesChecker(job.board, null).getIncompleteCount(net.net_number);
  }
}
