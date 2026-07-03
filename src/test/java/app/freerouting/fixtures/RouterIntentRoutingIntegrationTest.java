package app.freerouting.fixtures;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.board.Trace;
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

  private RoutingJob routeWithIntent(RouterIntentSettings intent) {
    TestingSettings settings = new TestingSettings();
    settings.setMaxPasses(20);
    settings.setJobTimeoutString("00:00:30");
    settings.setOptimizerEnabled(false);

    RoutingJob job = GetRoutingJob(FIXTURE, settings);
    job.routerSettings.intent = intent;
    RunRoutingJob(job);
    assertRoutingResult(job, FIXTURE)
        .maxDuration(Duration.ofSeconds(30))
        .maxIncompleteConnections(1)
        .check();
    assertEquals(0, incompleteCount(job, STEERED_NET));
    return job;
  }

  private RouterIntentSettings preferredLayerIntent(String netName, String layerName) {
    RouterIntentSettings.NetIntent netIntent = new RouterIntentSettings.NetIntent();
    netIntent.net = netName;
    netIntent.priority = RouterIntentSettings.Priority.CRITICAL;
    netIntent.scope = RouterIntentSettings.Scope.GLOBAL;
    netIntent.ripupProtection = RouterIntentSettings.RipupProtection.CRITICAL;
    netIntent.preferredLayers = new String[] { layerName };

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
