package app.freerouting.fixtures;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.board.PolylineTrace;
import app.freerouting.board.Trace;
import app.freerouting.board.Unit;
import app.freerouting.board.Via;
import app.freerouting.core.RoutingJob;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.sources.TestingSettings;
import java.time.Duration;
import org.junit.jupiter.api.Test;

class RouterIntentRoutingIntegrationTest extends RoutingFixtureTest {

  private static final String FIXTURE = "Issue269-min_fr_test/min_fr_test.dsn";
  private static final String STEERED_NET = "Net-(J1-Pin_1)";
  private static final String RIPUP_FIXTURE = "router-intent-ripup-cost.dsn";
  private static final String LOCAL_SCOPE_CORRIDOR_FIXTURE = "router-intent-local-scope-corridor.dsn";
  private static final String BLOCK_BOUNDARY_FIXTURE = "router-intent-design-block-boundary.dsn";
  private static final String LOCAL_NET = "LOCAL";
  private static final String BLOCK_NET = "BLOCK_NET";
  private static final double LOCAL_SCOPE_MARGIN_MM = 15.0;
  private static final double LOCAL_LEFT_PAD_X_UM = 10000.0;
  private static final double LOCAL_RIGHT_PAD_X_UM = 90000.0;
  private static final double LOCAL_PAD_Y_UM = -35000.0;
  private static final double BLOCK_BOUNDARY_CENTER_X_MM = 50.0;
  private static final double BLOCK_BOUNDARY_CENTER_Y_MM = -35.0;
  private static final double BLOCK_BOUNDARY_WIDTH_MM = 110.0;
  private static final double BLOCK_BOUNDARY_HEIGHT_MM = 30.0;

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

  @Test
  void localScopeIntentReducesRouteExcursion() {
    RouterIntentSettings localIntent = localScopeIntent(LOCAL_NET, LOCAL_SCOPE_MARGIN_MM, "L1.1", "L2.1");
    RoutingJob baseline = routeWithIntent(LOCAL_SCOPE_CORRIDOR_FIXTURE, null, LOCAL_NET, 0);
    RoutingJob scoped = routeWithIntent(LOCAL_SCOPE_CORRIDOR_FIXTURE, localIntent, LOCAL_NET, 0);

    double baselineExcursion = routedLocalScopeExcursionMm(baseline, LOCAL_NET);
    double scopedExcursion = routedLocalScopeExcursionMm(scoped, LOCAL_NET);

    assertTrue(
        scopedExcursion < baselineExcursion,
        "expected local-scope intent to choose lower-excursion geometry; baseline="
            + baselineExcursion
            + ", scoped="
            + scopedExcursion);
    assertTrue(
        routedLength(scoped, LOCAL_NET) > routedLength(baseline, LOCAL_NET),
        "expected lower-excursion local route to accept a longer in-region corridor");
  }

  @Test
  void blockPortBoundaryIntentReducesRouteExcursion() {
    RouterIntentSettings blockIntent = blockPortBoundaryIntent(BLOCK_NET);
    RoutingJob baseline = routeWithIntent(BLOCK_BOUNDARY_FIXTURE, null, BLOCK_NET, 0);
    RoutingJob bounded = routeWithIntent(BLOCK_BOUNDARY_FIXTURE, blockIntent, BLOCK_NET, 0);

    double baselineExcursion = routedBlockBoundaryExcursionMm(baseline, BLOCK_NET);
    double boundedExcursion = routedBlockBoundaryExcursionMm(bounded, BLOCK_NET);

    assertTrue(
        boundedExcursion < baselineExcursion,
        "expected design-block boundary intent to choose lower-excursion geometry; baseline="
            + baselineExcursion
            + ", bounded="
            + boundedExcursion);
    assertTrue(
        routedLength(bounded, BLOCK_NET) > routedLength(baseline, BLOCK_NET),
        "expected boundary-confined route to accept a longer in-block corridor");
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

  private RouterIntentSettings localScopeIntent(String netName, double maxDistanceMm, String... padRefs) {
    RouterIntentSettings.NetIntent netIntent = new RouterIntentSettings.NetIntent();
    netIntent.net = netName;
    netIntent.priority = RouterIntentSettings.Priority.NORMAL;
    netIntent.scope = RouterIntentSettings.Scope.LOCAL;
    netIntent.ripupProtection = RouterIntentSettings.RipupProtection.NONE;

    RouterIntentSettings.LocalSupportIntent localSupport = new RouterIntentSettings.LocalSupportIntent();
    localSupport.id = "local_scope_geometry_test";
    localSupport.kind = RouterIntentSettings.LocalSupportKind.SAME_NET_PAD_TIE;
    localSupport.nets = new String[] { netName };
    localSupport.padRefs = padRefs;
    localSupport.priority = RouterIntentSettings.Priority.NORMAL;
    localSupport.maxDistanceMm = maxDistanceMm;

    RouterIntentSettings intent = new RouterIntentSettings();
    intent.netIntents = new RouterIntentSettings.NetIntent[] { netIntent };
    intent.localSupport = new RouterIntentSettings.LocalSupportIntent[] { localSupport };
    return intent;
  }

  private RouterIntentSettings blockPortBoundaryIntent(String netName) {
    RouterIntentSettings.NetIntent netIntent = new RouterIntentSettings.NetIntent();
    netIntent.net = netName;
    netIntent.priority = RouterIntentSettings.Priority.NORMAL;
    netIntent.scope = RouterIntentSettings.Scope.NORMAL;
    netIntent.ripupProtection = RouterIntentSettings.RipupProtection.NONE;
    netIntent.blockPortIds = new String[] {
        "block_port_1_main_power_input",
        "block_port_2_main_power_output"
    };

    RouterIntentSettings.BlockPortIntent input = blockPortIntent(
        "block_port_1_main_power_input",
        netName,
        "input",
        "P_IN.1");
    RouterIntentSettings.BlockPortIntent output = blockPortIntent(
        "block_port_2_main_power_output",
        netName,
        "output",
        "P_OUT.1");

    RouterIntentSettings intent = new RouterIntentSettings();
    intent.netIntents = new RouterIntentSettings.NetIntent[] { netIntent };
    intent.blockPorts = new RouterIntentSettings.BlockPortIntent[] { input, output };
    return intent;
  }

  private RouterIntentSettings.BlockPortIntent blockPortIntent(
      String id,
      String netName,
      String portName,
      String padRef) {
    RouterIntentSettings.BlockPortIntent blockPort = new RouterIntentSettings.BlockPortIntent();
    blockPort.id = id;
    blockPort.block = "main_power";
    blockPort.port = portName;
    blockPort.kind = RouterIntentSettings.BlockPortKind.SIGNAL;
    blockPort.net = netName;
    blockPort.padRef = padRef;
    blockPort.boundaryName = "main_power_ownership";
    blockPort.boundaryCenterXMm = BLOCK_BOUNDARY_CENTER_X_MM;
    blockPort.boundaryCenterYMm = BLOCK_BOUNDARY_CENTER_Y_MM;
    blockPort.boundaryWidthMm = BLOCK_BOUNDARY_WIDTH_MM;
    blockPort.boundaryHeightMm = BLOCK_BOUNDARY_HEIGHT_MM;
    return blockPort;
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

  private double routedLocalScopeExcursionMm(RoutingJob job, String netName) {
    Net net = job.board.rules.nets.get(netName, 1);
    double scale = job.board.communication.resolution;
    double marginUm = LOCAL_SCOPE_MARGIN_MM * 1000.0;
    IntBox localRegion = new IntBox(
        (int) Math.floor((LOCAL_LEFT_PAD_X_UM - marginUm) * scale),
        (int) Math.floor((LOCAL_PAD_Y_UM - marginUm) * scale),
        (int) Math.ceil((LOCAL_RIGHT_PAD_X_UM + marginUm) * scale),
        (int) Math.ceil((LOCAL_PAD_Y_UM + marginUm) * scale));
    double mmResolution = job.board.communication.get_resolution(Unit.MM);

    double result = 0.0;
    for (Trace trace : job.board.get_traces()) {
      if (!trace.contains_net(net.net_number)) {
        continue;
      }
      FloatPoint[] points = trace instanceof PolylineTrace polylineTrace
          ? polylineTrace.polyline().corner_approx_arr()
          : new FloatPoint[] { trace.first_corner().to_float(), trace.last_corner().to_float() };
      for (FloatPoint point : points) {
        if (point.x >= localRegion.ll.x
            && point.x <= localRegion.ur.x
            && point.y >= localRegion.ll.y
            && point.y <= localRegion.ur.y) {
          continue;
        }
        result += point.distance(localRegion.nearest_point(point)) / mmResolution;
      }
    }
    return result;
  }

  private double routedBlockBoundaryExcursionMm(RoutingJob job, String netName) {
    Net net = job.board.rules.nets.get(netName, 1);
    double scale = job.board.communication.resolution;
    double minX = (BLOCK_BOUNDARY_CENTER_X_MM - BLOCK_BOUNDARY_WIDTH_MM / 2.0) * 1000.0;
    double maxX = (BLOCK_BOUNDARY_CENTER_X_MM + BLOCK_BOUNDARY_WIDTH_MM / 2.0) * 1000.0;
    double minY = (BLOCK_BOUNDARY_CENTER_Y_MM - BLOCK_BOUNDARY_HEIGHT_MM / 2.0) * 1000.0;
    double maxY = (BLOCK_BOUNDARY_CENTER_Y_MM + BLOCK_BOUNDARY_HEIGHT_MM / 2.0) * 1000.0;
    IntBox localRegion = new IntBox(
        (int) Math.floor(minX * scale),
        (int) Math.floor(minY * scale),
        (int) Math.ceil(maxX * scale),
        (int) Math.ceil(maxY * scale));
    double mmResolution = job.board.communication.get_resolution(Unit.MM);

    double result = 0.0;
    for (Trace trace : job.board.get_traces()) {
      if (!trace.contains_net(net.net_number)) {
        continue;
      }
      FloatPoint[] points = trace instanceof PolylineTrace polylineTrace
          ? polylineTrace.polyline().corner_approx_arr()
          : new FloatPoint[] { trace.first_corner().to_float(), trace.last_corner().to_float() };
      for (FloatPoint point : points) {
        if (point.x >= localRegion.ll.x
            && point.x <= localRegion.ur.x
            && point.y >= localRegion.ll.y
            && point.y <= localRegion.ur.y) {
          continue;
        }
        result += point.distance(localRegion.nearest_point(point)) / mmResolution;
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
