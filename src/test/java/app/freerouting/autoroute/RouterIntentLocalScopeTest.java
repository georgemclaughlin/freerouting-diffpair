package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.board.Unit;
import app.freerouting.core.RoutingJob;
import app.freerouting.fixtures.RoutingFixtureTest;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import org.junit.jupiter.api.Test;

class RouterIntentLocalScopeTest extends RoutingFixtureTest {

  @Test
  void localSupportPadRefsCreateBoundedRoutePenaltyRegion() {
    RoutingJob job = GetRoutingJob("router-intent-local-scope.dsn");
    job.routerSettings.intent = localScopeIntent("LOCAL", 5.0, "L1.1", "L2.1");
    RunRoutingJob(job);
    Net net = job.board.rules.nets.get("LOCAL", 1);
    AutorouteControl control = new AutorouteControl(job.board, net.net_number, job.routerSettings);
    double coordinateScale = job.board.communication.resolution;

    double insidePenalty = control.routerIntentLocalScopePenalty(
        new FloatPoint(50000.0 * coordinateScale, -20000.0 * coordinateScale),
        0);
    double outsidePenalty = control.routerIntentLocalScopePenalty(
        new FloatPoint(50000.0 * coordinateScale, -32000.0 * coordinateScale),
        0);

    assertEquals(0.0, insidePenalty);
    assertTrue(outsidePenalty > job.board.communication.get_resolution(Unit.MM));
  }

  private RouterIntentSettings localScopeIntent(String netName, double maxDistanceMm, String... padRefs) {
    RouterIntentSettings.NetIntent netIntent = new RouterIntentSettings.NetIntent();
    netIntent.net = netName;
    netIntent.priority = RouterIntentSettings.Priority.NORMAL;
    netIntent.scope = RouterIntentSettings.Scope.LOCAL;
    netIntent.ripupProtection = RouterIntentSettings.RipupProtection.LOCAL_SUPPORT;

    RouterIntentSettings.LocalSupportIntent localSupport = new RouterIntentSettings.LocalSupportIntent();
    localSupport.id = "local_support_fixture";
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
}
