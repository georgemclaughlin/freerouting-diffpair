package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.board.Unit;
import app.freerouting.board.Via;
import app.freerouting.core.RoutingJob;
import app.freerouting.core.scoring.BoardStatistics;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.fixtures.RoutingFixtureTest;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.sources.TestingSettings;
import java.time.Duration;
import org.junit.jupiter.api.Test;

class RouterIntentBoardScorerTest extends RoutingFixtureTest {
  private static final String VIA_FIXTURE = "Issue269-min_fr_test/min_fr_test.dsn";
  private static final String RIPUP_FIXTURE = "router-intent-ripup-cost.dsn";
  private static final String LOCAL_SCOPE_FIXTURE = "router-intent-local-scope.dsn";
  private static final String STEERED_NET = "Net-(J1-Pin_1)";
  private static final String KEEP_NET = "KEEP";
  private static final String CROSS_NET = "CROSS";
  private static final String LOCAL_NET = "LOCAL";

  @Test
  void penalizesIncompleteIntentNetsWithoutChangingGenericScore() {
    RoutingJob job = routedRipupFixture();
    RouterIntentSettings intent = protectedNetIntent(KEEP_NET);

    assertEquals(1, incompleteCount(job, KEEP_NET));

    float genericScore = new BoardStatistics(job.board).getNormalizedScore(job.routerSettings.scoring);
    float noIntentScore = RouterIntentBoardScorer.normalizedScore(job.board, job.routerSettings.scoring, null);
    float intentPenalty = RouterIntentBoardScorer.intentIncompletePenalty(job.board, intent);
    float intentScore = RouterIntentBoardScorer.normalizedScore(job.board, job.routerSettings.scoring, intent);

    assertEquals(genericScore, noIntentScore, 0.001f);
    assertEquals(175f, intentPenalty, 0.001f);
    assertEquals(Math.max(0f, genericScore - intentPenalty), intentScore, 0.001f);
  }

  @Test
  void penalizesViasOnProtectedIntentNets() {
    RouterIntentSettings intent = protectedNetIntent(STEERED_NET);
    RoutingJob baseline = routedViaFixture(null);
    RoutingJob protectedRoute = routedViaFixture(intent);

    int baselineViaCount = viaCountOnNet(baseline, STEERED_NET);
    int protectedViaCount = viaCountOnNet(protectedRoute, STEERED_NET);
    assertTrue(
        baselineViaCount > protectedViaCount,
        "fixture should preserve the via tradeoff used by the router-intent integration test");

    float baselinePenalty = RouterIntentBoardScorer.protectedViaPenalty(baseline.board, intent);
    float protectedPenalty = RouterIntentBoardScorer.protectedViaPenalty(protectedRoute.board, intent);
    float baselineGenericScore = new BoardStatistics(baseline.board).getNormalizedScore(baseline.routerSettings.scoring);
    float baselineIntentScore = RouterIntentBoardScorer.normalizedScore(
        baseline.board,
        baseline.routerSettings.scoring,
        intent);

    assertEquals(baselineViaCount * 60f, baselinePenalty, 0.001f);
    assertEquals(protectedViaCount * 60f, protectedPenalty, 0.001f);
    assertEquals(Math.max(0f, baselineGenericScore - baselinePenalty), baselineIntentScore, 0.001f);
  }

  @Test
  void penalizesCriticalPathExcessLengthWithoutNetIntent() {
    RoutingJob routed = routedViaFixture(null);
    double routedLengthMm = traceLengthMm(routed, STEERED_NET);
    double maxLengthMm = routedLengthMm - 0.5;
    RouterIntentSettings intent = criticalPathIntent(STEERED_NET, maxLengthMm);

    float genericScore = new BoardStatistics(routed.board).getNormalizedScore(routed.routerSettings.scoring);
    float lengthPenalty = RouterIntentBoardScorer.criticalPathExcessLengthPenalty(routed.board, intent);
    float intentScore = RouterIntentBoardScorer.normalizedScore(
        routed.board,
        routed.routerSettings.scoring,
        intent);

    assertTrue(routedLengthMm > 0.5, "fixture should route enough copper for a bounded excess-length check");
    assertEquals(15f, lengthPenalty, 0.001f);
    assertEquals(Math.max(0f, genericScore - lengthPenalty), intentScore, 0.001f);
  }

  @Test
  void penalizesDifferentialPairSkewWithoutNetIntent() {
    RoutingJob routed = routedProtectedRipupFixture();
    double keepLengthMm = traceLengthMm(routed, KEEP_NET);
    double crossLengthMm = traceLengthMm(routed, CROSS_NET);
    double skewMm = Math.abs(keepLengthMm - crossLengthMm);
    RouterIntentSettings intent = differentialPairIntent(KEEP_NET, CROSS_NET, skewMm - 0.5);

    float genericScore = new BoardStatistics(routed.board).getNormalizedScore(routed.routerSettings.scoring);
    float skewPenalty = RouterIntentBoardScorer.differentialPairSkewPenalty(routed.board, intent);
    float intentScore = RouterIntentBoardScorer.normalizedScore(
        routed.board,
        routed.routerSettings.scoring,
        intent);

    assertEquals(0, incompleteCount(routed, KEEP_NET));
    assertEquals(0, incompleteCount(routed, CROSS_NET));
    assertTrue(skewMm > 0.5, "fixture should route the two nets with measurable length skew");
    assertEquals(30f, skewPenalty, 0.001f);
    assertEquals(Math.max(0f, genericScore - skewPenalty), intentScore, 0.001f);
  }

  @Test
  void penalizesLocalScopeTraceExcursion() {
    RoutingJob routed = routedLocalScopeFixture();
    RouterIntentSettings strictLocalIntent = localScopeIntent(LOCAL_NET, 0.1, "L1.1");

    float genericScore = new BoardStatistics(routed.board).getNormalizedScore(routed.routerSettings.scoring);
    float localPenalty = RouterIntentBoardScorer.localScopeExcursionPenalty(routed.board, strictLocalIntent);
    float intentScore = RouterIntentBoardScorer.normalizedScore(
        routed.board,
        routed.routerSettings.scoring,
        strictLocalIntent);

    assertEquals(0, incompleteCount(routed, LOCAL_NET));
    assertTrue(localPenalty > 0f, "strict one-pad local region should penalize routed excursion");
    assertEquals(Math.max(0f, genericScore - localPenalty), intentScore, 0.001f);
  }

  private RoutingJob routedRipupFixture() {
    TestingSettings settings = new TestingSettings();
    settings.setMaxPasses(20);
    settings.setJobTimeoutString("00:00:30");
    settings.setOptimizerEnabled(false);

    RoutingJob job = GetRoutingJob(RIPUP_FIXTURE, settings);
    job.routerSettings.set_start_ripup_costs(1);
    RunRoutingJob(job);
    assertRoutingResult(job, RIPUP_FIXTURE)
        .maxDuration(Duration.ofSeconds(30))
        .maxIncompleteConnections(1)
        .check();
    return job;
  }

  private RoutingJob routedProtectedRipupFixture() {
    TestingSettings settings = new TestingSettings();
    settings.setMaxPasses(20);
    settings.setJobTimeoutString("00:00:30");
    settings.setOptimizerEnabled(false);

    RoutingJob job = GetRoutingJob(RIPUP_FIXTURE, settings);
    job.routerSettings.intent = protectedNetIntent(KEEP_NET);
    job.routerSettings.set_start_ripup_costs(1);
    RunRoutingJob(job);
    assertRoutingResult(job, RIPUP_FIXTURE)
        .maxDuration(Duration.ofSeconds(30))
        .maxIncompleteConnections(0)
        .check();
    return job;
  }

  private RoutingJob routedViaFixture(RouterIntentSettings intent) {
    TestingSettings settings = new TestingSettings();
    settings.setMaxPasses(20);
    settings.setJobTimeoutString("00:00:30");
    settings.setOptimizerEnabled(false);

    RoutingJob job = GetRoutingJob(VIA_FIXTURE, settings);
    job.routerSettings.intent = intent;
    RunRoutingJob(job);
    assertRoutingResult(job, VIA_FIXTURE)
        .maxDuration(Duration.ofSeconds(30))
        .maxIncompleteConnections(1)
        .check();
    assertEquals(0, incompleteCount(job, STEERED_NET));
    return job;
  }

  private RoutingJob routedLocalScopeFixture() {
    TestingSettings settings = new TestingSettings();
    settings.setMaxPasses(20);
    settings.setJobTimeoutString("00:00:30");
    settings.setOptimizerEnabled(false);

    RoutingJob job = GetRoutingJob(LOCAL_SCOPE_FIXTURE, settings);
    job.routerSettings.intent = localScopeIntent(LOCAL_NET, 5.0, "L1.1", "L2.1");
    RunRoutingJob(job);
    assertRoutingResult(job, LOCAL_SCOPE_FIXTURE)
        .maxDuration(Duration.ofSeconds(30))
        .maxIncompleteConnections(0)
        .check();
    return job;
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

  private RouterIntentSettings criticalPathIntent(String netName, double maxLengthMm) {
    RouterIntentSettings.CriticalPathIntent criticalPath = new RouterIntentSettings.CriticalPathIntent();
    criticalPath.id = "critical_path_length_test";
    criticalPath.net = netName;
    criticalPath.priority = RouterIntentSettings.Priority.CRITICAL;
    criticalPath.maxLengthMm = maxLengthMm;

    RouterIntentSettings intent = new RouterIntentSettings();
    intent.criticalPaths = new RouterIntentSettings.CriticalPathIntent[] { criticalPath };
    return intent;
  }

  private RouterIntentSettings differentialPairIntent(String positiveNet, String negativeNet, double maxSkewMm) {
    RouterIntentSettings.DifferentialPairIntent differentialPair = new RouterIntentSettings.DifferentialPairIntent();
    differentialPair.id = "differential_pair_skew_test";
    differentialPair.positiveNet = positiveNet;
    differentialPair.negativeNet = negativeNet;
    differentialPair.priority = RouterIntentSettings.Priority.CRITICAL;
    differentialPair.maxSkewMm = maxSkewMm;

    RouterIntentSettings intent = new RouterIntentSettings();
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] { differentialPair };
    return intent;
  }

  private RouterIntentSettings localScopeIntent(String netName, double maxDistanceMm, String... padRefs) {
    RouterIntentSettings.NetIntent netIntent = new RouterIntentSettings.NetIntent();
    netIntent.net = netName;
    netIntent.priority = RouterIntentSettings.Priority.NORMAL;
    netIntent.scope = RouterIntentSettings.Scope.LOCAL;
    netIntent.ripupProtection = RouterIntentSettings.RipupProtection.NONE;

    RouterIntentSettings.LocalSupportIntent localSupport = new RouterIntentSettings.LocalSupportIntent();
    localSupport.id = "local_scope_scoring_test";
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

  private int incompleteCount(RoutingJob job, String netName) {
    Net net = job.board.rules.nets.get(netName, 1);
    return new DesignRulesChecker(job.board, null).getIncompleteCount(net.net_number);
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

  private double traceLengthMm(RoutingJob job, String netName) {
    Net net = job.board.rules.nets.get(netName, 1);
    return net.get_trace_length() / job.board.communication.get_resolution(Unit.MM);
  }
}
