package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
  private static final String STEERED_NET = "Net-(J1-Pin_1)";

  @Test
  void penalizesIncompleteIntentNetsWithoutChangingGenericScore() {
    RoutingJob job = routedRipupFixture();
    RouterIntentSettings intent = protectedNetIntent("KEEP");

    assertEquals(1, incompleteCount(job, "KEEP"));

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

  private RoutingJob routedRipupFixture() {
    TestingSettings settings = new TestingSettings();
    settings.setMaxPasses(20);
    settings.setJobTimeoutString("00:00:30");
    settings.setOptimizerEnabled(false);

    RoutingJob job = GetRoutingJob("router-intent-ripup-cost.dsn", settings);
    job.routerSettings.set_start_ripup_costs(1);
    RunRoutingJob(job);
    assertRoutingResult(job, "router-intent-ripup-cost.dsn")
        .maxDuration(Duration.ofSeconds(30))
        .maxIncompleteConnections(1)
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
}
