package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;

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
}
