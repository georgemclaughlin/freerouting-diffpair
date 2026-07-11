package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.core.RoutingJob;
import app.freerouting.fixtures.RoutingFixtureTest;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.sources.TestingSettings;
import org.junit.jupiter.api.Test;

class RouterIntentPairMetricsTest extends RoutingFixtureTest {

  @Test
  void acceptanceMetricsCountOnlyCopperInsideDeclaredGapBand() {
    RouterIntentSettings.DifferentialPairIntent pair = pairIntent();
    RouterIntentSettings intent = new RouterIntentSettings();
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] {pair};
    TestingSettings settings = new TestingSettings();
    settings.setFanoutEnabled(false);
    settings.setOptimizerEnabled(false);
    settings.setMaxPasses(1);
    RoutingJob job = GetRoutingJob("router-intent-pair-gap-band-metrics.dsn", settings);
    job.routerSettings.intent = intent;
    job = RunRoutingJob(job);

    RouterIntentObjectiveRefiner.PairMetrics metrics =
        RouterIntentObjectiveRefiner.PairMetrics.measure(job.board, pair);

    assertTrue(metrics.parallelRatio() < 0.4, "nearby out-of-band copper must not count as coupled");
    assertTrue(metrics.uncoupledLengthMm() > 45.0, "out-of-band parallel copper must remain uncoupled");
    assertFalse(metrics.hardPass(pair));
  }

  private RouterIntentSettings.DifferentialPairIntent pairIntent() {
    RouterIntentSettings.DifferentialPairIntent pair = new RouterIntentSettings.DifferentialPairIntent();
    pair.id = "usb_pair";
    pair.positiveNet = "USB_D+";
    pair.negativeNet = "USB_D-";
    pair.positiveFrom = "P1.1";
    pair.positiveTo = "P2.1";
    pair.negativeFrom = "N1.1";
    pair.negativeTo = "N2.1";
    pair.allowedLayers = new String[] {"F.Cu"};
    pair.sameLayerRequired = true;
    pair.maxViasPerNet = 0;
    pair.matchedViaTransitionsRequired = true;
    pair.routeAsCoupledPair = false;
    pair.targetWidthMm = 0.2;
    pair.targetGapMm = 0.18;
    pair.gapToleranceMm = 0.12;
    pair.maxSkewMm = 1.0;
    pair.minParallelLengthRatio = 0.8;
    pair.maxUncoupledLengthMm = 20.0;
    pair.requireParallelEvidence = true;
    return pair;
  }
}
