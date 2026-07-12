package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

class RouterIntentObjectiveRefinerSafetyTest {

  @Test
  void rejectsReplacementClearanceViolationEvenWhenCountDoesNotIncrease() {
    assertFalse(RouterIntentObjectiveRefiner.hasNoNewClearanceViolations(
        Set.of("1-2-0"),
        Set.of("3-4-0")));
  }

  @Test
  void acceptsOnlyExistingOrRemovedClearanceViolations() {
    Set<String> baseline = Set.of("1-2-0", "3-4-1");

    assertTrue(RouterIntentObjectiveRefiner.hasNoNewClearanceViolations(baseline, baseline));
    assertTrue(RouterIntentObjectiveRefiner.hasNoNewClearanceViolations(baseline, Set.of("1-2-0")));
    assertTrue(RouterIntentObjectiveRefiner.hasNoNewClearanceViolations(baseline, Set.of()));
  }

  @Test
  void intervalCoverAlwaysPrefersStrictlyFartherReachBeforeItemIdTieBreak() {
    List<RouterIntentObjectiveRefiner.CoverageInterval> intervals = new ArrayList<>(List.of(
        new RouterIntentObjectiveRefiner.CoverageInterval(0.0, 0.500000005, 10),
        new RouterIntentObjectiveRefiner.CoverageInterval(0.0, 0.5, 1),
        new RouterIntentObjectiveRefiner.CoverageInterval(0.500000015, 1.0, 20)));
    Set<Integer> coveringIds = new LinkedHashSet<>();

    assertTrue(RouterIntentObjectiveRefiner.intervalsCoverSegment(
        intervals,
        coveringIds,
        1e-8));
    assertEquals(Set.of(10, 20), coveringIds);
  }
}
