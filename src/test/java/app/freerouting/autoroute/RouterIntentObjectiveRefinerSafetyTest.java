package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
}
