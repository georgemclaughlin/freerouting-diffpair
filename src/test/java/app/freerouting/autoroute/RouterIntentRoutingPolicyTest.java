package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.settings.RouterIntentSettings;
import org.junit.jupiter.api.Test;

class RouterIntentRoutingPolicyTest {

  @Test
  void compareNetNamesRoutesCriticalThenLocalThenProtectedNetsFirst() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "CRITICAL_3V3",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL,
            "F.Cu"),
        netIntent(
            "LOCAL_BOOT",
            RouterIntentSettings.Priority.NORMAL,
            RouterIntentSettings.Scope.LOCAL,
            RouterIntentSettings.RipupProtection.LOCAL_SUPPORT,
            "F.Cu"),
        netIntent(
            "ORDINARY",
            RouterIntentSettings.Priority.NORMAL,
            RouterIntentSettings.Scope.NORMAL,
            RouterIntentSettings.RipupProtection.NONE,
            "B.Cu"));

    assertTrue(RouterIntentRoutingPolicy.compareNetNames(intent, "CRITICAL_3V3", "LOCAL_BOOT") < 0);
    assertTrue(RouterIntentRoutingPolicy.compareNetNames(intent, "LOCAL_BOOT", "ORDINARY") < 0);
    assertTrue(RouterIntentRoutingPolicy.compareNetNames(intent, "CRITICAL_3V3", "ORDINARY") < 0);
    assertEquals(0, RouterIntentRoutingPolicy.compareNetNames(intent, "UNKNOWN_A", "UNKNOWN_B"));
  }

  @Test
  void traceCostForLayerPenalizesNonPreferredLayersWithoutDisablingThem() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "USB_D_PLUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL,
            "F.Cu"));
    AutorouteControl.ExpansionCostFactor base = new AutorouteControl.ExpansionCostFactor(2.0, 3.0);

    assertSame(base, RouterIntentRoutingPolicy.traceCostForLayer(intent, "USB_D_PLUS", "F.Cu", base));

    AutorouteControl.ExpansionCostFactor penalized = RouterIntentRoutingPolicy.traceCostForLayer(
        intent, "USB_D_PLUS", "B.Cu", base);
    assertEquals(8.0, penalized.horizontal);
    assertEquals(12.0, penalized.vertical);
  }

  @Test
  void viaCostFactorUsesClosedRipupProtectionRanks() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "NORMAL_NET",
            RouterIntentSettings.Priority.NORMAL,
            RouterIntentSettings.Scope.NORMAL,
            RouterIntentSettings.RipupProtection.NONE),
        netIntent(
            "CRITICAL_NET",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL),
        netIntent(
            "LOCAL_SUPPORT_NET",
            RouterIntentSettings.Priority.NORMAL,
            RouterIntentSettings.Scope.LOCAL,
            RouterIntentSettings.RipupProtection.LOCAL_SUPPORT),
        netIntent(
            "SOURCE_COPPER_NET",
            RouterIntentSettings.Priority.NORMAL,
            RouterIntentSettings.Scope.LOCAL,
            RouterIntentSettings.RipupProtection.SOURCE_COPPER));

    assertEquals(1.0, RouterIntentRoutingPolicy.viaCostFactor(intent, "NORMAL_NET"));
    assertEquals(1.5, RouterIntentRoutingPolicy.viaCostFactor(intent, "CRITICAL_NET"));
    assertEquals(2.0, RouterIntentRoutingPolicy.viaCostFactor(intent, "LOCAL_SUPPORT_NET"));
    assertEquals(3.0, RouterIntentRoutingPolicy.viaCostFactor(intent, "SOURCE_COPPER_NET"));
    assertEquals(1.0, RouterIntentRoutingPolicy.viaCostFactor(intent, "UNKNOWN_NET"));
  }

  @Test
  void ripupCostFactorProtectsHigherRankedCopperMoreStrongly() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "NORMAL_NET",
            RouterIntentSettings.Priority.NORMAL,
            RouterIntentSettings.Scope.NORMAL,
            RouterIntentSettings.RipupProtection.NONE),
        netIntent(
            "CRITICAL_NET",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL),
        netIntent(
            "LOCAL_SUPPORT_NET",
            RouterIntentSettings.Priority.NORMAL,
            RouterIntentSettings.Scope.LOCAL,
            RouterIntentSettings.RipupProtection.LOCAL_SUPPORT),
        netIntent(
            "SOURCE_COPPER_NET",
            RouterIntentSettings.Priority.NORMAL,
            RouterIntentSettings.Scope.LOCAL,
            RouterIntentSettings.RipupProtection.SOURCE_COPPER));

    assertEquals(1.0, RouterIntentRoutingPolicy.ripupCostFactor(intent, "NORMAL_NET"));
    assertEquals(2.0, RouterIntentRoutingPolicy.ripupCostFactor(intent, "CRITICAL_NET"));
    assertEquals(4.0, RouterIntentRoutingPolicy.ripupCostFactor(intent, "LOCAL_SUPPORT_NET"));
    assertEquals(8.0, RouterIntentRoutingPolicy.ripupCostFactor(intent, "SOURCE_COPPER_NET"));
    assertEquals(1.0, RouterIntentRoutingPolicy.ripupCostFactor(intent, "UNKNOWN_NET"));
  }

  @Test
  void localScopeExitCostFactorOnlyAppliesToLocalNets() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "NORMAL_NET",
            RouterIntentSettings.Priority.NORMAL,
            RouterIntentSettings.Scope.NORMAL,
            RouterIntentSettings.RipupProtection.NONE),
        netIntent(
            "LOCAL_SUPPORT_NET",
            RouterIntentSettings.Priority.NORMAL,
            RouterIntentSettings.Scope.LOCAL,
            RouterIntentSettings.RipupProtection.LOCAL_SUPPORT));

    assertEquals(0.0, RouterIntentRoutingPolicy.localScopeExitCostFactor(intent, "NORMAL_NET"));
    assertTrue(RouterIntentRoutingPolicy.localScopeExitCostFactor(intent, "LOCAL_SUPPORT_NET") > 1.0);
    assertEquals(0.0, RouterIntentRoutingPolicy.localScopeExitCostFactor(intent, "UNKNOWN_NET"));
  }

  private RouterIntentSettings intentWith(RouterIntentSettings.NetIntent... netIntents) {
    RouterIntentSettings intent = new RouterIntentSettings();
    intent.netIntents = netIntents;
    return intent;
  }

  private RouterIntentSettings.NetIntent netIntent(
      String netName,
      RouterIntentSettings.Priority priority,
      RouterIntentSettings.Scope scope,
      RouterIntentSettings.RipupProtection ripupProtection,
      String... preferredLayers) {
    RouterIntentSettings.NetIntent intent = new RouterIntentSettings.NetIntent();
    intent.net = netName;
    intent.priority = priority;
    intent.scope = scope;
    intent.ripupProtection = ripupProtection;
    intent.preferredLayers = preferredLayers;
    return intent;
  }
}
