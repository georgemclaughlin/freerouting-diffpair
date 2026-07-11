package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.settings.RouterIntentSettings;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class RouterIntentRoutingPolicyTest {

  @Test
  void compareNetNamesRoutesHigherRouteOrderRankFirst() {
    RouterIntentSettings.NetIntent critical = netIntent(
        "CRITICAL_3V3",
        RouterIntentSettings.Priority.CRITICAL,
        RouterIntentSettings.Scope.GLOBAL,
        RouterIntentSettings.RipupProtection.CRITICAL,
        "F.Cu");
    critical.routeOrderRank = 10;
    RouterIntentSettings.NetIntent earlySignal = netIntent(
        "USB_D_PLUS",
        RouterIntentSettings.Priority.NORMAL,
        RouterIntentSettings.Scope.NORMAL,
        RouterIntentSettings.RipupProtection.NONE,
        "F.Cu");
    earlySignal.routeOrderRank = 90;
    RouterIntentSettings intent = intentWith(critical, earlySignal);

    assertTrue(RouterIntentRoutingPolicy.compareNetNames(intent, "USB_D_PLUS", "CRITICAL_3V3") < 0);
    assertTrue(RouterIntentRoutingPolicy.compareNetNames(intent, "CRITICAL_3V3", "USB_D_PLUS") > 0);
  }

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
  void compareNetNamesKeepsDifferentialPairMembersAdjacent() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "VBUS_SENSE",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL,
            "F.Cu"),
        netIntent(
            "USB_D_MINUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL,
            "F.Cu"),
        netIntent(
            "USB_D_PLUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL,
            "F.Cu"));
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] {
        differentialPair("usb2_data", "USB_D_PLUS", "USB_D_MINUS")
    };

    List<String> nets = new ArrayList<>(List.of("VBUS_SENSE", "USB_D_MINUS", "USB_D_PLUS"));
    nets.sort((left, right) -> RouterIntentRoutingPolicy.compareNetNames(intent, left, right));

    assertEquals(List.of("USB_D_PLUS", "USB_D_MINUS", "VBUS_SENSE"), nets);
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
  void layerAllowedRejectsHardPairLayerViolation() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "USB_D_PLUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL));
    RouterIntentSettings.DifferentialPairIntent pair = differentialPair("usb2_data", "USB_D_PLUS", "USB_D_MINUS");
    pair.allowedLayers = new String[] { "F.Cu" };
    pair.sameLayerRequired = false;
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] { pair };
    AutorouteControl.ExpansionCostFactor base = new AutorouteControl.ExpansionCostFactor(2.0, 3.0);

    assertSame(base, RouterIntentRoutingPolicy.traceCostForLayer(intent, "USB_D_PLUS", "F.Cu", base));
    assertTrue(RouterIntentRoutingPolicy.layerAllowed(intent, "USB_D_PLUS", "F.Cu"));
    assertFalse(RouterIntentRoutingPolicy.layerAllowed(intent, "USB_D_PLUS", "In1.Cu"));
    assertTrue(RouterIntentRoutingPolicy.layerAllowed(intent, "VBUS", "In1.Cu"));
  }

  @Test
  void traceCostForDifferentialPairSiblingLayerPenalizesLayersUnusedBySibling() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "USB_D_PLUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL),
        netIntent(
            "USB_D_MINUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL));
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] {
        differentialPair("usb2_data", "USB_D_PLUS", "USB_D_MINUS")
    };
    AutorouteControl.ExpansionCostFactor base = new AutorouteControl.ExpansionCostFactor(2.0, 3.0);

    assertSame(
        base,
        RouterIntentRoutingPolicy.traceCostForDifferentialPairSiblingLayer(intent, "USB_D_MINUS", true, base));

    AutorouteControl.ExpansionCostFactor penalized =
        RouterIntentRoutingPolicy.traceCostForDifferentialPairSiblingLayer(intent, "USB_D_MINUS", false, base);
    assertEquals(6.0, penalized.horizontal);
    assertEquals(9.0, penalized.vertical);

    assertSame(
        base,
        RouterIntentRoutingPolicy.traceCostForDifferentialPairSiblingLayer(intent, "VBUS", false, base));
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
    assertEquals(1.2, RouterIntentRoutingPolicy.viaCostFactor(intent, "LOCAL_SUPPORT_NET"));
    assertEquals(3.0, RouterIntentRoutingPolicy.viaCostFactor(intent, "SOURCE_COPPER_NET"));
    assertEquals(1.0, RouterIntentRoutingPolicy.viaCostFactor(intent, "UNKNOWN_NET"));
  }

  @Test
  void viaCostFactorPenalizesDifferentialPairMembers() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "USB_D_PLUS",
            RouterIntentSettings.Priority.NORMAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.NONE),
        netIntent(
            "USB_D_MINUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL),
        netIntent(
            "VBUS",
            RouterIntentSettings.Priority.NORMAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.NONE));
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] {
        differentialPair("usb2_data", "USB_D_PLUS", "USB_D_MINUS")
    };

    assertEquals(1.5, RouterIntentRoutingPolicy.viaCostFactor(intent, "USB_D_PLUS"));
    assertEquals(2.25, RouterIntentRoutingPolicy.viaCostFactor(intent, "USB_D_MINUS"));
    assertEquals(1.0, RouterIntentRoutingPolicy.viaCostFactor(intent, "VBUS"));
  }

  @Test
  void viasAllowedRejectsHardNoViaPairMembers() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "USB_D_PLUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL));
    RouterIntentSettings.DifferentialPairIntent pair = differentialPair("usb2_data", "USB_D_PLUS", "USB_D_MINUS");
    pair.maxViasPerNet = 0;
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] { pair };

    assertFalse(RouterIntentRoutingPolicy.viasAllowed(intent, "USB_D_PLUS"));
    assertTrue(Double.isInfinite(RouterIntentRoutingPolicy.viaCostFactor(intent, "USB_D_PLUS")));
    assertTrue(RouterIntentRoutingPolicy.viasAllowed(intent, "VBUS"));
    assertEquals(1.0, RouterIntentRoutingPolicy.viaCostFactor(intent, "VBUS"));
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
    assertEquals(1.2, RouterIntentRoutingPolicy.ripupCostFactor(intent, "LOCAL_SUPPORT_NET"));
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

  @Test
  void differentialPairSkewExcessCostFactorOnlyAppliesToPairMembers() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "USB_D_PLUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL),
        netIntent(
            "USB_D_MINUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL));
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] {
        differentialPair("usb2_data", "USB_D_PLUS", "USB_D_MINUS")
    };

    assertTrue(RouterIntentRoutingPolicy.differentialPairSkewExcessCostFactor(intent, "USB_D_PLUS") > 0.0);
    assertTrue(RouterIntentRoutingPolicy.differentialPairSkewExcessCostFactor(intent, "USB_D_MINUS") > 0.0);
    assertEquals(0.0, RouterIntentRoutingPolicy.differentialPairSkewExcessCostFactor(intent, "VBUS"));
  }

  @Test
  void coupledDifferentialPairUsesStrongCorridorExitCost() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "USB_D_PLUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL),
        netIntent(
            "USB_D_MINUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL));
    RouterIntentSettings.DifferentialPairIntent pair = differentialPair("usb2_data", "USB_D_PLUS", "USB_D_MINUS");
    pair.routeAsCoupledPair = true;
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] { pair };

    assertTrue(RouterIntentRoutingPolicy.differentialPairCorridorExitCostFactor(intent, "USB_D_PLUS") >= 10.0);
    assertTrue(RouterIntentRoutingPolicy.differentialPairCorridorExitCostFactor(intent, "USB_D_MINUS") >= 10.0);
    assertEquals(0.0, RouterIntentRoutingPolicy.differentialPairCorridorExitCostFactor(intent, "VBUS"));
  }

  @Test
  void coupledDifferentialPairUsesSeparateSoftCenterlineBandCost() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "USB_D_PLUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL),
        netIntent(
            "USB_D_MINUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL));
    RouterIntentSettings.DifferentialPairIntent pair = differentialPair("usb2_data", "USB_D_PLUS", "USB_D_MINUS");
    pair.routeAsCoupledPair = true;
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] { pair };

    double centerlineFactor = RouterIntentRoutingPolicy.differentialPairCenterlineBandCostFactor(
        intent,
        "USB_D_PLUS");
    double corridorFactor = RouterIntentRoutingPolicy.differentialPairCorridorExitCostFactor(
        intent,
        "USB_D_PLUS");
    assertEquals(2.5, centerlineFactor);
    assertTrue(centerlineFactor < corridorFactor);
    assertEquals(0.0, RouterIntentRoutingPolicy.differentialPairCenterlineBandCostFactor(intent, "VBUS"));
  }

  @Test
  void differentialPairCorridorObstacleRipupCostFactorOnlyDiscountsPairMemberRoutes() {
    RouterIntentSettings intent = intentWith(
        netIntent(
            "USB_D_PLUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL),
        netIntent(
            "USB_D_MINUS",
            RouterIntentSettings.Priority.CRITICAL,
            RouterIntentSettings.Scope.GLOBAL,
            RouterIntentSettings.RipupProtection.CRITICAL));
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] {
        differentialPair("usb2_data", "USB_D_PLUS", "USB_D_MINUS")
    };

    assertTrue(RouterIntentRoutingPolicy.differentialPairCorridorObstacleRipupCostFactor(intent, "USB_D_PLUS") < 1.0);
    assertTrue(RouterIntentRoutingPolicy.differentialPairCorridorObstacleRipupCostFactor(intent, "USB_D_MINUS") < 1.0);
    assertEquals(1.0, RouterIntentRoutingPolicy.differentialPairCorridorObstacleRipupCostFactor(intent, "VBUS"));
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

  private RouterIntentSettings.DifferentialPairIntent differentialPair(
      String id,
      String positiveNet,
      String negativeNet) {
    RouterIntentSettings.DifferentialPairIntent intent = new RouterIntentSettings.DifferentialPairIntent();
    intent.id = id;
    intent.positiveNet = positiveNet;
    intent.negativeNet = negativeNet;
    intent.priority = RouterIntentSettings.Priority.CRITICAL;
    return intent;
  }
}
