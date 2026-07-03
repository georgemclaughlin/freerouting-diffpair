package app.freerouting.autoroute;

import app.freerouting.settings.RouterIntentSettings;

final class RouterIntentRoutingPolicy {
  private static final double NON_PREFERRED_LAYER_COST_FACTOR = 4.0;
  private static final double CRITICAL_NET_VIA_COST_FACTOR = 1.5;
  private static final double LOCAL_SUPPORT_VIA_COST_FACTOR = 1.2;
  private static final double SOURCE_COPPER_VIA_COST_FACTOR = 3.0;
  private static final double CRITICAL_NET_RIPUP_COST_FACTOR = 2.0;
  private static final double LOCAL_SUPPORT_RIPUP_COST_FACTOR = 1.2;
  private static final double SOURCE_COPPER_RIPUP_COST_FACTOR = 8.0;
  private static final double LOCAL_SCOPE_EXIT_COST_FACTOR = 6.0;
  private static final double DIFFERENTIAL_PAIR_SIBLING_LAYER_COST_FACTOR = 3.0;
  private static final double DIFFERENTIAL_PAIR_CORRIDOR_EXIT_COST_FACTOR = 0.25;
  private static final double DIFFERENTIAL_PAIR_EXTRA_VIA_COST_FACTOR = 1.5;
  private static final double DIFFERENTIAL_PAIR_UNMATCHED_VIA_TRANSITION_COST_FACTOR = 0.5;
  private static final double DIFFERENTIAL_PAIR_SKEW_EXCESS_COST_FACTOR = 1.0;
  private static final double DIFFERENTIAL_PAIR_CORRIDOR_OBSTACLE_RIPUP_COST_FACTOR = 0.25;

  private RouterIntentRoutingPolicy() {
  }

  static int compareNetNames(RouterIntentSettings intent, String leftNetName, String rightNetName) {
    int priorityCompare = Integer.compare(priorityRank(intent, rightNetName), priorityRank(intent, leftNetName));
    if (priorityCompare != 0) {
      return priorityCompare;
    }

    int scopeCompare = Integer.compare(scopeRank(intent, rightNetName), scopeRank(intent, leftNetName));
    if (scopeCompare != 0) {
      return scopeCompare;
    }

    int differentialPairCompare = compareDifferentialPairMembership(intent, leftNetName, rightNetName);
    if (differentialPairCompare != 0) {
      return differentialPairCompare;
    }

    return Integer.compare(ripupProtectionRank(intent, rightNetName), ripupProtectionRank(intent, leftNetName));
  }

  static AutorouteControl.ExpansionCostFactor traceCostForLayer(
      RouterIntentSettings intent,
      String netName,
      String layerName,
      AutorouteControl.ExpansionCostFactor base) {
    if (intent == null || base == null || !intent.hasPreferredLayerIntent(netName)
        || intent.isPreferredLayerForNet(netName, layerName)) {
      return base;
    }
    return new AutorouteControl.ExpansionCostFactor(
        base.horizontal * NON_PREFERRED_LAYER_COST_FACTOR,
        base.vertical * NON_PREFERRED_LAYER_COST_FACTOR);
  }

  static AutorouteControl.ExpansionCostFactor traceCostForDifferentialPairSiblingLayer(
      RouterIntentSettings intent,
      String netName,
      boolean siblingUsesLayer,
      AutorouteControl.ExpansionCostFactor base) {
    if (intent == null
        || base == null
        || siblingUsesLayer
        || intent.differentialPairSiblingNetForNet(netName) == null) {
      return base;
    }
    return new AutorouteControl.ExpansionCostFactor(
        base.horizontal * DIFFERENTIAL_PAIR_SIBLING_LAYER_COST_FACTOR,
        base.vertical * DIFFERENTIAL_PAIR_SIBLING_LAYER_COST_FACTOR);
  }

  static double viaCostFactor(RouterIntentSettings intent, String netName) {
    double ripupFactor = switch (ripupProtectionRank(intent, netName)) {
      case 1 -> CRITICAL_NET_VIA_COST_FACTOR;
      case 2 -> LOCAL_SUPPORT_VIA_COST_FACTOR;
      case 3 -> SOURCE_COPPER_VIA_COST_FACTOR;
      default -> 1.0;
    };
    return ripupFactor * differentialPairViaCostFactor(intent, netName);
  }

  static double ripupCostFactor(RouterIntentSettings intent, String netName) {
    return switch (ripupProtectionRank(intent, netName)) {
      case 1 -> CRITICAL_NET_RIPUP_COST_FACTOR;
      case 2 -> LOCAL_SUPPORT_RIPUP_COST_FACTOR;
      case 3 -> SOURCE_COPPER_RIPUP_COST_FACTOR;
      default -> 1.0;
    };
  }

  static double localScopeExitCostFactor(RouterIntentSettings intent, String netName) {
    return intent != null && intent.hasLocalConfinementIntent(netName) ? LOCAL_SCOPE_EXIT_COST_FACTOR : 0.0;
  }

  static double differentialPairCorridorExitCostFactor(RouterIntentSettings intent, String netName) {
    return intent != null && intent.differentialPairSiblingNetForNet(netName) != null
        ? DIFFERENTIAL_PAIR_CORRIDOR_EXIT_COST_FACTOR
        : 0.0;
  }

  static double differentialPairUnmatchedViaTransitionCostFactor(RouterIntentSettings intent, String netName) {
    return intent != null && intent.differentialPairSiblingNetForNet(netName) != null
        ? DIFFERENTIAL_PAIR_UNMATCHED_VIA_TRANSITION_COST_FACTOR
        : 0.0;
  }

  static double differentialPairSkewExcessCostFactor(RouterIntentSettings intent, String netName) {
    return intent != null && intent.differentialPairSiblingNetForNet(netName) != null
        ? DIFFERENTIAL_PAIR_SKEW_EXCESS_COST_FACTOR
        : 0.0;
  }

  static double differentialPairCorridorObstacleRipupCostFactor(RouterIntentSettings intent, String netName) {
    return intent != null && intent.differentialPairSiblingNetForNet(netName) != null
        ? DIFFERENTIAL_PAIR_CORRIDOR_OBSTACLE_RIPUP_COST_FACTOR
        : 1.0;
  }

  private static double differentialPairViaCostFactor(RouterIntentSettings intent, String netName) {
    return intent != null && intent.differentialPairSiblingNetForNet(netName) != null
        ? DIFFERENTIAL_PAIR_EXTRA_VIA_COST_FACTOR
        : 1.0;
  }

  private static int priorityRank(RouterIntentSettings intent, String netName) {
    return intent == null ? 0 : intent.priorityRankForNet(netName);
  }

  private static int scopeRank(RouterIntentSettings intent, String netName) {
    return intent == null ? 0 : intent.scopeRankForNet(netName);
  }

  private static int ripupProtectionRank(RouterIntentSettings intent, String netName) {
    return intent == null ? 0 : intent.ripupProtectionRankForNet(netName);
  }

  private static int compareDifferentialPairMembership(
      RouterIntentSettings intent,
      String leftNetName,
      String rightNetName) {
    if (intent == null) {
      return 0;
    }

    String leftGroup = intent.differentialPairGroupForNet(leftNetName);
    String rightGroup = intent.differentialPairGroupForNet(rightNetName);
    if (leftGroup == null && rightGroup == null) {
      return 0;
    }
    if (leftGroup == null) {
      return 1;
    }
    if (rightGroup == null) {
      return -1;
    }

    int groupCompare = leftGroup.compareTo(rightGroup);
    if (groupCompare != 0) {
      return groupCompare;
    }
    return Integer.compare(
        intent.differentialPairMemberRankForNet(leftNetName),
        intent.differentialPairMemberRankForNet(rightNetName));
  }
}
