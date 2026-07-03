package app.freerouting.autoroute;

import app.freerouting.settings.RouterIntentSettings;

final class RouterIntentRoutingPolicy {
  private static final double NON_PREFERRED_LAYER_COST_FACTOR = 4.0;

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

  private static int priorityRank(RouterIntentSettings intent, String netName) {
    return intent == null ? 0 : intent.priorityRankForNet(netName);
  }

  private static int scopeRank(RouterIntentSettings intent, String netName) {
    return intent == null ? 0 : intent.scopeRankForNet(netName);
  }

  private static int ripupProtectionRank(RouterIntentSettings intent, String netName) {
    return intent == null ? 0 : intent.ripupProtectionRankForNet(netName);
  }
}
