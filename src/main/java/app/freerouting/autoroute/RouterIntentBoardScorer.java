package app.freerouting.autoroute;

import app.freerouting.board.RoutingBoard;
import app.freerouting.board.Unit;
import app.freerouting.board.Via;
import app.freerouting.core.scoring.BoardStatistics;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.RouterScoringSettings;

final class RouterIntentBoardScorer {
  private static final float INTENT_INCOMPLETE_PENALTY_POINTS = 25f;
  private static final float PROTECTED_VIA_PENALTY_POINTS = 20f;
  private static final float CRITICAL_PATH_EXCESS_LENGTH_PENALTY_POINTS_PER_MM = 10f;
  private static final float DIFFERENTIAL_PAIR_SKEW_PENALTY_POINTS_PER_MM = 20f;

  private RouterIntentBoardScorer() {
  }

  static float normalizedScore(
      RoutingBoard board,
      RouterScoringSettings scoringSettings,
      RouterIntentSettings intent) {
    float baseScore = new BoardStatistics(board).getNormalizedScore(scoringSettings);
    if (!hasScoringIntent(intent)) {
      return baseScore;
    }
    float intentPenalty = intentIncompletePenalty(board, intent)
        + protectedViaPenalty(board, intent)
        + criticalPathExcessLengthPenalty(board, intent)
        + differentialPairSkewPenalty(board, intent);
    return Math.max(0f, baseScore - intentPenalty);
  }

  static float intentIncompletePenalty(RoutingBoard board, RouterIntentSettings intent) {
    if (board == null || board.rules == null || intent == null || !intent.hasNetIntents()) {
      return 0f;
    }

    DesignRulesChecker checker = new DesignRulesChecker(board, null);
    float penalty = 0f;
    for (int netNo = 1; netNo <= board.rules.nets.max_net_no(); netNo++) {
      Net net = board.rules.nets.get(netNo);
      if (net == null || net.name == null) {
        continue;
      }

      int intentRank = intentRank(intent, net.name);
      if (intentRank <= 0) {
        continue;
      }

      int incompleteCount = checker.getIncompleteCount(netNo);
      penalty += incompleteCount * intentRank * INTENT_INCOMPLETE_PENALTY_POINTS;
    }
    return penalty;
  }

  static float protectedViaPenalty(RoutingBoard board, RouterIntentSettings intent) {
    if (board == null || board.rules == null || intent == null || !intent.hasNetIntents()) {
      return 0f;
    }

    float penalty = 0f;
    for (Via via : board.get_vias()) {
      for (int netNo = 1; netNo <= board.rules.nets.max_net_no(); netNo++) {
        Net net = board.rules.nets.get(netNo);
        if (net == null || net.name == null || !via.contains_net(netNo)) {
          continue;
        }

        int protectionRank = intent.ripupProtectionRankForNet(net.name);
        if (protectionRank > 0) {
          penalty += protectionRank * PROTECTED_VIA_PENALTY_POINTS;
        }
      }
    }
    return penalty;
  }

  static float criticalPathExcessLengthPenalty(RoutingBoard board, RouterIntentSettings intent) {
    if (board == null
        || board.rules == null
        || intent == null
        || intent.criticalPaths == null
        || intent.criticalPaths.length == 0) {
      return 0f;
    }

    double mmResolution = board.communication.get_resolution(Unit.MM);
    if (mmResolution <= 0) {
      return 0f;
    }

    float penalty = 0f;
    for (RouterIntentSettings.CriticalPathIntent criticalPath : intent.criticalPaths) {
      if (criticalPath == null
          || criticalPath.net == null
          || criticalPath.maxLengthMm == null
          || criticalPath.maxLengthMm <= 0) {
        continue;
      }

      Net net = board.rules.nets.get(criticalPath.net, 1);
      if (net == null) {
        continue;
      }

      double routedLengthMm = net.get_trace_length() / mmResolution;
      double excessMm = routedLengthMm - criticalPath.maxLengthMm;
      if (excessMm > 0) {
        penalty += excessMm
            * priorityRank(criticalPath.priority)
            * CRITICAL_PATH_EXCESS_LENGTH_PENALTY_POINTS_PER_MM;
      }
    }
    return penalty;
  }

  static float differentialPairSkewPenalty(RoutingBoard board, RouterIntentSettings intent) {
    if (board == null
        || board.rules == null
        || intent == null
        || intent.differentialPairs == null
        || intent.differentialPairs.length == 0) {
      return 0f;
    }

    double mmResolution = board.communication.get_resolution(Unit.MM);
    if (mmResolution <= 0) {
      return 0f;
    }

    float penalty = 0f;
    for (RouterIntentSettings.DifferentialPairIntent differentialPair : intent.differentialPairs) {
      if (differentialPair == null
          || differentialPair.positiveNet == null
          || differentialPair.negativeNet == null
          || differentialPair.maxSkewMm == null
          || differentialPair.maxSkewMm < 0) {
        continue;
      }

      Net positiveNet = board.rules.nets.get(differentialPair.positiveNet, 1);
      Net negativeNet = board.rules.nets.get(differentialPair.negativeNet, 1);
      if (positiveNet == null || negativeNet == null) {
        continue;
      }

      double positiveLengthMm = positiveNet.get_trace_length() / mmResolution;
      double negativeLengthMm = negativeNet.get_trace_length() / mmResolution;
      double skewMm = Math.abs(positiveLengthMm - negativeLengthMm);
      double excessMm = skewMm - differentialPair.maxSkewMm;
      if (excessMm > 0) {
        penalty += excessMm
            * priorityRank(differentialPair.priority)
            * DIFFERENTIAL_PAIR_SKEW_PENALTY_POINTS_PER_MM;
      }
    }
    return penalty;
  }

  private static boolean hasScoringIntent(RouterIntentSettings intent) {
    return intent != null
        && (intent.hasNetIntents()
            || (intent.criticalPaths != null && intent.criticalPaths.length > 0)
            || (intent.differentialPairs != null && intent.differentialPairs.length > 0));
  }

  private static int intentRank(RouterIntentSettings intent, String netName) {
    return intent.priorityRankForNet(netName)
        + intent.scopeRankForNet(netName)
        + intent.ripupProtectionRankForNet(netName);
  }

  private static int priorityRank(RouterIntentSettings.Priority priority) {
    if (priority == null) {
      return 1;
    }
    return switch (priority) {
      case NORMAL -> 1;
      case HIGH -> 2;
      case CRITICAL -> 3;
    };
  }
}
