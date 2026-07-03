package app.freerouting.autoroute;

import app.freerouting.board.RoutingBoard;
import app.freerouting.board.Via;
import app.freerouting.core.scoring.BoardStatistics;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.RouterScoringSettings;

final class RouterIntentBoardScorer {
  private static final float INTENT_INCOMPLETE_PENALTY_POINTS = 25f;
  private static final float PROTECTED_VIA_PENALTY_POINTS = 20f;

  private RouterIntentBoardScorer() {
  }

  static float normalizedScore(
      RoutingBoard board,
      RouterScoringSettings scoringSettings,
      RouterIntentSettings intent) {
    float baseScore = new BoardStatistics(board).getNormalizedScore(scoringSettings);
    if (intent == null || !intent.hasNetIntents()) {
      return baseScore;
    }
    float intentPenalty = intentIncompletePenalty(board, intent)
        + protectedViaPenalty(board, intent);
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

  private static int intentRank(RouterIntentSettings intent, String netName) {
    return intent.priorityRankForNet(netName)
        + intent.scopeRankForNet(netName)
        + intent.ripupProtectionRankForNet(netName);
  }
}
