package app.freerouting.autoroute;

import app.freerouting.board.RoutingBoard;
import app.freerouting.core.scoring.BoardStatistics;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.RouterScoringSettings;

final class RouterIntentBoardScorer {
  private static final float INTENT_INCOMPLETE_PENALTY_POINTS = 25f;

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
    return Math.max(0f, baseScore - intentIncompletePenalty(board, intent));
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

  private static int intentRank(RouterIntentSettings intent, String netName) {
    return intent.priorityRankForNet(netName)
        + intent.scopeRankForNet(netName)
        + intent.ripupProtectionRankForNet(netName);
  }
}
