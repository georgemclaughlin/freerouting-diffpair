package app.freerouting.autoroute;

import app.freerouting.board.RoutingBoard;
import app.freerouting.board.PolylineTrace;
import app.freerouting.board.Trace;
import app.freerouting.board.Unit;
import app.freerouting.board.Via;
import app.freerouting.core.scoring.BoardStatistics;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.RouterScoringSettings;

final class RouterIntentBoardScorer {
  private static final float INTENT_INCOMPLETE_PENALTY_POINTS = 25f;
  private static final float PROTECTED_VIA_PENALTY_POINTS = 20f;
  private static final float CRITICAL_PATH_EXCESS_LENGTH_PENALTY_POINTS_PER_MM = 10f;
  private static final float DIFFERENTIAL_PAIR_SKEW_PENALTY_POINTS_PER_MM = 20f;
  private static final float DIFFERENTIAL_PAIR_GAP_PENALTY_POINTS_PER_MM = 35f;
  private static final float ROUTE_LENGTH_MATCH_SKEW_PENALTY_POINTS_PER_MM = 20f;
  private static final float LOCAL_SCOPE_EXCURSION_PENALTY_POINTS_PER_MM = 5f;
  private static final float RETURN_PATH_PLANE_LAYER_PENALTY_POINTS_PER_MM = 8f;

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
        + differentialPairSkewPenalty(board, intent)
        + differentialPairGapPenalty(board, intent)
        + routeLengthMatchSkewPenalty(board, intent)
        + localScopeExcursionPenalty(board, intent)
        + returnPathPlaneLayerPenalty(board, intent);
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

  static float differentialPairGapPenalty(RoutingBoard board, RouterIntentSettings intent) {
    if (board == null
        || board.rules == null
        || board.communication == null
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
          || !Boolean.TRUE.equals(differentialPair.routeAsCoupledPair)
          || differentialPair.targetGapMm == null
          || differentialPair.targetGapMm < 0) {
        continue;
      }

      Net positiveNet = board.rules.nets.get(differentialPair.positiveNet, 1);
      Net negativeNet = board.rules.nets.get(differentialPair.negativeNet, 1);
      if (positiveNet == null || negativeNet == null) {
        continue;
      }

      double nearestGapBoard = nearestTraceGap(board, positiveNet.net_number, negativeNet.net_number);
      if (!Double.isFinite(nearestGapBoard)) {
        continue;
      }

      double nearestGapMm = nearestGapBoard / mmResolution;
      double toleranceMm = differentialPair.gapToleranceMm != null && differentialPair.gapToleranceMm >= 0
          ? differentialPair.gapToleranceMm
          : 0.0;
      double lowLimitMm = Math.max(0.0, differentialPair.targetGapMm - toleranceMm);
      double highLimitMm = differentialPair.targetGapMm + toleranceMm;
      double excessMm = nearestGapMm < lowLimitMm
          ? lowLimitMm - nearestGapMm
          : Math.max(0.0, nearestGapMm - highLimitMm);
      if (excessMm > 0) {
        penalty += excessMm
            * priorityRank(differentialPair.priority)
            * DIFFERENTIAL_PAIR_GAP_PENALTY_POINTS_PER_MM;
      }
    }
    return penalty;
  }

  static float routeLengthMatchSkewPenalty(RoutingBoard board, RouterIntentSettings intent) {
    if (board == null
        || board.rules == null
        || board.rules.nets == null
        || board.communication == null
        || intent == null
        || intent.routeLengthMatches == null
        || intent.routeLengthMatches.length == 0) {
      return 0f;
    }

    double mmResolution = board.communication.get_resolution(Unit.MM);
    if (mmResolution <= 0) {
      return 0f;
    }

    float penalty = 0f;
    for (RouterIntentSettings.RouteLengthMatchIntent match : intent.routeLengthMatches) {
      if (match == null
          || match.id == null
          || match.id.isEmpty()
          || match.nets == null
          || match.nets.length < 2
          || match.priority == null
          || match.maxSkewMm == null
          || match.maxSkewMm < 0
          || !Double.isFinite(match.maxSkewMm)) {
        continue;
      }
      if (hasEquivalentDifferentialPair(intent, match)) {
        continue;
      }

      double shortestLengthMm = Double.POSITIVE_INFINITY;
      double longestLengthMm = 0.0;
      boolean validGroup = true;
      for (String netName : match.nets) {
        if (netName == null || netName.isEmpty()) {
          validGroup = false;
          break;
        }

        Net net = board.rules.nets.get(netName, 1);
        if (net == null) {
          validGroup = false;
          break;
        }

        double routedLengthMm = net.get_trace_length() / mmResolution;
        if (routedLengthMm <= 0.0 || !Double.isFinite(routedLengthMm)) {
          validGroup = false;
          break;
        }

        shortestLengthMm = Math.min(shortestLengthMm, routedLengthMm);
        longestLengthMm = Math.max(longestLengthMm, routedLengthMm);
      }

      if (!validGroup || shortestLengthMm == Double.POSITIVE_INFINITY) {
        continue;
      }

      double excessMm = (longestLengthMm - shortestLengthMm) - match.maxSkewMm;
      if (excessMm > 0) {
        penalty += excessMm
            * priorityRank(match.priority)
            * ROUTE_LENGTH_MATCH_SKEW_PENALTY_POINTS_PER_MM;
      }
    }
    return penalty;
  }

  private static boolean hasEquivalentDifferentialPair(
      RouterIntentSettings intent,
      RouterIntentSettings.RouteLengthMatchIntent match) {
    if (intent == null
        || intent.differentialPairs == null
        || match == null
        || match.nets == null
        || match.nets.length != 2) {
      return false;
    }

    String leftNet = match.nets[0];
    String rightNet = match.nets[1];
    if (leftNet == null || leftNet.isEmpty() || rightNet == null || rightNet.isEmpty()) {
      return false;
    }

    for (RouterIntentSettings.DifferentialPairIntent differentialPair : intent.differentialPairs) {
      if (differentialPair == null
          || differentialPair.positiveNet == null
          || differentialPair.negativeNet == null) {
        continue;
      }
      if ((leftNet.equals(differentialPair.positiveNet) && rightNet.equals(differentialPair.negativeNet))
          || (leftNet.equals(differentialPair.negativeNet) && rightNet.equals(differentialPair.positiveNet))) {
        return true;
      }
    }
    return false;
  }

  private static double nearestTraceGap(RoutingBoard board, int firstNetNo, int secondNetNo) {
    double result = Double.POSITIVE_INFINITY;
    for (Trace firstTrace : board.get_traces()) {
      if (!(firstTrace instanceof PolylineTrace firstPolyline) || !firstTrace.contains_net(firstNetNo)) {
        continue;
      }
      for (Trace secondTrace : board.get_traces()) {
        if (!(secondTrace instanceof PolylineTrace secondPolyline)
            || !secondTrace.contains_net(secondNetNo)
            || firstTrace.get_layer() != secondTrace.get_layer()) {
          continue;
        }
        double gap = nearestPolylineGap(firstPolyline, secondPolyline);
        if (Double.isFinite(gap)) {
          result = Math.min(result, gap);
        }
      }
    }
    return result;
  }

  private static double nearestPolylineGap(PolylineTrace firstTrace, PolylineTrace secondTrace) {
    FloatPoint[] firstCorners = firstTrace.polyline().corner_approx_arr();
    FloatPoint[] secondCorners = secondTrace.polyline().corner_approx_arr();
    if (firstCorners.length < 2 || secondCorners.length < 2) {
      return Double.NaN;
    }

    double centerlineDistance = Double.POSITIVE_INFINITY;
    for (int i = 0; i < firstCorners.length - 1; i++) {
      for (int j = 0; j < secondCorners.length - 1; j++) {
        centerlineDistance = Math.min(
            centerlineDistance,
            segmentDistance(firstCorners[i], firstCorners[i + 1], secondCorners[j], secondCorners[j + 1]));
      }
    }
    if (!Double.isFinite(centerlineDistance)) {
      return Double.NaN;
    }
    return Math.max(0.0, centerlineDistance - firstTrace.get_half_width() - secondTrace.get_half_width());
  }

  private static double segmentDistance(FloatPoint firstStart, FloatPoint firstEnd, FloatPoint secondStart, FloatPoint secondEnd) {
    if (segmentsIntersect(firstStart, firstEnd, secondStart, secondEnd)) {
      return 0.0;
    }
    return Math.min(
        Math.min(pointSegmentDistance(firstStart, secondStart, secondEnd), pointSegmentDistance(firstEnd, secondStart, secondEnd)),
        Math.min(pointSegmentDistance(secondStart, firstStart, firstEnd), pointSegmentDistance(secondEnd, firstStart, firstEnd)));
  }

  private static double pointSegmentDistance(FloatPoint point, FloatPoint segmentStart, FloatPoint segmentEnd) {
    double dx = segmentEnd.x - segmentStart.x;
    double dy = segmentEnd.y - segmentStart.y;
    double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= 0.0) {
      return point.distance(segmentStart);
    }
    double t = ((point.x - segmentStart.x) * dx + (point.y - segmentStart.y) * dy) / lengthSquared;
    t = Math.max(0.0, Math.min(1.0, t));
    return point.distance(new FloatPoint(segmentStart.x + t * dx, segmentStart.y + t * dy));
  }

  private static boolean segmentsIntersect(FloatPoint firstStart, FloatPoint firstEnd, FloatPoint secondStart, FloatPoint secondEnd) {
    double o1 = orientation(firstStart, firstEnd, secondStart);
    double o2 = orientation(firstStart, firstEnd, secondEnd);
    double o3 = orientation(secondStart, secondEnd, firstStart);
    double o4 = orientation(secondStart, secondEnd, firstEnd);
    return o1 * o2 < 0.0 && o3 * o4 < 0.0;
  }

  private static double orientation(FloatPoint a, FloatPoint b, FloatPoint c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  }

  static float localScopeExcursionPenalty(RoutingBoard board, RouterIntentSettings intent) {
    if (board == null || board.rules == null || intent == null || !intent.hasNetIntents()) {
      return 0f;
    }

    double mmResolution = board.communication.get_resolution(Unit.MM);
    if (mmResolution <= 0) {
      return 0f;
    }

    float penalty = 0f;
    for (int netNo = 1; netNo <= board.rules.nets.max_net_no(); netNo++) {
      Net net = board.rules.nets.get(netNo);
      if (net == null || net.name == null || !intent.hasLocalConfinementIntent(net.name)) {
        continue;
      }

      IntBox localRegion = RouterIntentLocalScope.localRegion(board, intent, net);
      if (localRegion == null) {
        continue;
      }

      double excursionMm = 0.0;
      for (Trace trace : board.get_traces()) {
        if (!trace.contains_net(netNo)) {
          continue;
        }
        FloatPoint[] points = trace instanceof PolylineTrace polylineTrace
            ? polylineTrace.polyline().corner_approx_arr()
            : new FloatPoint[] { trace.first_corner().to_float(), trace.last_corner().to_float() };
        for (FloatPoint point : points) {
          excursionMm += RouterIntentLocalScope.distanceOutside(localRegion, point) / mmResolution;
        }
      }

      if (excursionMm > 0) {
        penalty += excursionMm
            * intentRank(intent, net.name)
            * LOCAL_SCOPE_EXCURSION_PENALTY_POINTS_PER_MM;
      }
    }
    return penalty;
  }

  static float returnPathPlaneLayerPenalty(RoutingBoard board, RouterIntentSettings intent) {
    if (board == null || board.rules == null || intent == null || !intent.hasNetIntents()) {
      return 0f;
    }

    double mmResolution = board.communication.get_resolution(Unit.MM);
    if (mmResolution <= 0 || board.layer_structure == null || board.layer_structure.arr == null) {
      return 0f;
    }

    float penalty = 0f;
    for (int netNo = 1; netNo <= board.rules.nets.max_net_no(); netNo++) {
      Net net = board.rules.nets.get(netNo);
      if (net == null || net.name == null || !intent.hasPlaneLayerIntent(net.name)) {
        continue;
      }

      double planeLayerSignalLengthMm = 0.0;
      for (Trace trace : board.get_traces()) {
        int layerIndex = trace.get_layer();
        if (!trace.contains_net(netNo)
            || layerIndex < 0
            || layerIndex >= board.layer_structure.arr.length
            || board.layer_structure.arr[layerIndex] == null
            || !intent.isPlaneLayerForNet(net.name, board.layer_structure.arr[layerIndex].name)) {
          continue;
        }
        planeLayerSignalLengthMm += trace.get_length() / mmResolution;
      }

      if (planeLayerSignalLengthMm > 0) {
        penalty += planeLayerSignalLengthMm
            * Math.max(1, intentRank(intent, net.name))
            * RETURN_PATH_PLANE_LAYER_PENALTY_POINTS_PER_MM;
      }
    }
    return penalty;
  }

  private static boolean hasScoringIntent(RouterIntentSettings intent) {
    return intent != null
        && (intent.hasNetIntents()
            || (intent.criticalPaths != null && intent.criticalPaths.length > 0)
            || (intent.differentialPairs != null && intent.differentialPairs.length > 0)
            || (intent.routeLengthMatches != null && intent.routeLengthMatches.length > 0));
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
