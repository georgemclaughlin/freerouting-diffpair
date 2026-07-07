package app.freerouting.autoroute;

import app.freerouting.board.Component;
import app.freerouting.board.FixedState;
import app.freerouting.board.Item;
import app.freerouting.board.Pin;
import app.freerouting.board.PolylineTrace;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.Trace;
import app.freerouting.board.Unit;
import app.freerouting.board.Via;
import app.freerouting.core.RoutingJob;
import app.freerouting.drc.AirLine;
import app.freerouting.drc.ClearanceViolation;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.drc.UnconnectedItems;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Point;
import app.freerouting.geometry.planar.Polyline;
import app.freerouting.logger.FRLogger;
import app.freerouting.rules.DifferentialPair;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * Applies router-intent behavior for differential pairs and route-length groups.
 *
 * <p>Coupled differential-pair intents are reserved before the general maze router runs and are
 * re-evaluated after routing. Single-net length tuning is deliberately kept away from those
 * coupled pairs because it can satisfy skew while destroying the parallel corridor.
 */
public class DifferentialPairAutorouter {

  private static final double DEFAULT_MAX_SKEW_MM = 1.0;
  private static final double TARGET_SKEW_FRACTION = 0.50;
  private static final int MAX_ATTEMPTS_PER_PAIR = 8;
  private static final int MAX_ATTEMPTS_PER_ROUTE_LENGTH_GROUP = 24;
  private static final int MAX_CANDIDATES_PER_PASS = 128;
  private static final int MAX_COUPLED_PREROUTE_CANDIDATES = 512;
  private static final int MAX_COUPLED_REPLACE_CANDIDATES = 512;
  private static final int MIN_COUPLED_CANDIDATES_PER_FAMILY = 48;
  private static final int COUPLED_INSERT_MAX_TRACE_RECURSION_DEPTH = 20;
  private static final int COUPLED_INSERT_MAX_VIA_RECURSION_DEPTH = 0;
  private static final int COUPLED_INSERT_MAX_SPRING_OVER_RECURSION_DEPTH = 5;
  private static final int COUPLED_INSERT_PULL_TIGHT_ACCURACY = 20;
  private static final double PAIR_MEANDER_MIN_AMPLITUDE_MM = 0.20;
  private static final double PAIR_MEANDER_MAX_AMPLITUDE_MM = 3.0;
  private static final double PAIR_MEANDER_MIN_SPACING_MM = 0.60;
  private static final double PAIR_MEANDER_MIN_BUMP_WIDTH_MM = 0.80;
  private static final double FLOW_THROUGH_MIN_BUMP_HEIGHT_MM = 0.20;
  private static final double FLOW_THROUGH_MAX_BUMP_HEIGHT_MM = 2.50;
  private static final double FLOW_THROUGH_MIN_BUMP_WIDTH_MM = 0.80;
  private static final double FLOW_THROUGH_MIN_BUMP_SPACING_MM = 0.60;
  private static final double FLOW_THROUGH_MIN_PLATEAU_MM = 0.60;
  private static final double FLOW_THROUGH_MAX_RAMP_ANGLE_ERROR_DEG = 8.0;
  private static final double FLOW_THROUGH_MAX_BEND_ANGLE_DEG = 60.0;
  private static final double FLOW_THROUGH_MAX_ROUNDED_SEGMENT_ANGLE_DEG = 67.5;
  private static final double FLOW_THROUGH_MAX_ROUNDED_TURN_DEG = 45.0;
  private static final double FLOW_THROUGH_KICAD_CORNER_RADIUS_PERCENT = 80.0;
  private static final int FLOW_THROUGH_ROUNDED_RAMP_SAMPLES = 12;

  private final RoutingJob job;
  private final RoutingBoard board;
  private final double maxSkewBoard;
  private String lastSelectedCoupledCandidateFamily;

  public DifferentialPairAutorouter(RoutingJob p_job) {
    this(p_job, p_job.board, DEFAULT_MAX_SKEW_MM);
  }

  public DifferentialPairAutorouter(RoutingJob p_job, RoutingBoard p_board, double p_max_skew_mm) {
    this.job = p_job;
    this.board = p_board;
    this.maxSkewBoard = mm_to_board(p_board, p_max_skew_mm);
    this.lastSelectedCoupledCandidateFamily = null;
  }

  String last_selected_coupled_candidate_family_for_test() {
    return lastSelectedCoupledCandidateFamily;
  }

  /**
   * Runs differential-pair length matching for all parsed net-pair descriptors.
   *
   * @return number of accepted pair-length edits.
   */
  public int run() {
    if (board == null || board.rules == null) {
      return 0;
    }
    int acceptedEdits = 0;
    if (board.rules.differential_pairs != null) {
      List<DifferentialPair> pairs = new ArrayList<>();
      for (int i = 0; i < board.rules.differential_pairs.count(); i++) {
        pairs.add(board.rules.differential_pairs.get(i));
      }
      pairs.sort(Comparator.comparingDouble(this::pair_endpoint_span));
      for (DifferentialPair pair : pairs) {
        acceptedEdits += route_coupled_pair(pair);
        acceptedEdits += match_pair(pair);
      }
    }
    acceptedEdits += match_route_length_groups();
    if (acceptedEdits > 0) {
      logInfo("Post-route intent matching accepted " + acceptedEdits + " length adjustment"
          + (acceptedEdits == 1 ? "" : "s") + ".");
    }
    return acceptedEdits;
  }

  public int preRouteCoupledPairs() {
    if (board == null || board.rules == null || board.rules.differential_pairs == null) {
      return 0;
    }
    int acceptedRoutes = 0;
    List<DifferentialPair> pairs = new ArrayList<>();
    for (int i = 0; i < board.rules.differential_pairs.count(); i++) {
      pairs.add(board.rules.differential_pairs.get(i));
    }
    pairs.sort(Comparator.comparingDouble(this::pair_endpoint_span));
    for (DifferentialPair pair : pairs) {
      if (pair != null) {
        acceptedRoutes += pre_route_coupled_pair(pair);
      }
    }
    if (acceptedRoutes > 0) {
      logInfo("Pre-route intent reserved " + acceptedRoutes + " coupled differential-pair corridor"
          + (acceptedRoutes == 1 ? "." : "s."));
    }
    return acceptedRoutes;
  }

  private int match_route_length_groups() {
    if (job == null
        || job.routerSettings == null
        || job.routerSettings.intent == null
        || job.routerSettings.intent.routeLengthMatches == null) {
      return 0;
    }

    int acceptedEdits = 0;
    for (RouterIntentSettings.RouteLengthMatchIntent match : job.routerSettings.intent.routeLengthMatches) {
      acceptedEdits += match_route_length_group(match);
    }
    return acceptedEdits;
  }

  private int match_route_length_group(RouterIntentSettings.RouteLengthMatchIntent p_match) {
    if (p_match == null || p_match.nets == null || p_match.nets.length < 2) {
      return 0;
    }
    for (String netName : p_match.nets) {
      if (net_has_coupled_pair_intent(netName)) {
        logInfo("Route-length group " + route_length_match_name(p_match)
            + " skipped single-net length tuning because it contains coupled differential-pair net "
            + netName + ".");
        return 0;
      }
    }
    double maxSkew = mm_to_board(board, p_match.maxSkewMm == null ? DEFAULT_MAX_SKEW_MM : p_match.maxSkewMm);
    int acceptedEdits = 0;
    for (int attempt = 0; attempt < MAX_ATTEMPTS_PER_ROUTE_LENGTH_GROUP; attempt++) {
      List<PairMemberMeasurement> members = route_length_members(p_match);
      if (members.size() < 2) {
        return acceptedEdits;
      }
      PairMemberMeasurement shortest = members.get(0);
      PairMemberMeasurement longest = members.get(members.size() - 1);
      double skew = longest.length() - shortest.length();
      if (skew <= maxSkew) {
        if (acceptedEdits > 0) {
          logInfo(String.format(Locale.US,
              "Route-length group %s matched to %.3f mm skew.",
              route_length_match_name(p_match),
              board_to_mm(board, skew)));
        }
        return acceptedEdits;
      }

      double desiredExtra = skew - (maxSkew * TARGET_SKEW_FRACTION);
      if (desiredExtra <= 0 || !try_lengthen_net(shortest, desiredExtra, skew)) {
        logInfo(String.format(Locale.US,
            "Route-length group %s remains %.3f mm skewed; no DRC-clean lengthening candidate was found for %s.",
            route_length_match_name(p_match),
            board_to_mm(board, skew),
            shortest.netName()));
        return acceptedEdits;
      }
      acceptedEdits++;
    }
    return acceptedEdits;
  }

  private List<PairMemberMeasurement> route_length_members(RouterIntentSettings.RouteLengthMatchIntent p_match) {
    List<PairMemberMeasurement> result = new ArrayList<>();
    for (String netName : p_match.nets) {
      Net net = net_by_name(netName);
      if (net != null) {
        result.add(total_member_measurement(net));
      }
    }
    result.sort(Comparator.comparingDouble(PairMemberMeasurement::length));
    return result;
  }

  private Net net_by_name(String p_net_name) {
    if (p_net_name == null || board.rules == null || board.rules.nets == null) {
      return null;
    }
    return board.rules.nets.get(p_net_name, 1);
  }

  private String net_name(int p_net_no) {
    Net net = board.rules == null || board.rules.nets == null ? null : board.rules.nets.get(p_net_no);
    return net == null ? Integer.toString(p_net_no) : net.name;
  }

  private static String route_length_match_name(RouterIntentSettings.RouteLengthMatchIntent p_match) {
    return p_match.id == null || p_match.id.isBlank() ? "<unnamed>" : p_match.id;
  }

  private int route_coupled_pair(DifferentialPair p_pair) {
    RouterIntentSettings.DifferentialPairIntent intentPair = router_intent_pair(p_pair);
    if (intentPair == null) {
      logInfo("Differential-pair coupled routing skipped because no router-intent pair matched board rules.");
      return 0;
    }
    Net firstNet = board.rules.nets.get(p_pair.first_net_no());
    Net secondNet = board.rules.nets.get(p_pair.second_net_no());
    String pairName = pair_name(firstNet, secondNet);
    if (!Boolean.TRUE.equals(intentPair.routeAsCoupledPair)) {
      logInfo("Differential pair " + pairName + " skipped coupled routing because route_as_coupled_pair is not true.");
      return 0;
    }
    if (!p_pair.has_scoped_pins()) {
      logInfo("Differential pair " + pairName + " skipped coupled routing because scoped endpoint pins are missing.");
      return 0;
    }
    if (!allows_only_front_layer(intentPair)) {
      logInfo("Differential pair " + pairName + " skipped coupled routing because it is not constrained to F.Cu with zero vias.");
      return 0;
    }
    if (intentPair.targetGapMm == null || intentPair.targetGapMm < 0) {
      logInfo("Differential pair " + pairName + " skipped coupled routing because target_gap_mm is missing.");
      return 0;
    }

    if (firstNet == null || secondNet == null) {
      return 0;
    }

    PairMeasurements before = measure_pair(p_pair, firstNet, secondNet);
    if (!before.scoped()) {
      return insert_coupled_pair_when_scoped_paths_missing(p_pair, intentPair, firstNet, secondNet, pairName);
    }

    Pin firstFrom = find_pin(firstNet.net_number, p_pair.first_from_pin());
    Pin firstTo = find_pin(firstNet.net_number, p_pair.first_to_pin());
    Pin secondFrom = find_pin(secondNet.net_number, p_pair.second_from_pin());
    Pin secondTo = find_pin(secondNet.net_number, p_pair.second_to_pin());
    if (firstFrom == null || firstTo == null || secondFrom == null || secondTo == null) {
      logInfo("Differential pair " + pairName + " skipped coupled routing because endpoint pads could not be found.");
      return 0;
    }

    List<PolylineTrace> firstTraces = collect_unfixed_traces(before.first());
    List<PolylineTrace> secondTraces = collect_unfixed_traces(before.second());
    if (firstTraces.isEmpty() || secondTraces.isEmpty()) {
      logInfo("Differential pair " + pairName + " skipped coupled routing because scoped traces were not replaceable.");
      return 0;
    }
    List<PolylineTrace> firstReplacementTraces = collect_unfixed_net_traces(firstNet.net_number);
    List<PolylineTrace> secondReplacementTraces = collect_unfixed_net_traces(secondNet.net_number);
    if (firstReplacementTraces.isEmpty() || secondReplacementTraces.isEmpty()) {
      logInfo("Differential pair " + pairName + " skipped coupled routing because pair-net traces were not replaceable.");
      return 0;
    }

    int frontLayer = board_layer_index("F.Cu");
    if (frontLayer < 0) {
      logInfo("Differential pair " + pairName + " skipped coupled routing because F.Cu was not found in board layers.");
      return 0;
    }
    TraceStyle firstStyle = trace_style(firstTraces.get(0));
    TraceStyle secondStyle = trace_style(secondTraces.get(0));
    double targetCenterSpacing = mm_to_board(board, intentPair.targetGapMm)
        + firstStyle.halfWidth()
        + secondStyle.halfWidth();
    double minParallelLengthRatio = intentPair.minParallelLengthRatio == null ? 0.0 : intentPair.minParallelLengthRatio;
    double maxUncoupledLength = max_uncoupled_length(intentPair);
    double gapToleranceMm = intentPair.gapToleranceMm == null ? 0.0 : intentPair.gapToleranceMm;
    double maxPairGap = mm_to_board(
        board,
        intentPair.targetGapMm + gapToleranceMm);
    double minPairGap = mm_to_board(board, intentPair.targetGapMm);
    boolean requireParallelEvidence = Boolean.TRUE.equals(intentPair.requireParallelEvidence)
        || minParallelLengthRatio > 0.0
        || Double.isFinite(maxUncoupledLength);

    int baselineIncompletes = incomplete_count(board);
    int baselineClearanceViolations = clearance_violation_count(board);
    double baselineGap = nearest_gap_between_traces(firstTraces, secondTraces);
    double baselineParallelLength = parallel_length(firstTraces, secondTraces, targetCenterSpacing);
    double baselineReferenceLength = Math.max(before.first().length(), before.second().length());
    double baselineParallelRatio = baselineReferenceLength > 0.0 ? baselineParallelLength / baselineReferenceLength : 0.0;
    double baselineUncoupledLength = uncoupled_length(before, baselineParallelLength);
    double maxSkew = intentPair.maxSkewMm == null
        ? maxSkewBoard
        : mm_to_board(board, intentPair.maxSkewMm);
    double beforeSkew = Math.abs(before.first().length() - before.second().length());
    if (beforeSkew <= maxSkew
        && (Double.isNaN(baselineGap) || (baselineGap >= minPairGap && baselineGap <= maxPairGap))
        && (!requireParallelEvidence || baselineParallelLength > 0.0)
        && baselineParallelRatio >= minParallelLengthRatio
        && uncoupled_length_is_within_limit(baselineUncoupledLength, maxUncoupledLength)) {
      logInfo(String.format(Locale.US,
          "Differential pair %s kept existing coupled route because it already satisfies route intent: parallel ratio %.3f, uncoupled %.3f mm, skew %.3f mm, gap %.3f mm.",
          pairName,
          baselineParallelRatio,
          board_to_mm(board, baselineUncoupledLength),
          board_to_mm(board, beforeSkew),
          board_to_mm(board, baselineGap)));
      return 0;
    }

    List<CoupledCandidate> candidates = new ArrayList<>();
    for (double candidateCenterSpacing : coupled_candidate_center_spacings(targetCenterSpacing)) {
      candidates.addAll(coupled_candidates_with_pin_gateways(
          firstFrom,
          firstTo,
          secondFrom,
          secondTo,
          firstTraces,
          secondTraces,
          firstStyle,
          secondStyle,
          frontLayer,
          candidateCenterSpacing));
    }
    Map<String, Integer> generatedFamilies = coupled_candidate_family_counts(candidates);
    candidates = ranked_coupled_candidates(candidates, MAX_COUPLED_REPLACE_CANDIDATES, targetCenterSpacing);
    Map<String, Integer> evaluatedFamilies = coupled_candidate_family_counts(candidates);
    CoupledCandidateEvaluation bestCandidate = null;
    CoupledCandidateEvaluation bestRejectedCandidate = null;
    CoupledCandidateEvaluation bestRejectedAnyCandidate = null;
    Map<String, Integer> rejectionReasons = new HashMap<>();
    Map<String, Integer> rejectedFamilies = new HashMap<>();
    int evaluatedCandidateCount = 0;
    for (CoupledCandidate candidate : candidates) {
      evaluatedCandidateCount++;
      CoupledCandidateEvaluation evaluation = evaluate_replace_pair_routes(
          p_pair,
          before,
          firstReplacementTraces,
          secondReplacementTraces,
          candidate,
          frontLayer,
          firstStyle,
          secondStyle,
          baselineIncompletes,
          baselineClearanceViolations,
          baselineGap,
          baselineParallelLength,
          minParallelLengthRatio,
          maxUncoupledLength,
          minPairGap,
          maxPairGap,
          requireParallelEvidence,
          pairName,
          false);
      if (evaluation.accepted()
          && (bestCandidate == null || coupled_candidate_is_preferred(evaluation, bestCandidate))) {
        bestCandidate = evaluation;
      } else if (!evaluation.accepted()
          && evaluation.hasMeasurements()
          && (bestRejectedCandidate == null || evaluation.score() > bestRejectedCandidate.score())) {
        bestRejectedCandidate = evaluation;
      }
      if (!evaluation.accepted()) {
        rejectionReasons.merge(coupled_rejection_category(evaluation), 1, Integer::sum);
        rejectedFamilies.merge(candidate.family(), 1, Integer::sum);
        if (bestRejectedAnyCandidate == null || evaluation.score() > bestRejectedAnyCandidate.score()) {
          bestRejectedAnyCandidate = evaluation;
        }
      }
    }
    if (bestCandidate == null) {
      if (bestRejectedCandidate == null) {
        logInfo(String.format(Locale.US,
            "Differential pair %s/%s kept existing route; %d coupled top-layer candidates were evaluated, but none survived DRC and scope checks. Generated families: %s. Evaluated families: %s. Rejected families: %s. Rejections: %s%s",
            firstNet.name,
            secondNet.name,
            evaluatedCandidateCount,
            candidate_family_summary(generatedFamilies),
            candidate_family_summary(evaluatedFamilies),
            candidate_family_summary(rejectedFamilies),
            rejection_summary(rejectionReasons),
            bestRejectedAnyCandidate == null ? "" : "; best unmeasured: " + bestRejectedAnyCandidate.reason()));
      } else {
        logInfo(String.format(Locale.US,
            "Differential pair %s/%s kept existing route; best rejected coupled candidate among %d candidates was family %s with parallel ratio %.3f, skew %.3f mm, gap %.3f mm: %s. Generated families: %s. Evaluated families: %s. Rejected families: %s. Rejections: %s",
            firstNet.name,
            secondNet.name,
            evaluatedCandidateCount,
            bestRejectedCandidate.candidate().family(),
            bestRejectedCandidate.parallelRatio(),
            board_to_mm(board, bestRejectedCandidate.skew()),
            board_to_mm(board, bestRejectedCandidate.gap()),
            bestRejectedCandidate.reason(),
            candidate_family_summary(generatedFamilies),
            candidate_family_summary(evaluatedFamilies),
            candidate_family_summary(rejectedFamilies),
            rejection_summary(rejectionReasons)));
      }
      if (pair_route_satisfies_intent(
          before,
          baselineGap,
          baselineParallelLength,
          minParallelLengthRatio,
          maxUncoupledLength,
          minPairGap,
          maxPairGap,
          requireParallelEvidence)) {
        logInfo(String.format(Locale.US,
            "Differential pair %s/%s kept existing coupled route unchanged because it already satisfies route-quality intent.",
            firstNet.name,
            secondNet.name));
        return 0;
      }
      cleanup_existing_pair_accessories(p_pair, before, pairName);
      return 0;
    }

    CoupledCandidateEvaluation committed = evaluate_replace_pair_routes(
        p_pair,
        before,
        firstReplacementTraces,
        secondReplacementTraces,
        bestCandidate.candidate(),
        frontLayer,
        firstStyle,
        secondStyle,
        baselineIncompletes,
        baselineClearanceViolations,
        baselineGap,
          baselineParallelLength,
          minParallelLengthRatio,
          maxUncoupledLength,
          minPairGap,
        maxPairGap,
        requireParallelEvidence,
        pairName,
        true);
    if (committed.accepted()) {
      lastSelectedCoupledCandidateFamily = bestCandidate.candidate().family();
      logInfo(String.format(Locale.US,
          "Differential pair %s/%s rerouted as a coupled top-layer corridor from family %s: parallel ratio %.3f, skew %.3f mm, gap %.3f mm.",
          firstNet.name,
          secondNet.name,
          bestCandidate.candidate().family(),
          committed.parallelRatio(),
          board_to_mm(board, committed.skew()),
          board_to_mm(board, committed.gap())));
      return 1;
    }

    logInfo(String.format(Locale.US,
        "Differential pair %s/%s selected candidate could not be committed: %s.",
        firstNet.name,
        secondNet.name,
        committed.reason()));
    return 0;
  }

  private int insert_coupled_pair_when_scoped_paths_missing(
      DifferentialPair p_pair,
      RouterIntentSettings.DifferentialPairIntent p_intent_pair,
      Net p_first_net,
      Net p_second_net,
      String p_pair_name) {
    Pin firstFrom = find_pin(p_first_net.net_number, p_pair.first_from_pin());
    Pin firstTo = find_pin(p_first_net.net_number, p_pair.first_to_pin());
    Pin secondFrom = find_pin(p_second_net.net_number, p_pair.second_from_pin());
    Pin secondTo = find_pin(p_second_net.net_number, p_pair.second_to_pin());
    if (firstFrom == null || firstTo == null || secondFrom == null || secondTo == null) {
      logInfo("Differential pair " + p_pair_name + " skipped coupled routing because endpoint pads could not be found.");
      return 0;
    }
    int frontLayer = board_layer_index("F.Cu");
    if (frontLayer < 0) {
      logInfo("Differential pair " + p_pair_name + " skipped coupled routing because F.Cu was not found in board layers.");
      return 0;
    }
    TraceStyle firstStyle = trace_style(p_first_net, frontLayer);
    TraceStyle secondStyle = trace_style(p_second_net, frontLayer);
    if (firstStyle.halfWidth() <= 0 || secondStyle.halfWidth() <= 0) {
      logInfo("Differential pair " + p_pair_name + " skipped coupled routing because net-class trace widths were not usable.");
      return 0;
    }

    double targetCenterSpacing = mm_to_board(board, p_intent_pair.targetGapMm)
        + firstStyle.halfWidth()
        + secondStyle.halfWidth();
    double minParallelLengthRatio = p_intent_pair.minParallelLengthRatio == null ? 0.0 : p_intent_pair.minParallelLengthRatio;
    double maxUncoupledLength = max_uncoupled_length(p_intent_pair);
    double gapToleranceMm = p_intent_pair.gapToleranceMm == null ? 0.0 : p_intent_pair.gapToleranceMm;
    double maxPairGap = mm_to_board(board, p_intent_pair.targetGapMm + gapToleranceMm);
    double minPairGap = mm_to_board(board, p_intent_pair.targetGapMm);
    double maxSkew = p_intent_pair.maxSkewMm == null ? maxSkewBoard : mm_to_board(board, p_intent_pair.maxSkewMm);
    boolean requireParallelEvidence = Boolean.TRUE.equals(p_intent_pair.requireParallelEvidence)
        || minParallelLengthRatio > 0.0
        || Double.isFinite(maxUncoupledLength);
    int baselineIncompletes = incomplete_count(board);
    int baselineClearanceViolations = clearance_violation_count(board);
    TraceStyle firstEndpointEscapeStyle = endpoint_escape_style(p_intent_pair, firstStyle);
    TraceStyle secondEndpointEscapeStyle = endpoint_escape_style(p_intent_pair, secondStyle);
    double endpointEscapeLength = endpoint_escape_length(p_intent_pair);

    List<PolylineTrace> firstTraces = collect_pair_traces(total_member_measurement(p_first_net));
    List<PolylineTrace> secondTraces = collect_pair_traces(total_member_measurement(p_second_net));
    int missingMemberRepair = repair_missing_pair_member_from_companion(
        p_pair,
        p_first_net,
        p_second_net,
        firstFrom,
        firstTo,
        secondFrom,
        secondTo,
        firstStyle,
        secondStyle,
        firstTraces,
        secondTraces,
        firstEndpointEscapeStyle,
        secondEndpointEscapeStyle,
        endpointEscapeLength,
        frontLayer,
        targetCenterSpacing,
        baselineIncompletes,
        baselineClearanceViolations,
        minParallelLengthRatio,
        maxUncoupledLength,
        minPairGap,
        maxPairGap,
        requireParallelEvidence,
        p_pair_name);
    if (missingMemberRepair > 0) {
      return missingMemberRepair;
    }

    List<CoupledCandidate> candidates = new ArrayList<>();
    for (double candidateCenterSpacing : coupled_candidate_center_spacings(targetCenterSpacing)) {
      candidates.addAll(coupled_candidates_with_pin_gateways(
          firstFrom,
          firstTo,
          secondFrom,
          secondTo,
          firstTraces,
          secondTraces,
          firstStyle,
          secondStyle,
          frontLayer,
          candidateCenterSpacing));
    }
    Map<String, Integer> generatedFamilies = coupled_candidate_family_counts(candidates);
    candidates = ranked_coupled_candidates(candidates, MAX_COUPLED_REPLACE_CANDIDATES, targetCenterSpacing);
    Map<String, Integer> evaluatedFamilies = coupled_candidate_family_counts(candidates);
    CoupledCandidateEvaluation bestCandidate = null;
    CoupledCandidateEvaluation bestRejectedCandidate = null;
    Map<String, Integer> rejectionReasons = new HashMap<>();
    Map<String, Integer> rejectedFamilies = new HashMap<>();
    int evaluatedCandidateCount = 0;
    for (CoupledCandidate candidate : candidates) {
      evaluatedCandidateCount++;
      CoupledCandidateEvaluation evaluation = evaluate_insert_pair_routes(
          p_pair,
          candidate,
          frontLayer,
          firstStyle,
          secondStyle,
          baselineIncompletes,
          baselineClearanceViolations,
          maxSkew,
          minParallelLengthRatio,
          maxUncoupledLength,
          minPairGap,
          maxPairGap,
          requireParallelEvidence,
          false);
      if (evaluation.accepted()
          && (bestCandidate == null || evaluation.score() > bestCandidate.score())) {
        bestCandidate = evaluation;
      } else if (!evaluation.accepted()) {
        rejectionReasons.merge(coupled_rejection_category(evaluation), 1, Integer::sum);
        rejectedFamilies.merge(candidate.family(), 1, Integer::sum);
        if (bestRejectedCandidate == null
            || (evaluation.hasMeasurements()
                && (!bestRejectedCandidate.hasMeasurements() || evaluation.score() > bestRejectedCandidate.score()))
            || (!evaluation.hasMeasurements()
                && !bestRejectedCandidate.hasMeasurements()
                && evaluation.rejectionScore() > bestRejectedCandidate.rejectionScore())) {
          bestRejectedCandidate = evaluation;
        }
      }
    }
    if (bestCandidate == null) {
      logInfo(String.format(Locale.US,
          "Differential pair %s/%s could not repair missing scoped route; %d coupled top-layer candidates were evaluated. Best rejected: %s. Generated families: %s. Evaluated families: %s. Rejected families: %s. Rejections: %s",
          p_first_net.name,
          p_second_net.name,
          evaluatedCandidateCount,
          bestRejectedCandidate == null ? "<none>" : bestRejectedCandidate.reason(),
          candidate_family_summary(generatedFamilies),
          candidate_family_summary(evaluatedFamilies),
          candidate_family_summary(rejectedFamilies),
          rejection_summary(rejectionReasons)));
      return 0;
    }

    CoupledCandidateEvaluation committed = evaluate_insert_pair_routes(
        p_pair,
        bestCandidate.candidate(),
        frontLayer,
        firstStyle,
        secondStyle,
        baselineIncompletes,
        baselineClearanceViolations,
        maxSkew,
        minParallelLengthRatio,
        maxUncoupledLength,
        minPairGap,
        maxPairGap,
        requireParallelEvidence,
        true);
    if (!committed.accepted()) {
      logInfo(String.format(Locale.US,
          "Differential pair %s/%s selected missing-route repair candidate could not be committed: %s.",
          p_first_net.name,
          p_second_net.name,
          committed.reason()));
      return 0;
    }
    logInfo(String.format(Locale.US,
        "Differential pair %s/%s repaired missing scoped route with coupled top-layer corridor from family %s: parallel ratio %.3f, skew %.3f mm, gap %.3f mm.",
        p_first_net.name,
        p_second_net.name,
        bestCandidate.candidate().family(),
        committed.parallelRatio(),
        board_to_mm(board, committed.skew()),
        board_to_mm(board, committed.gap())));
    return 1;
  }

  private int repair_missing_pair_member_from_companion(
      DifferentialPair p_pair,
      Net p_first_net,
      Net p_second_net,
      Pin p_first_from,
      Pin p_first_to,
      Pin p_second_from,
      Pin p_second_to,
      TraceStyle p_first_style,
      TraceStyle p_second_style,
      List<PolylineTrace> p_first_traces,
      List<PolylineTrace> p_second_traces,
      TraceStyle p_first_endpoint_escape_style,
      TraceStyle p_second_endpoint_escape_style,
      double p_endpoint_escape_length,
      int p_layer,
      double p_target_center_spacing,
      int p_max_incompletes,
      int p_max_clearance_violations,
      double p_min_parallel_length_ratio,
      double p_max_uncoupled_length,
      double p_min_pair_gap,
      double p_max_pair_gap,
      boolean p_require_parallel_evidence,
      String p_pair_name) {
    boolean firstMissing = p_first_traces.isEmpty() && !p_second_traces.isEmpty();
    boolean secondMissing = p_second_traces.isEmpty() && !p_first_traces.isEmpty();
    if (!firstMissing && !secondMissing) {
      return 0;
    }

    Pin missingFrom = firstMissing ? p_first_from : p_second_from;
    Pin missingTo = firstMissing ? p_first_to : p_second_to;
    TraceStyle missingStyle = firstMissing ? p_first_style : p_second_style;
    TraceStyle missingEndpointEscapeStyle = firstMissing ? p_first_endpoint_escape_style : p_second_endpoint_escape_style;
    int missingNetNo = firstMissing ? p_pair.first_net_no() : p_pair.second_net_no();
    List<PolylineTrace> companionTraces = firstMissing ? p_second_traces : p_first_traces;

    List<MissingMemberCandidate> candidates = new ArrayList<>();
    for (double candidateCenterSpacing : coupled_candidate_center_spacings(p_target_center_spacing)) {
      candidates.addAll(missing_member_candidates_from_companion_trace(
          missingFrom,
          missingTo,
          companionTraces,
          p_layer,
          candidateCenterSpacing));
    }

    MissingMemberCandidateEvaluation bestCandidate = null;
    MissingMemberCandidateEvaluation bestRejectedCandidate = null;
    int evaluatedCandidateCount = 0;
    for (MissingMemberCandidate candidate : candidates) {
      evaluatedCandidateCount++;
      MissingMemberCandidateEvaluation evaluation = evaluate_insert_missing_pair_member(
          p_pair,
          candidate,
          p_layer,
          missingStyle,
          missingEndpointEscapeStyle,
          p_endpoint_escape_length,
          missingNetNo,
          p_first_net,
          p_second_net,
          p_max_incompletes,
          p_max_clearance_violations,
          p_min_parallel_length_ratio,
          p_max_uncoupled_length,
          p_min_pair_gap,
          p_max_pair_gap,
          p_require_parallel_evidence,
          false);
      if (evaluation.accepted()
          && (bestCandidate == null || evaluation.score() > bestCandidate.score())) {
        bestCandidate = evaluation;
      } else if (!evaluation.accepted()
          && (bestRejectedCandidate == null
          || evaluation.rejectionScore() > bestRejectedCandidate.rejectionScore())) {
        bestRejectedCandidate = evaluation;
      }
    }
    if (bestCandidate == null) {
      if (evaluatedCandidateCount > 0) {
        logInfo(String.format(Locale.US,
            "Differential pair %s missing-member companion repair evaluated %d candidates but found no legal route. Best rejected: %s.",
            p_pair_name,
            evaluatedCandidateCount,
            bestRejectedCandidate == null ? "<none>" : bestRejectedCandidate.reason()));
      }
      return 0;
    }

    MissingMemberCandidateEvaluation committed = evaluate_insert_missing_pair_member(
        p_pair,
        bestCandidate.candidate(),
        p_layer,
        missingStyle,
        missingEndpointEscapeStyle,
        p_endpoint_escape_length,
        missingNetNo,
        p_first_net,
        p_second_net,
        p_max_incompletes,
        p_max_clearance_violations,
        p_min_parallel_length_ratio,
        p_max_uncoupled_length,
        p_min_pair_gap,
        p_max_pair_gap,
        p_require_parallel_evidence,
        true);
    if (!committed.accepted()) {
      logInfo(String.format(Locale.US,
          "Differential pair %s selected missing-member companion repair could not be committed: %s.",
          p_pair_name,
          committed.reason()));
      return 0;
    }
    logInfo(String.format(Locale.US,
        "Differential pair %s repaired missing %s member from existing companion trace family %s: parallel ratio %.3f, skew %.3f mm, gap %.3f mm.",
        p_pair_name,
        firstMissing ? p_first_net.name : p_second_net.name,
        bestCandidate.candidate().family(),
        committed.parallelRatio(),
        board_to_mm(board, committed.skew()),
        board_to_mm(board, committed.gap())));
    return 1;
  }

  private int cleanup_existing_pair_accessories(
      DifferentialPair p_pair,
      PairMeasurements p_measurements,
      String p_pair_name) {
    List<PolylineTrace> protectedTraces = new ArrayList<>();
    List<PolylineTrace> firstTraces = collect_unfixed_traces(p_measurements.first());
    List<PolylineTrace> secondTraces = collect_unfixed_traces(p_measurements.second());
    List<PolylineTrace> firstCorridorTraces = coupled_corridor_traces(firstTraces, secondTraces);
    List<PolylineTrace> secondCorridorTraces = coupled_corridor_traces(secondTraces, firstTraces);
    protectedTraces.addAll(firstCorridorTraces);
    protectedTraces.addAll(secondCorridorTraces);
    if (protectedTraces.isEmpty()) {
      return 0;
    }
    int reconnected = reconnect_net_pins_to_corridor(
        p_pair.first_net_no(),
        corridor_points(firstCorridorTraces),
        firstCorridorTraces)
        + reconnect_net_pins_to_corridor(
            p_pair.second_net_no(),
            corridor_points(secondCorridorTraces),
            secondCorridorTraces);
    Set<Integer> protectedTraceIds = trace_id_set(protectedTraces);
    int removed = prune_endpoint_pair_branches(p_pair.first_net_no(), protectedTraceIds)
        + prune_endpoint_pair_branches(p_pair.second_net_no(), protectedTraceIds)
        + prune_redundant_net_traces(p_pair.first_net_no(), protectedTraceIds)
        + prune_redundant_net_traces(p_pair.second_net_no(), protectedTraceIds)
        + prune_redundant_net_components(p_pair.first_net_no(), protectedTraceIds)
        + prune_redundant_net_components(p_pair.second_net_no(), protectedTraceIds);
    if (reconnected > 0 || removed > 0) {
      logInfo(String.format(Locale.US,
          "Differential pair %s reconnected %d pin corridor contact%s and removed %d redundant accessory item%s from the existing coupled route.",
          p_pair_name,
          reconnected,
          reconnected == 1 ? "" : "s",
          removed,
          removed == 1 ? "" : "s"));
    }
    return removed;
  }

  private int prune_endpoint_pair_branches(int p_net_no, Set<Integer> p_protected_trace_ids) {
    int removed = 0;
    boolean progressed = true;
    while (progressed) {
      progressed = false;
      Set<Item> visited = new HashSet<>();
      for (Item item : new ArrayList<>(board.get_connectable_items(p_net_no))) {
        if (visited.contains(item) || !is_prunable_pair_accessory_item(item, p_net_no, p_protected_trace_ids)) {
          continue;
        }
        Set<Item> component = connected_pair_accessory_component(item, p_net_no, p_protected_trace_ids);
        visited.addAll(component);
        double componentLength = component_trace_length(component);
        if (component.isEmpty()
            || !component_touches_pin(component, p_net_no)
            || componentLength < mm_to_board(board, 1.5)
            || componentLength > mm_to_board(board, 10.0)) {
          continue;
        }
        int beforeIncompletes = incomplete_count_for_net(p_net_no);
        int beforeClearance = clearance_violation_count(board);
        int beforeUnconnected = unconnected_item_count_for_net(p_net_no);
        board.generate_snapshot();
        if (!board.remove_items(component)) {
          board.undo(null);
          continue;
        }
        board.combine_traces(p_net_no);
        int afterClearance = clearance_violation_count(board);
        int afterUnconnected = unconnected_item_count_for_net(p_net_no);
        if (afterClearance <= beforeClearance && afterUnconnected <= beforeUnconnected) {
          board.pop_snapshot();
          removed += component.size();
          logInfo(String.format(Locale.US,
              "Differential pair cleanup removed %d endpoint branch item%s from net %s despite freerouting incompletes %d->%d because the branch touched a protected coupled corridor and endpoint pin.",
              component.size(),
              component.size() == 1 ? "" : "s",
              net_name(p_net_no),
              beforeIncompletes,
              incomplete_count_for_net(p_net_no)));
          progressed = true;
          break;
        }
        board.undo(null);
      }
    }
    return removed;
  }

  private static boolean component_touches_pin(Set<Item> p_component, int p_net_no) {
    for (Item item : p_component) {
      for (Item contact : item.get_normal_contacts()) {
        if (contact instanceof Pin && contact.contains_net(p_net_no)) {
          return true;
        }
      }
    }
    return false;
  }

  private static double component_trace_length(Set<Item> p_component) {
    double result = 0.0;
    for (Item item : p_component) {
      if (item instanceof Trace trace) {
        result += trace.get_length();
      }
    }
    return result;
  }

  private static Point[] corridor_points(List<PolylineTrace> p_traces) {
    List<Point> points = new ArrayList<>();
    for (PolylineTrace trace : p_traces) {
      for (FloatPoint point : trace.polyline().corner_approx_arr()) {
        append_distinct(points, point.round());
      }
    }
    return points.toArray(Point[]::new);
  }

  private List<PolylineTrace> coupled_corridor_traces(
      List<PolylineTrace> p_traces,
      List<PolylineTrace> p_mate_traces) {
    List<PolylineTrace> result = new ArrayList<>();
    double maxGap = mm_to_board(board, 0.80);
    for (PolylineTrace trace : p_traces) {
      double bestCoupled = 0.0;
      for (PolylineTrace mate : p_mate_traces) {
        if (trace.get_layer() != mate.get_layer()) {
          continue;
        }
        bestCoupled = Math.max(bestCoupled, parallel_length(trace, mate, maxGap));
      }
      if (bestCoupled >= Math.min(trace.get_length() * 0.50, mm_to_board(board, 5.0))) {
        result.add(trace);
      }
    }
    return result;
  }

  private int pre_route_coupled_pair(DifferentialPair p_pair) {
    RouterIntentSettings.DifferentialPairIntent intentPair = router_intent_pair(p_pair);
    if (intentPair == null
        || !Boolean.TRUE.equals(intentPair.routeAsCoupledPair)
        || !p_pair.has_scoped_pins()
        || !allows_only_front_layer(intentPair)
        || intentPair.targetGapMm == null
        || intentPair.targetGapMm < 0) {
      return 0;
    }

    Net firstNet = board.rules.nets.get(p_pair.first_net_no());
    Net secondNet = board.rules.nets.get(p_pair.second_net_no());
    if (firstNet == null || secondNet == null) {
      return 0;
    }
    String pairName = pair_name(firstNet, secondNet);

    Pin firstFrom = find_pin(firstNet.net_number, p_pair.first_from_pin());
    Pin firstTo = find_pin(firstNet.net_number, p_pair.first_to_pin());
    Pin secondFrom = find_pin(secondNet.net_number, p_pair.second_from_pin());
    Pin secondTo = find_pin(secondNet.net_number, p_pair.second_to_pin());
    if (firstFrom == null || firstTo == null || secondFrom == null || secondTo == null) {
      logInfo("Differential pair " + pairName + " skipped pre-route corridor because endpoint pads could not be found.");
      return 0;
    }

    int frontLayer = board_layer_index("F.Cu");
    if (frontLayer < 0) {
      logInfo("Differential pair " + pairName + " skipped pre-route corridor because F.Cu was not found in board layers.");
      return 0;
    }

    TraceStyle firstStyle = trace_style(firstNet, frontLayer);
    TraceStyle secondStyle = trace_style(secondNet, frontLayer);
    if (firstStyle.halfWidth() <= 0 || secondStyle.halfWidth() <= 0) {
      logInfo("Differential pair " + pairName + " skipped pre-route corridor because net-class trace widths were not usable.");
      return 0;
    }

    double targetCenterSpacing = mm_to_board(board, intentPair.targetGapMm)
        + firstStyle.halfWidth()
        + secondStyle.halfWidth();
    double minParallelLengthRatio = intentPair.minParallelLengthRatio == null ? 0.0 : intentPair.minParallelLengthRatio;
    double maxUncoupledLength = max_uncoupled_length(intentPair);
    double gapToleranceMm = intentPair.gapToleranceMm == null ? 0.0 : intentPair.gapToleranceMm;
    double maxPairGap = mm_to_board(
        board,
        intentPair.targetGapMm + gapToleranceMm);
    double minPairGap = mm_to_board(board, intentPair.targetGapMm);
    double maxSkew = intentPair.maxSkewMm == null ? maxSkewBoard : mm_to_board(board, intentPair.maxSkewMm);
    boolean requireParallelEvidence = Boolean.TRUE.equals(intentPair.requireParallelEvidence)
        || minParallelLengthRatio > 0.0
        || Double.isFinite(maxUncoupledLength);
    int baselineIncompletes = incomplete_count(board);
    int baselineClearanceViolations = clearance_violation_count(board);

    List<CoupledCandidate> candidates = new ArrayList<>();
    for (double candidateCenterSpacing : coupled_candidate_center_spacings(targetCenterSpacing)) {
      candidates.addAll(coupled_candidates_with_pin_gateways(
          firstFrom,
          firstTo,
          secondFrom,
          secondTo,
          List.of(),
          List.of(),
          firstStyle,
          secondStyle,
          frontLayer,
          candidateCenterSpacing));
    }
    Map<String, Integer> generatedFamilies = coupled_candidate_family_counts(candidates);
    candidates = ranked_coupled_candidates(candidates, MAX_COUPLED_PREROUTE_CANDIDATES, targetCenterSpacing);
    Map<String, Integer> evaluatedFamilies = coupled_candidate_family_counts(candidates);

    CoupledCandidateEvaluation bestCandidate = null;
    CoupledCandidateEvaluation bestRejectedCandidate = null;
    Map<String, Integer> rejectionReasons = new HashMap<>();
    Map<String, Integer> rejectedFamilies = new HashMap<>();
    int evaluatedCandidateCount = 0;
    for (CoupledCandidate candidate : candidates) {
      evaluatedCandidateCount++;
      CoupledCandidateEvaluation evaluation = evaluate_insert_pair_routes(
          p_pair,
          candidate,
          frontLayer,
          firstStyle,
          secondStyle,
          baselineIncompletes,
          baselineClearanceViolations,
          maxSkew,
          minParallelLengthRatio,
          maxUncoupledLength,
          minPairGap,
          maxPairGap,
          requireParallelEvidence,
          false);
      if (evaluation.accepted()
          && (bestCandidate == null || coupled_candidate_is_preferred(evaluation, bestCandidate))) {
        bestCandidate = evaluation;
      } else if (!evaluation.accepted()) {
        rejectionReasons.merge(coupled_rejection_category(evaluation), 1, Integer::sum);
        rejectedFamilies.merge(candidate.family(), 1, Integer::sum);
        if (bestRejectedCandidate == null
            || (evaluation.hasMeasurements()
                && (!bestRejectedCandidate.hasMeasurements() || evaluation.score() > bestRejectedCandidate.score()))
            || (!evaluation.hasMeasurements()
                && !bestRejectedCandidate.hasMeasurements()
                && evaluation.rejectionScore() > bestRejectedCandidate.rejectionScore())) {
          bestRejectedCandidate = evaluation;
        }
      }
    }

    if (bestCandidate == null) {
      if (bestRejectedCandidate == null) {
        logInfo(String.format(Locale.US,
            "Differential pair %s/%s did not reserve a pre-route corridor; %d coupled top-layer candidates were evaluated, but none survived DRC and scope checks. Generated families: %s. Evaluated families: %s. Rejected families: %s. Rejections: %s",
            firstNet.name,
            secondNet.name,
            evaluatedCandidateCount,
            candidate_family_summary(generatedFamilies),
            candidate_family_summary(evaluatedFamilies),
            candidate_family_summary(rejectedFamilies),
            rejection_summary(rejectionReasons)));
      } else {
        logInfo(String.format(Locale.US,
            "Differential pair %s/%s did not reserve a pre-route corridor; best rejected candidate among %d candidates was family %s with parallel ratio %.3f, skew %.3f mm, gap %.3f mm: %s. Generated families: %s. Evaluated families: %s. Rejected families: %s. Rejections: %s",
            firstNet.name,
            secondNet.name,
            evaluatedCandidateCount,
            bestRejectedCandidate.candidate().family(),
            bestRejectedCandidate.parallelRatio(),
            board_to_mm(board, bestRejectedCandidate.skew()),
            board_to_mm(board, bestRejectedCandidate.gap()),
            bestRejectedCandidate.reason(),
            candidate_family_summary(generatedFamilies),
            candidate_family_summary(evaluatedFamilies),
            candidate_family_summary(rejectedFamilies),
            rejection_summary(rejectionReasons)));
      }
      return 0;
    }

    CoupledCandidateEvaluation committed = evaluate_insert_pair_routes(
        p_pair,
        bestCandidate.candidate(),
        frontLayer,
        firstStyle,
        secondStyle,
        baselineIncompletes,
        baselineClearanceViolations,
        maxSkew,
        minParallelLengthRatio,
        maxUncoupledLength,
        minPairGap,
        maxPairGap,
        requireParallelEvidence,
        true);
    if (!committed.accepted()) {
      logInfo(String.format(Locale.US,
          "Differential pair %s/%s selected pre-route corridor could not be committed: %s.",
          firstNet.name,
          secondNet.name,
          committed.reason()));
      return 0;
    }

    logInfo(String.format(Locale.US,
        "Differential pair %s/%s reserved as a coupled top-layer corridor from family %s before general routing: parallel ratio %.3f, skew %.3f mm, gap %.3f mm. Generated families: %s. Evaluated families: %s. Rejected families: %s. Rejections: %s",
        firstNet.name,
        secondNet.name,
        bestCandidate.candidate().family(),
        committed.parallelRatio(),
        board_to_mm(board, committed.skew()),
        board_to_mm(board, committed.gap()),
        candidate_family_summary(generatedFamilies),
        candidate_family_summary(evaluatedFamilies),
        candidate_family_summary(rejectedFamilies),
        rejection_summary(rejectionReasons)));
    lastSelectedCoupledCandidateFamily = bestCandidate.candidate().family();
    return 1;
  }

  private static String coupled_rejection_category(CoupledCandidateEvaluation p_evaluation) {
    String reason = p_evaluation.reason();
    if (reason == null || reason.isBlank()) {
      return "unknown";
    }
    if (reason.contains("clearance violations")) {
      return reason.replaceFirst("^.*clearance violations ([0-9]+>[0-9]+).*$", "clearance violations $1");
    }
    if (reason.contains("incompletes")) {
      return reason.replaceFirst("^.*incompletes ([0-9]+>[0-9]+).*$", "incompletes $1");
    }
    int comma = reason.indexOf(',');
    int semicolon = reason.indexOf(';');
    int end = reason.length();
    if (comma >= 0) {
      end = Math.min(end, comma);
    }
    if (semicolon >= 0) {
      end = Math.min(end, semicolon);
    }
    return reason.substring(0, end).trim();
  }

  private static String rejection_summary(Map<String, Integer> p_reasons) {
    if (p_reasons.isEmpty()) {
      return "<none>";
    }
    List<Map.Entry<String, Integer>> entries = new ArrayList<>(p_reasons.entrySet());
    entries.sort(Map.Entry.<String, Integer>comparingByValue().reversed().thenComparing(Map.Entry::getKey));
    List<String> parts = new ArrayList<>();
    for (Map.Entry<String, Integer> entry : entries) {
      parts.add(entry.getKey() + "=" + entry.getValue());
    }
    return String.join(", ", parts);
  }

  private static Map<String, Integer> coupled_candidate_family_counts(List<CoupledCandidate> p_candidates) {
    Map<String, Integer> result = new HashMap<>();
    for (CoupledCandidate candidate : p_candidates) {
      result.merge(candidate.family(), 1, Integer::sum);
    }
    return result;
  }

  private static boolean coupled_candidate_is_preferred(
      CoupledCandidateEvaluation p_candidate,
      CoupledCandidateEvaluation p_current_best) {
    boolean candidateFlowThrough = p_candidate.candidate().family().startsWith("flow_through_skew_compensated");
    boolean currentFlowThrough = p_current_best.candidate().family().startsWith("flow_through_skew_compensated");
    if (candidateFlowThrough != currentFlowThrough) {
      return candidateFlowThrough;
    }
    return p_candidate.score() > p_current_best.score();
  }

  private static String candidate_family_summary(Map<String, Integer> p_families) {
    if (p_families.isEmpty()) {
      return "<none>";
    }
    List<Map.Entry<String, Integer>> entries = new ArrayList<>(p_families.entrySet());
    entries.sort(Map.Entry.<String, Integer>comparingByValue().reversed().thenComparing(Map.Entry::getKey));
    List<String> parts = new ArrayList<>();
    for (Map.Entry<String, Integer> entry : entries) {
      parts.add(entry.getKey() + "=" + entry.getValue());
    }
    return String.join(", ", parts);
  }

  private int match_pair(DifferentialPair p_pair) {
    RouterIntentSettings.DifferentialPairIntent intentPair = router_intent_pair(p_pair);
    Net firstNet = board.rules.nets.get(p_pair.first_net_no());
    Net secondNet = board.rules.nets.get(p_pair.second_net_no());
    if (firstNet == null || secondNet == null) {
      return 0;
    }
    if (intentPair != null && Boolean.TRUE.equals(intentPair.routeAsCoupledPair)) {
      return match_coupled_pair_with_meanders(p_pair, intentPair, firstNet, secondNet);
    }

    int acceptedEdits = 0;
    for (int attempt = 0; attempt < MAX_ATTEMPTS_PER_PAIR; attempt++) {
      PairMeasurements measurements = measure_pair(p_pair, firstNet, secondNet);
      double firstLength = measurements.first().length();
      double secondLength = measurements.second().length();
      double skew = Math.abs(firstLength - secondLength);
      if (skew <= maxSkewBoard) {
        if (acceptedEdits > 0) {
          logInfo(String.format(Locale.US,
              "Differential pair %s/%s matched to %.3f mm %sskew.",
              firstNet.name, secondNet.name, board_to_mm(board, skew),
              measurements.scoped() ? "scoped-path " : ""));
        }
        return acceptedEdits;
      }

      PairMemberMeasurement shorterMember = firstLength < secondLength ? measurements.first() : measurements.second();
      double desiredExtra = skew - (maxSkewBoard * TARGET_SKEW_FRACTION);
      if (desiredExtra <= 0) {
        return acceptedEdits;
      }

      if (!try_lengthen_net(shorterMember, desiredExtra, skew)) {
        logInfo(String.format(Locale.US,
            "Differential pair %s/%s remains %.3f mm %sskewed; no DRC-clean lengthening candidate was found.",
            firstNet.name, secondNet.name, board_to_mm(board, skew),
            measurements.scoped() ? "scoped-path " : ""));
        return acceptedEdits;
      }
      acceptedEdits++;
    }
    return acceptedEdits;
  }

  private int match_coupled_pair_with_meanders(
      DifferentialPair p_pair,
      RouterIntentSettings.DifferentialPairIntent p_intent_pair,
      Net p_first_net,
      Net p_second_net) {
    double maxSkew = p_intent_pair != null && p_intent_pair.maxSkewMm != null
        ? mm_to_board(board, p_intent_pair.maxSkewMm)
        : maxSkewBoard;
    int acceptedEdits = 0;
    for (int attempt = 0; attempt < MAX_ATTEMPTS_PER_PAIR; attempt++) {
      PairMeasurements measurements = measure_pair(p_pair, p_first_net, p_second_net);
      if (measurements == null || measurements.first() == null || measurements.second() == null) {
        return acceptedEdits;
      }
      double firstLength = measurements.first().length();
      double secondLength = measurements.second().length();
      double skew = Math.abs(firstLength - secondLength);
      if (skew <= maxSkew) {
        if (acceptedEdits > 0) {
          logInfo(String.format(Locale.US,
              "Differential pair %s/%s tuned with pair-aware meanders to %.3f mm %sskew.",
              p_first_net.name,
              p_second_net.name,
              board_to_mm(board, skew),
              measurements.scoped() ? "scoped-path " : ""));
        }
        return acceptedEdits;
      }

      PairMemberMeasurement shorterMember = firstLength < secondLength ? measurements.first() : measurements.second();
      PairMemberMeasurement longerMember = firstLength < secondLength ? measurements.second() : measurements.first();
      double desiredExtra = skew - (maxSkew * TARGET_SKEW_FRACTION);
      if (desiredExtra <= 0) {
        return acceptedEdits;
      }
      if (!try_meander_shorter_pair_member(
          p_pair,
          p_first_net,
          p_second_net,
          shorterMember,
          longerMember,
          desiredExtra,
          skew,
          maxSkew)) {
        logInfo(String.format(Locale.US,
            "Differential pair %s/%s remains %.3f mm %sskewed; no DRC-clean pair-aware meander candidate was found.",
            p_first_net.name,
            p_second_net.name,
            board_to_mm(board, skew),
            measurements.scoped() ? "scoped-path " : ""));
        return acceptedEdits;
      }
      acceptedEdits++;
    }
    return acceptedEdits;
  }

  private boolean try_meander_shorter_pair_member(
      DifferentialPair p_pair,
      Net p_first_net,
      Net p_second_net,
      PairMemberMeasurement p_shorter_member,
      PairMemberMeasurement p_longer_member,
      double p_desired_extra,
      double p_current_skew,
      double p_max_skew) {
    int baselineIncompletes = incomplete_count(board);
    int baselineClearanceViolations = clearance_violation_count(board);
    List<PolylineTrace> traces = collect_pair_traces(p_shorter_member);
    List<PolylineTrace> companionTraces = collect_pair_traces(p_longer_member);
    traces.removeIf(Trace::is_user_fixed);
    traces.sort(Comparator.comparingDouble(DifferentialPairAutorouter::longest_segment_length).reversed());

    int checkedCandidates = 0;
    for (PolylineTrace trace : traces) {
      List<PairMeanderCandidate> candidates = build_pair_meander_candidates(trace, companionTraces, p_desired_extra);
      for (PairMeanderCandidate candidate : candidates) {
        if (++checkedCandidates > MAX_CANDIDATES_PER_PASS) {
          return false;
        }
        double afterSkew = try_replace_pair_member_trace(
            p_pair,
            p_first_net,
            p_second_net,
            trace,
            candidate.points(),
            baselineIncompletes,
            baselineClearanceViolations,
            p_current_skew,
            p_max_skew);
        if (Double.isFinite(afterSkew)) {
          logInfo(String.format(Locale.US,
              "Differential pair %s/%s lengthened shorter member %s with a pair-aware meander; skew %.3f mm -> %.3f mm.",
              p_first_net.name,
              p_second_net.name,
              p_shorter_member.netName(),
              board_to_mm(board, p_current_skew),
              board_to_mm(board, afterSkew)));
          return true;
        }
      }
    }
    return false;
  }

  private boolean try_lengthen_net(PairMemberMeasurement p_member, double p_desired_extra, double p_current_skew) {
    int baselineIncompletes = incomplete_count(board);
    int baselineClearanceViolations = clearance_violation_count(board);
    List<PolylineTrace> traces = collect_unfixed_traces(p_member);
    traces.sort(Comparator.comparingDouble(DifferentialPairAutorouter::longest_segment_length).reversed());

    int checkedCandidates = 0;
    for (PolylineTrace trace : traces) {
      List<Point[]> candidates = build_detour_candidates(trace, p_desired_extra);
      for (Point[] candidate : candidates) {
        if (++checkedCandidates > MAX_CANDIDATES_PER_PASS) {
          return false;
        }
        double beforeLength = measurement_length(p_member);
        double afterLength = try_replace_trace(
            trace,
            candidate,
            baselineIncompletes,
            baselineClearanceViolations,
            p_member,
            beforeLength);
        if (Double.isFinite(afterLength)) {
          double added = afterLength - beforeLength;
          logInfo(String.format(Locale.US,
              "Differential-pair matcher lengthened net %s %sby %.3f mm toward %.3f mm skew.",
              p_member.netName(), p_member.scoped() ? "scoped path " : "",
              board_to_mm(board, added), board_to_mm(board, p_current_skew)));
          return true;
        }
      }
    }
    return false;
  }

  private CoupledCandidateEvaluation evaluate_replace_pair_routes(
      DifferentialPair p_pair,
      PairMeasurements p_before,
      List<PolylineTrace> p_first_traces,
      List<PolylineTrace> p_second_traces,
      CoupledCandidate p_candidate,
      int p_layer,
      TraceStyle p_first_style,
      TraceStyle p_second_style,
      int p_max_incompletes,
      int p_max_clearance_violations,
      double p_baseline_gap,
      double p_baseline_parallel_length,
      double p_min_parallel_length_ratio,
      double p_max_uncoupled_length,
      double p_min_pair_gap,
      double p_max_pair_gap,
      boolean p_require_parallel_evidence,
      String p_pair_name,
      boolean p_commit) {
    board.generate_snapshot();
    Set<Item> removeItems = new HashSet<>();
    removeItems.addAll(p_first_traces);
    removeItems.addAll(p_second_traces);
    if (!board.remove_items(removeItems)) {
      board.undo(null);
      return CoupledCandidateEvaluation.rejected(p_candidate, "existing traces could not be removed");
    }

    insert_exact_coupled_trace(
        p_candidate.first(),
        p_layer,
        p_first_style,
        p_pair.first_net_no());

    insert_exact_coupled_trace(
        p_candidate.second(),
        p_layer,
        p_second_style,
        p_pair.second_net_no());

    board.combine_traces(p_pair.first_net_no());
    board.combine_traces(p_pair.second_net_no());

    DesignRulesChecker incompleteChecker = new DesignRulesChecker(board, null);
    incompleteChecker.calculateAllIncompletes();
    int afterIncompletes = incompleteChecker.getIncompleteCount();
    int afterPairIncompletes = incompleteChecker.getIncompleteCount(p_pair.first_net_no())
        + incompleteChecker.getIncompleteCount(p_pair.second_net_no());
    int afterOutsidePairIncompletes = Math.max(0, afterIncompletes - afterPairIncompletes);
    ClearanceCheck clearanceCheck = clearance_check_excluding_pair(board, p_pair, 4);
    int afterClearanceViolations = clearanceCheck.count();
    if (afterOutsidePairIncompletes > p_max_incompletes || afterClearanceViolations > p_max_clearance_violations) {
      String reason = String.format(Locale.US,
          "outside-pair incompletes %d>%d, pair incompletes %d, clearance violations %d>%d%s",
          afterOutsidePairIncompletes,
          p_max_incompletes,
          afterPairIncompletes,
          afterClearanceViolations,
          p_max_clearance_violations,
          clearanceCheck.summary().isEmpty() ? "" : "; " + clearanceCheck.summary());
      board.undo(null);
      return CoupledCandidateEvaluation.rejected(p_candidate, reason, afterIncompletes, afterClearanceViolations);
    }
    if (afterIncompletes > p_max_incompletes) {
      String reason = String.format(Locale.US,
          "pair incompletes %d with outside-pair incompletes %d<=%d, clearance violations %d>%d%s",
          afterPairIncompletes,
          afterOutsidePairIncompletes,
          p_max_incompletes,
          afterClearanceViolations,
          p_max_clearance_violations,
          clearanceCheck.summary().isEmpty() ? "" : "; " + clearanceCheck.summary());
      FRLogger.debug("Differential pair " + p_pair_name + " replacement has pair-local incompletes before scoped measurement: " + reason);
    }

    Net firstNet = board.rules.nets.get(p_pair.first_net_no());
    Net secondNet = board.rules.nets.get(p_pair.second_net_no());
    if (firstNet == null || secondNet == null) {
      board.undo(null);
      return CoupledCandidateEvaluation.rejected(p_candidate, "pair nets were not found");
    }
    PairMeasurements after = measure_pair(p_pair, firstNet, secondNet);
    double skew = Math.abs(after.first().length() - after.second().length());
    List<PolylineTrace> afterFirstTraces = collect_unfixed_traces(after.first());
    List<PolylineTrace> afterSecondTraces = collect_unfixed_traces(after.second());
    double afterGap = nearest_gap_between_traces(afterFirstTraces, afterSecondTraces);
    double afterParallelLength = parallel_length(
        afterFirstTraces,
        afterSecondTraces,
        p_candidate.centerSpacing());
    double afterReferenceLength = Math.max(after.first().length(), after.second().length());
    double afterParallelRatio = afterReferenceLength > 0.0 ? afterParallelLength / afterReferenceLength : 0.0;
    double afterUncoupledLength = uncoupled_length(after, afterParallelLength);
    double beforeSkew = Math.abs(p_before.first().length() - p_before.second().length());
    boolean hardSafe = after.scoped()
        && skew <= Math.max(maxSkewBoard, beforeSkew)
        && (Double.isNaN(afterGap) || afterGap >= p_min_pair_gap)
        && (Double.isNaN(afterGap) || afterGap <= p_max_pair_gap)
        && (!p_require_parallel_evidence || afterParallelLength > 0.0)
        && uncoupled_length_is_within_limit(afterUncoupledLength, p_max_uncoupled_length);
    boolean fullyPassing = hardSafe
        && (Double.isNaN(p_baseline_gap) || Double.isNaN(afterGap) || afterGap <= p_baseline_gap)
        && afterParallelLength > p_baseline_parallel_length
        && afterParallelRatio >= p_min_parallel_length_ratio;
    boolean monotonicImprovement = hardSafe
        && afterParallelLength > p_baseline_parallel_length
        && afterParallelRatio >= Math.min(p_min_parallel_length_ratio, 0.50)
        && (Double.isNaN(p_baseline_gap)
            || Double.isNaN(afterGap)
            || afterGap >= p_min_pair_gap
            || afterGap > p_baseline_gap)
        && skew <= beforeSkew;
    boolean allowMonotonicCommit = !p_candidate.family().startsWith("flow_through_");
    if (fullyPassing || (allowMonotonicCommit && monotonicImprovement)) {
      CoupledCandidateEvaluation evaluation = new CoupledCandidateEvaluation(
          p_candidate,
          true,
          fullyPassing ? "accepted" : "accepted as strict-safe monotonic improvement",
          skew,
          afterGap,
          afterParallelLength,
          afterParallelRatio,
          afterUncoupledLength,
          afterIncompletes,
          afterClearanceViolations);
      if (p_commit) {
        if (afterPairIncompletes > 0) {
          int repaired = repair_pair_accessory_connections(p_pair, p_candidate, afterFirstTraces, afterSecondTraces);
          if (repaired > 0) {
            logInfo(String.format(Locale.US,
                "Differential pair %s repaired %d same-net accessory connection%s after coupled replacement.",
                p_pair_name,
                repaired,
                repaired == 1 ? "" : "s"));
          }
        }
        board.pop_snapshot();
      } else {
        board.undo(null);
      }
      return evaluation;
    }

    String reason = String.format(Locale.US,
        "scoped=%s, skew=%.3f mm, baseline skew=%.3f mm, max skew=%.3f mm, gap=%.3f mm, min gap=%.3f mm, max gap=%.3f mm, baseline gap=%.3f mm, parallel=%.3f mm, baseline parallel=%.3f mm, parallel ratio=%.3f, min parallel ratio=%.3f, uncoupled=%.3f mm, max uncoupled=%.3f mm",
        after.scoped(),
        board_to_mm(board, skew),
        board_to_mm(board, beforeSkew),
        board_to_mm(board, maxSkewBoard),
        board_to_mm(board, afterGap),
        board_to_mm(board, p_min_pair_gap),
        board_to_mm(board, p_max_pair_gap),
        board_to_mm(board, p_baseline_gap),
        board_to_mm(board, afterParallelLength),
        board_to_mm(board, p_baseline_parallel_length),
        afterParallelRatio,
        p_min_parallel_length_ratio,
        board_to_mm(board, afterUncoupledLength),
        board_to_mm(board, p_max_uncoupled_length));
    board.undo(null);
    return new CoupledCandidateEvaluation(
        p_candidate,
        false,
        reason,
        skew,
        afterGap,
        afterParallelLength,
        afterParallelRatio,
        afterUncoupledLength,
        afterIncompletes,
        afterClearanceViolations);
  }

  private int repair_pair_accessory_connections(
      DifferentialPair p_pair,
      CoupledCandidate p_candidate,
      List<PolylineTrace> p_first_corridor_traces,
      List<PolylineTrace> p_second_corridor_traces) {
    if (job == null || job.routerSettings == null) {
      return 0;
    }
    List<PolylineTrace> fixedCorridorTraces = new ArrayList<>();
    fixedCorridorTraces.addAll(p_first_corridor_traces);
    fixedCorridorTraces.addAll(p_second_corridor_traces);
    for (PolylineTrace trace : fixedCorridorTraces) {
      trace.set_fixed_state(FixedState.USER_FIXED);
    }
    try {
      Set<Integer> protectedTraceIds = trace_id_set(fixedCorridorTraces);
      int repaired = reconnect_net_pins_to_corridor(p_pair.first_net_no(), p_candidate.first(), p_first_corridor_traces)
          + reconnect_net_pins_to_corridor(p_pair.second_net_no(), p_candidate.second(), p_second_corridor_traces);
      repaired += stitch_pair_accessory_airlines(p_pair);
      repaired += repair_net_accessory_connections(p_pair.first_net_no())
          + repair_net_accessory_connections(p_pair.second_net_no());
      repaired += prune_non_front_layer_net_items(p_pair.first_net_no(), protectedTraceIds)
          + prune_non_front_layer_net_items(p_pair.second_net_no(), protectedTraceIds);
      repaired += prune_redundant_net_traces(p_pair.first_net_no(), protectedTraceIds)
          + prune_redundant_net_traces(p_pair.second_net_no(), protectedTraceIds);
      repaired += prune_redundant_net_components(p_pair.first_net_no(), protectedTraceIds)
          + prune_redundant_net_components(p_pair.second_net_no(), protectedTraceIds);
      repaired += stitch_pair_accessory_airlines(p_pair);
      repaired += repair_net_accessory_connections(p_pair.first_net_no())
          + repair_net_accessory_connections(p_pair.second_net_no());
      return repaired;
    } finally {
      for (PolylineTrace trace : fixedCorridorTraces) {
        trace.set_fixed_state(FixedState.UNFIXED);
      }
    }
  }

  private boolean pair_route_satisfies_intent(
      PairMeasurements p_measurements,
      double p_gap,
      double p_parallel_length,
      double p_min_parallel_length_ratio,
      double p_max_uncoupled_length,
      double p_min_pair_gap,
      double p_max_pair_gap,
      boolean p_require_parallel_evidence) {
    if (!p_measurements.scoped()) {
      return false;
    }
    double skew = Math.abs(p_measurements.first().length() - p_measurements.second().length());
    double referenceLength = Math.max(p_measurements.first().length(), p_measurements.second().length());
    double parallelRatio = referenceLength > 0.0 ? p_parallel_length / referenceLength : 0.0;
    double uncoupledLength = uncoupled_length(p_measurements, p_parallel_length);
    double tolerance = mm_to_board(board, 0.001);
    return skew <= maxSkewBoard + tolerance
        && (Double.isNaN(p_gap) || p_gap + tolerance >= p_min_pair_gap)
        && (Double.isNaN(p_gap) || p_gap <= p_max_pair_gap + tolerance)
        && (!p_require_parallel_evidence || p_parallel_length > 0.0)
        && parallelRatio + 1e-6 >= p_min_parallel_length_ratio
        && uncoupledLength <= p_max_uncoupled_length + tolerance;
  }

  private double max_uncoupled_length(RouterIntentSettings.DifferentialPairIntent p_intent_pair) {
    if (p_intent_pair == null || p_intent_pair.maxUncoupledLengthMm == null) {
      return Double.POSITIVE_INFINITY;
    }
    return mm_to_board(board, Math.max(0.0, p_intent_pair.maxUncoupledLengthMm));
  }

  private static double uncoupled_length(PairMeasurements p_measurements, double p_parallel_length) {
    double referenceLength = Math.max(p_measurements.first().length(), p_measurements.second().length());
    return Math.max(0.0, referenceLength - Math.max(0.0, p_parallel_length));
  }

  private boolean uncoupled_length_is_within_limit(double p_uncoupled_length, double p_max_uncoupled_length) {
    if (!Double.isFinite(p_max_uncoupled_length)) {
      return true;
    }
    return p_uncoupled_length <= p_max_uncoupled_length + mm_to_board(board, 0.001);
  }

  private static Set<Integer> trace_id_set(List<PolylineTrace> p_traces) {
    Set<Integer> result = new HashSet<>();
    for (PolylineTrace trace : p_traces) {
      result.add(trace.get_id_no());
    }
    return result;
  }

  private int prune_non_front_layer_net_items(int p_net_no, Set<Integer> p_protected_trace_ids) {
    int frontLayer = board_layer_index("F.Cu");
    if (frontLayer < 0) {
      return 0;
    }
    int removed = 0;
    boolean progressed = true;
    while (progressed) {
      progressed = false;
      for (Item item : new ArrayList<>(board.get_connectable_items(p_net_no))) {
        if (!is_prunable_non_front_pair_item(item, p_net_no, p_protected_trace_ids, frontLayer)) {
          continue;
        }
        Set<Item> removeItems = connected_non_front_pair_component(
            item,
            p_net_no,
            p_protected_trace_ids,
            frontLayer);
        if (removeItems.isEmpty()) {
          continue;
        }
        int beforeIncompletes = incomplete_count_for_net(p_net_no);
        int beforeClearance = clearance_violation_count(board);
        int beforeUnconnected = unconnected_item_count_for_net(p_net_no);
        board.generate_snapshot();
        if (!board.remove_items(removeItems)) {
          board.undo(null);
          continue;
        }
        board.combine_traces(p_net_no);
        int afterIncompletes = incomplete_count_for_net(p_net_no);
        int afterClearance = clearance_violation_count(board);
        int afterUnconnected = unconnected_item_count_for_net(p_net_no);
        if (afterIncompletes <= beforeIncompletes
            && afterClearance <= beforeClearance
            && afterUnconnected <= beforeUnconnected) {
          board.pop_snapshot();
          removed += removeItems.size();
          logInfo(String.format(Locale.US,
              "Differential pair cleanup removed %d non-front accessory item%s from net %s.",
              removeItems.size(),
              removeItems.size() == 1 ? "" : "s",
              net_name(p_net_no)));
          progressed = true;
          break;
        }
        logInfo(String.format(Locale.US,
            "Differential pair cleanup kept non-front accessory branch on net %s; removing %d item%s would change incompletes %d->%d, unconnected %d->%d, clearance %d->%d.",
            net_name(p_net_no),
            removeItems.size(),
            removeItems.size() == 1 ? "" : "s",
            beforeIncompletes,
            afterIncompletes,
            beforeUnconnected,
            afterUnconnected,
            beforeClearance,
            afterClearance));
        board.undo(null);
      }
    }
    return removed;
  }

  private Set<Item> connected_non_front_pair_component(
      Item p_seed,
      int p_net_no,
      Set<Integer> p_protected_trace_ids,
      int p_front_layer) {
    Set<Item> result = new HashSet<>();
    List<Item> stack = new ArrayList<>();
    stack.add(p_seed);
    while (!stack.isEmpty()) {
      Item item = stack.remove(stack.size() - 1);
      if (result.contains(item)
          || !is_prunable_pair_accessory_item(item, p_net_no, p_protected_trace_ids)) {
        continue;
      }
      result.add(item);
      for (Item contact : item.get_normal_contacts()) {
        if (contact.contains_net(p_net_no)
            && is_prunable_pair_accessory_item(contact, p_net_no, p_protected_trace_ids)) {
          stack.add(contact);
        }
      }
    }
    return result;
  }

  private static boolean is_prunable_non_front_pair_item(
      Item p_item,
      int p_net_no,
      Set<Integer> p_protected_trace_ids,
      int p_front_layer) {
    if (p_item == null
        || p_item.is_user_fixed()
        || p_item.net_count() != 1
        || !p_item.contains_net(p_net_no)
        || p_item instanceof Pin) {
      return false;
    }
    if (p_item instanceof PolylineTrace trace) {
      if (p_protected_trace_ids.contains(trace.get_id_no())) {
        return false;
      }
      return trace.first_layer() != p_front_layer || trace.last_layer() != p_front_layer;
    }
    return p_item instanceof Via;
  }

  private static boolean is_prunable_pair_accessory_item(
      Item p_item,
      int p_net_no,
      Set<Integer> p_protected_trace_ids) {
    if (p_item == null
        || p_item.is_user_fixed()
        || p_item.net_count() != 1
        || !p_item.contains_net(p_net_no)
        || p_item instanceof Pin) {
      return false;
    }
    if (p_item instanceof PolylineTrace trace) {
      return !p_protected_trace_ids.contains(trace.get_id_no());
    }
    return p_item instanceof Via;
  }

  private int prune_redundant_net_traces(int p_net_no, Set<Integer> p_protected_trace_ids) {
    int removed = 0;
    boolean progressed = true;
    while (progressed) {
      progressed = false;
      for (Item item : new ArrayList<>(board.get_connectable_items(p_net_no))) {
        if (!(item instanceof PolylineTrace trace)
            || trace.is_user_fixed()
            || trace.net_count() != 1
            || trace.get_net_no(0) != p_net_no
            || trace.get_length() < mm_to_board(board, 0.25)
            || p_protected_trace_ids.contains(trace.get_id_no())) {
          continue;
        }
        int beforeIncompletes = incomplete_count_for_net(p_net_no);
        int beforeClearance = clearance_violation_count(board);
        int beforeUnconnected = unconnected_item_count_for_net(p_net_no);
        Set<Item> removeItems = new HashSet<>();
        removeItems.add(trace);
        board.generate_snapshot();
        if (!board.remove_items(removeItems)) {
          board.undo(null);
          continue;
        }
        board.combine_traces(p_net_no);
        int afterIncompletes = incomplete_count_for_net(p_net_no);
        int afterClearance = clearance_violation_count(board);
        int afterUnconnected = unconnected_item_count_for_net(p_net_no);
        if (afterIncompletes <= beforeIncompletes
            && afterClearance <= beforeClearance
            && afterUnconnected <= beforeUnconnected) {
          board.pop_snapshot();
          removed++;
          progressed = true;
          break;
        }
        board.undo(null);
      }
    }
    return removed;
  }

  private int prune_redundant_net_components(int p_net_no, Set<Integer> p_protected_trace_ids) {
    int removed = 0;
    boolean progressed = true;
    while (progressed) {
      progressed = false;
      Set<Item> visited = new HashSet<>();
      for (Item item : new ArrayList<>(board.get_connectable_items(p_net_no))) {
        if (visited.contains(item) || !is_prunable_pair_accessory_item(item, p_net_no, p_protected_trace_ids)) {
          continue;
        }
        Set<Item> component = connected_pair_accessory_component(item, p_net_no, p_protected_trace_ids);
        visited.addAll(component);
        if (component.size() <= 1) {
          continue;
        }
        int beforeIncompletes = incomplete_count_for_net(p_net_no);
        int beforeClearance = clearance_violation_count(board);
        int beforeUnconnected = unconnected_item_count_for_net(p_net_no);
        board.generate_snapshot();
        if (!board.remove_items(component)) {
          board.undo(null);
          continue;
        }
        board.combine_traces(p_net_no);
        int afterIncompletes = incomplete_count_for_net(p_net_no);
        int afterClearance = clearance_violation_count(board);
        int afterUnconnected = unconnected_item_count_for_net(p_net_no);
        if (afterIncompletes <= beforeIncompletes
            && afterClearance <= beforeClearance
            && afterUnconnected <= beforeUnconnected) {
          board.pop_snapshot();
          removed += component.size();
          logInfo(String.format(Locale.US,
              "Differential pair cleanup removed %d redundant accessory item%s from net %s.",
              component.size(),
              component.size() == 1 ? "" : "s",
              net_name(p_net_no)));
          progressed = true;
          break;
        }
        board.undo(null);
      }
    }
    return removed;
  }

  private Set<Item> connected_pair_accessory_component(
      Item p_seed,
      int p_net_no,
      Set<Integer> p_protected_trace_ids) {
    Set<Item> result = new HashSet<>();
    List<Item> stack = new ArrayList<>();
    stack.add(p_seed);
    while (!stack.isEmpty()) {
      Item item = stack.remove(stack.size() - 1);
      if (result.contains(item) || !is_prunable_pair_accessory_item(item, p_net_no, p_protected_trace_ids)) {
        continue;
      }
      result.add(item);
      for (Item contact : item.get_normal_contacts()) {
        if (is_prunable_pair_accessory_item(contact, p_net_no, p_protected_trace_ids)) {
          stack.add(contact);
        }
      }
    }
    return result;
  }

  private int unconnected_item_count_for_net(int p_net_no) {
    int count = 0;
    for (UnconnectedItems unconnected : new DesignRulesChecker(board, null).getAllUnconnectedItems()) {
      if (unconnected.first_item != null && unconnected.first_item.contains_net(p_net_no)) {
        count++;
        continue;
      }
      if (unconnected.second_item != null && unconnected.second_item.contains_net(p_net_no)) {
        count++;
      }
    }
    return count;
  }

  private int repair_net_accessory_connections(int p_net_no) {
    int repaired = 0;
    int viaCosts = job.routerSettings.get_via_costs();
    for (Item item : new ArrayList<>(board.get_connectable_items(p_net_no))) {
      if (item.is_user_fixed() || item.net_count() != 1 || item.get_net_no(0) != p_net_no) {
        continue;
      }
      if (item.get_unconnected_set(p_net_no).isEmpty()) {
        continue;
      }
      int before = incomplete_count_for_net(p_net_no);
      board.autoroute(item, job.routerSettings, viaCosts, null, null);
      board.combine_traces(p_net_no);
      int after = incomplete_count_for_net(p_net_no);
      if (after < before) {
        repaired += before - after;
      }
      if (after == 0) {
        break;
      }
    }
    return repaired;
  }

  private int reconnect_net_pins_to_corridor(
      int p_net_no,
      Point[] p_candidate_points,
      List<PolylineTrace> p_corridor_traces) {
    if (p_candidate_points.length < 2 || p_corridor_traces.isEmpty()) {
      return 0;
    }
    Net net = board.rules.nets.get(p_net_no);
    if (net == null) {
      return 0;
    }
    int layer = board_layer_index("F.Cu");
    TraceStyle style = trace_style(net, layer);
    if (style.halfWidth() <= 0) {
      return 0;
    }
    int repaired = 0;
    for (Item item : new ArrayList<>(board.get_connectable_items(p_net_no))) {
      if (!(item instanceof Pin pin) || item.is_user_fixed() || item.net_count() != 1 || item.get_net_no(0) != p_net_no) {
        continue;
      }
      if (!pin.is_on_layer(layer)) {
        continue;
      }
      Point start = pin.get_center();
      Point target = nearest_corridor_point(start.to_float(), p_corridor_traces);
      if (target == null || start.equals(target)) {
        continue;
      }
      int beforeIncompletes = incomplete_count_for_net(p_net_no);
      int beforeClearance = clearance_violation_count(board);
      board.generate_snapshot();
      boolean inserted = insert_forced_accessory_connection(
          start,
          target,
          style.halfWidth(),
          layer,
          p_net_no,
          style.clearanceClass());
      if (!inserted) {
        board.undo(null);
        continue;
      }
      board.combine_traces(p_net_no);
      int afterIncompletes = incomplete_count_for_net(p_net_no);
      int afterClearance = clearance_violation_count(board);
      if (afterIncompletes <= beforeIncompletes
          && afterClearance <= beforeClearance) {
        board.pop_snapshot();
        repaired += Math.max(1, beforeIncompletes - afterIncompletes);
        continue;
      }
      board.undo(null);
    }
    return repaired;
  }

  private boolean insert_forced_accessory_connection(
      Point p_start,
      Point p_target,
      int p_half_width,
      int p_layer,
      int p_net_no,
      int p_clearance_class) {
    List<Point[]> candidates = accessory_connection_candidates(p_start, p_target);
    for (int halfWidth : accessory_escape_half_widths(p_half_width)) {
      for (Point[] points : candidates) {
        Polyline polyline = new Polyline(points);
        if (polyline.arr.length == 0) {
          continue;
        }
        board.generate_snapshot();
        Point reached = board.insert_forced_trace_polyline(
            polyline,
            halfWidth,
            p_layer,
            new int[] { p_net_no },
            p_clearance_class,
            COUPLED_INSERT_MAX_TRACE_RECURSION_DEPTH,
            COUPLED_INSERT_MAX_VIA_RECURSION_DEPTH,
            COUPLED_INSERT_MAX_SPRING_OVER_RECURSION_DEPTH,
            Integer.MAX_VALUE,
            COUPLED_INSERT_PULL_TIGHT_ACCURACY,
            true,
            null);
        if (reached != null && reached.equals(p_target)) {
          board.pop_snapshot();
          return true;
        }
        board.undo(null);
      }
    }
    return false;
  }

  private List<Point[]> accessory_connection_candidates(Point p_start, Point p_target) {
    List<Point[]> result = new ArrayList<>();
    add_accessory_candidate(result, p_start, p_target);
    if (!(p_start instanceof IntPoint start) || !(p_target instanceof IntPoint target)) {
      return result;
    }

    add_accessory_candidate(result, p_start, new IntPoint(target.x, start.y), p_target);
    add_accessory_candidate(result, p_start, new IntPoint(start.x, target.y), p_target);

    int minX = Math.min(start.x, target.x);
    int maxX = Math.max(start.x, target.x);
    int minY = Math.min(start.y, target.y);
    int maxY = Math.max(start.y, target.y);
    int midX = (int) Math.round((start.x + target.x) / 2.0);
    int midY = (int) Math.round((start.y + target.y) / 2.0);
    int[] laneOffsets = {
        (int) Math.round(mm_to_board(board, 0.40)),
        (int) Math.round(mm_to_board(board, 0.80)),
        (int) Math.round(mm_to_board(board, 1.20)),
        (int) Math.round(mm_to_board(board, 1.80)),
        (int) Math.round(mm_to_board(board, 2.50)),
    };

    add_accessory_candidate(result, p_start, new IntPoint(start.x, midY), new IntPoint(target.x, midY), p_target);
    add_accessory_candidate(result, p_start, new IntPoint(midX, start.y), new IntPoint(midX, target.y), p_target);

    for (int offset : laneOffsets) {
      if (offset <= 0) {
        continue;
      }
      int[] laneYs = { minY - offset, maxY + offset };
      int[] laneXs = { minX - offset, maxX + offset };
      for (int laneY : laneYs) {
        add_accessory_candidate(result, p_start, new IntPoint(start.x, laneY), new IntPoint(target.x, laneY), p_target);
        for (int laneX : laneXs) {
          add_accessory_candidate(
              result,
              p_start,
              new IntPoint(start.x, laneY),
              new IntPoint(laneX, laneY),
              new IntPoint(laneX, target.y),
              p_target);
          add_accessory_candidate(
              result,
              p_start,
              new IntPoint(laneX, start.y),
              new IntPoint(laneX, laneY),
              new IntPoint(target.x, laneY),
              p_target);
        }
      }
      for (int laneX : laneXs) {
        add_accessory_candidate(result, p_start, new IntPoint(laneX, start.y), new IntPoint(laneX, target.y), p_target);
        for (int laneY : laneYs) {
          add_accessory_candidate(
              result,
              p_start,
              new IntPoint(laneX, start.y),
              new IntPoint(laneX, laneY),
              new IntPoint(target.x, laneY),
              p_target);
          add_accessory_candidate(
              result,
              p_start,
              new IntPoint(start.x, laneY),
              new IntPoint(laneX, laneY),
              new IntPoint(laneX, target.y),
              p_target);
        }
      }
    }
    return result;
  }

  private static void add_accessory_candidate(List<Point[]> p_candidates, Point... p_points) {
    List<Point> points = new ArrayList<>();
    for (Point point : p_points) {
      append_distinct(points, point);
    }
    if (points.size() < 2) {
      return;
    }
    Point[] candidate = points.toArray(Point[]::new);
    for (Point[] existing : p_candidates) {
      if (same_points(existing, candidate)) {
        return;
      }
    }
    p_candidates.add(candidate);
  }

  private static boolean same_points(Point[] p_first, Point[] p_second) {
    if (p_first.length != p_second.length) {
      return false;
    }
    for (int i = 0; i < p_first.length; i++) {
      if (!p_first[i].equals(p_second[i])) {
        return false;
      }
    }
    return true;
  }

  private List<Integer> accessory_escape_half_widths(int p_nominal_half_width) {
    int minHalfWidth = Math.max(1, board.get_min_trace_half_width());
    List<Integer> result = new ArrayList<>();
    add_unique_width(result, p_nominal_half_width);
    add_unique_width(result, Math.max(minHalfWidth, (int) Math.round(p_nominal_half_width * 0.75)));
    add_unique_width(result, Math.max(minHalfWidth, (int) Math.round(p_nominal_half_width * 0.60)));
    add_unique_width(result, minHalfWidth);
    return result;
  }

  private static void add_unique_width(List<Integer> p_widths, int p_width) {
    if (p_width <= 0 || p_widths.contains(p_width)) {
      return;
    }
    p_widths.add(p_width);
  }

  private static Point pin_escape_point(
      Pin p_pin,
      Point[] p_corridor_points,
      int p_trace_half_width,
      int p_layer) {
    Point center = p_pin.get_center();
    Point nearestCorridor = nearest_corridor_point(center.to_float(), p_corridor_points);
    if (nearestCorridor == null) {
      return center;
    }
    FloatPoint exit = p_pin.nearest_trace_exit_corner(nearestCorridor.to_float(), p_trace_half_width, p_layer);
    return exit == null ? center : exit.round();
  }

  private static Point nearest_corridor_point(FloatPoint p_from, Point[] p_points) {
    FloatPoint best = null;
    double bestDistance = Double.POSITIVE_INFINITY;
    for (int i = 0; i < p_points.length - 1; i++) {
      FloatPoint candidate = new app.freerouting.geometry.planar.FloatLine(
          p_points[i].to_float(),
          p_points[i + 1].to_float()).nearest_segment_point(p_from);
      if (candidate == null) {
        continue;
      }
      double distance = candidate.distance_square(p_from);
      if (distance < bestDistance) {
        bestDistance = distance;
        best = candidate;
      }
    }
    return best == null ? null : best.round();
  }

  private static Point nearest_corridor_point(FloatPoint p_from, List<PolylineTrace> p_traces) {
    FloatPoint best = null;
    double bestDistance = Double.POSITIVE_INFINITY;
    for (PolylineTrace trace : p_traces) {
      FloatPoint candidate = trace.polyline().nearest_point_approx(p_from);
      if (candidate == null) {
        continue;
      }
      double distance = candidate.distance_square(p_from);
      if (distance < bestDistance) {
        bestDistance = distance;
        best = candidate;
      }
    }
    return best == null ? null : best.round();
  }

  private int stitch_pair_accessory_airlines(DifferentialPair p_pair) {
    return stitch_net_accessory_airlines(p_pair.first_net_no())
        + stitch_net_accessory_airlines(p_pair.second_net_no());
  }

  private int stitch_net_accessory_airlines(int p_net_no) {
    Net net = board.rules.nets.get(p_net_no);
    if (net == null) {
      return 0;
    }
    TraceStyle style = trace_style(net, board_layer_index("F.Cu"));
    if (style.halfWidth() <= 0) {
      return 0;
    }
    int repaired = 0;
    boolean progressed = true;
    while (progressed) {
      progressed = false;
      DesignRulesChecker checker = new DesignRulesChecker(board, null);
      checker.calculateAllIncompletes();
      int beforeIncompletes = checker.getIncompleteCount(p_net_no);
      int beforeClearance = clearance_violation_count(board);
      if (beforeIncompletes <= 0) {
        break;
      }
      for (AirLine airline : checker.getAllAirlines()) {
        if (airline.net == null || airline.net.net_number != p_net_no) {
          continue;
        }
        if (!airline.from_item.shares_layer(airline.to_item)) {
          continue;
        }
        int layer = common_layer(airline.from_item, airline.to_item);
        if (layer < 0) {
          continue;
        }
        board.generate_snapshot();
        Point from = airline.from_corner.round();
        Point to = airline.to_corner.round();
        Point reached = board.insert_forced_trace_segment(
            from,
            to,
            style.halfWidth(),
            layer,
            new int[] { p_net_no },
            style.clearanceClass(),
            COUPLED_INSERT_MAX_TRACE_RECURSION_DEPTH,
            COUPLED_INSERT_MAX_VIA_RECURSION_DEPTH,
            COUPLED_INSERT_MAX_SPRING_OVER_RECURSION_DEPTH,
            Integer.MAX_VALUE,
            COUPLED_INSERT_PULL_TIGHT_ACCURACY,
            true,
            null);
        if (reached == null || !reached.equals(to)) {
          board.undo(null);
          continue;
        }
        board.combine_traces(p_net_no);
        DesignRulesChecker afterChecker = new DesignRulesChecker(board, null);
        afterChecker.calculateAllIncompletes();
        int afterIncompletes = afterChecker.getIncompleteCount(p_net_no);
        int afterClearance = clearance_violation_count(board);
        if (afterIncompletes < beforeIncompletes && afterClearance <= beforeClearance) {
          board.pop_snapshot();
          repaired += beforeIncompletes - afterIncompletes;
          progressed = true;
          break;
        }
        board.undo(null);
      }
    }
    return repaired;
  }

  private static int common_layer(Item p_first, Item p_second) {
    int firstFrom = p_first.first_layer();
    int firstTo = p_first.last_layer();
    int secondFrom = p_second.first_layer();
    int secondTo = p_second.last_layer();
    int from = Math.max(firstFrom, secondFrom);
    int to = Math.min(firstTo, secondTo);
    return from <= to ? from : -1;
  }

  private CoupledCandidateEvaluation evaluate_insert_pair_routes(
      DifferentialPair p_pair,
      CoupledCandidate p_candidate,
      int p_layer,
      TraceStyle p_first_style,
      TraceStyle p_second_style,
      int p_max_incompletes,
      int p_max_clearance_violations,
      double p_max_skew,
      double p_min_parallel_length_ratio,
      double p_max_uncoupled_length,
      double p_min_pair_gap,
      double p_max_pair_gap,
      boolean p_require_parallel_evidence,
      boolean p_commit) {
    board.generate_snapshot();
    insert_exact_coupled_trace(
        p_candidate.first(),
        p_layer,
        p_first_style,
        p_pair.first_net_no(),
        FixedState.SHOVE_FIXED);
    insert_exact_coupled_trace(
        p_candidate.second(),
        p_layer,
        p_second_style,
        p_pair.second_net_no(),
        FixedState.SHOVE_FIXED);

    board.combine_traces(p_pair.first_net_no());
    board.combine_traces(p_pair.second_net_no());

    int afterIncompletes = incomplete_count(board);
    ClearanceCheck clearanceCheck = clearance_check_excluding_pair(board, p_pair, 4);
    int afterClearanceViolations = clearanceCheck.count();
    if (afterIncompletes > p_max_incompletes || afterClearanceViolations > p_max_clearance_violations) {
      String reason = String.format(Locale.US,
          "incompletes %d>%d, clearance violations %d>%d%s",
          afterIncompletes,
          p_max_incompletes,
          afterClearanceViolations,
          p_max_clearance_violations,
          clearanceCheck.summary().isEmpty() ? "" : "; " + clearanceCheck.summary());
      board.undo(null);
      return CoupledCandidateEvaluation.rejected(p_candidate, reason, afterIncompletes, afterClearanceViolations);
    }

    Net firstNet = board.rules.nets.get(p_pair.first_net_no());
    Net secondNet = board.rules.nets.get(p_pair.second_net_no());
    if (firstNet == null || secondNet == null) {
      board.undo(null);
      return CoupledCandidateEvaluation.rejected(p_candidate, "pair nets were not found");
    }
    PairMeasurements after = measure_pair(p_pair, firstNet, secondNet);
    double skew = Math.abs(after.first().length() - after.second().length());
    List<PolylineTrace> afterFirstTraces = collect_pair_traces(after.first());
    List<PolylineTrace> afterSecondTraces = collect_pair_traces(after.second());
    double afterGap = nearest_gap_between_traces(afterFirstTraces, afterSecondTraces);
    double afterParallelLength = parallel_length(
        afterFirstTraces,
        afterSecondTraces,
        p_candidate.centerSpacing());
    double afterReferenceLength = Math.max(after.first().length(), after.second().length());
    double afterParallelRatio = afterReferenceLength > 0.0 ? afterParallelLength / afterReferenceLength : 0.0;
    double afterUncoupledLength = uncoupled_length(after, afterParallelLength);
    if (after.scoped()
        && skew <= p_max_skew
        && (Double.isNaN(afterGap) || afterGap >= p_min_pair_gap)
        && (Double.isNaN(afterGap) || afterGap <= p_max_pair_gap)
        && (!p_require_parallel_evidence || afterParallelLength > 0.0)
        && afterParallelRatio >= p_min_parallel_length_ratio
        && uncoupled_length_is_within_limit(afterUncoupledLength, p_max_uncoupled_length)) {
      CoupledCandidateEvaluation evaluation = new CoupledCandidateEvaluation(
          p_candidate,
          true,
          "accepted",
          skew,
          afterGap,
          afterParallelLength,
          afterParallelRatio,
          afterUncoupledLength,
          afterIncompletes,
          afterClearanceViolations);
      if (p_commit) {
        board.pop_snapshot();
      } else {
        board.undo(null);
      }
      return evaluation;
    }

    String reason = String.format(Locale.US,
        "scoped=%s, skew=%.3f mm, max skew=%.3f mm, gap=%.3f mm, min gap=%.3f mm, max gap=%.3f mm, parallel=%.3f mm, parallel ratio=%.3f, min parallel ratio=%.3f, uncoupled=%.3f mm, max uncoupled=%.3f mm",
        after.scoped(),
        board_to_mm(board, skew),
        board_to_mm(board, p_max_skew),
        board_to_mm(board, afterGap),
        board_to_mm(board, p_min_pair_gap),
        board_to_mm(board, p_max_pair_gap),
        board_to_mm(board, afterParallelLength),
        afterParallelRatio,
        p_min_parallel_length_ratio,
        board_to_mm(board, afterUncoupledLength),
        board_to_mm(board, p_max_uncoupled_length));
    board.undo(null);
    return new CoupledCandidateEvaluation(
        p_candidate,
        false,
        reason,
        skew,
        afterGap,
        afterParallelLength,
        afterParallelRatio,
        afterUncoupledLength,
        afterIncompletes,
        afterClearanceViolations);
  }

  private MissingMemberCandidateEvaluation evaluate_insert_missing_pair_member(
      DifferentialPair p_pair,
      MissingMemberCandidate p_candidate,
      int p_layer,
      TraceStyle p_style,
      TraceStyle p_endpoint_escape_style,
      double p_endpoint_escape_length,
      int p_net_no,
      Net p_first_net,
      Net p_second_net,
      int p_max_incompletes,
      int p_max_clearance_violations,
      double p_min_parallel_length_ratio,
      double p_max_uncoupled_length,
      double p_min_pair_gap,
      double p_max_pair_gap,
      boolean p_require_parallel_evidence,
      boolean p_commit) {
    board.generate_snapshot();
    insert_missing_member_trace(
        p_candidate.points(),
        p_layer,
        p_style,
        p_endpoint_escape_style,
        p_endpoint_escape_length,
        p_net_no,
        FixedState.SHOVE_FIXED);
    board.combine_traces(p_net_no);

    int afterIncompletes = incomplete_count(board);
    ClearanceCheck clearanceCheck = clearance_check_excluding_pair(board, p_pair, 4);
    int afterClearanceViolations = clearanceCheck.count();
    if (afterIncompletes > p_max_incompletes || afterClearanceViolations > p_max_clearance_violations) {
      String reason = String.format(Locale.US,
          "incompletes %d>%d, clearance violations %d>%d%s",
          afterIncompletes,
          p_max_incompletes,
          afterClearanceViolations,
          p_max_clearance_violations,
          clearanceCheck.summary().isEmpty() ? "" : "; " + clearanceCheck.summary());
      board.undo(null);
      return MissingMemberCandidateEvaluation.rejected(p_candidate, reason, afterIncompletes, afterClearanceViolations);
    }

    PairMeasurements after = measure_pair(p_pair, p_first_net, p_second_net);
    double skew = Math.abs(after.first().length() - after.second().length());
    List<PolylineTrace> afterFirstTraces = collect_pair_traces(after.first());
    List<PolylineTrace> afterSecondTraces = collect_pair_traces(after.second());
    double afterGap = nearest_gap_between_traces(afterFirstTraces, afterSecondTraces);
    double afterParallelLength = parallel_length(
        afterFirstTraces,
        afterSecondTraces,
        p_candidate.centerSpacing());
    double afterReferenceLength = Math.max(after.first().length(), after.second().length());
    double afterParallelRatio = afterReferenceLength > 0.0 ? afterParallelLength / afterReferenceLength : 0.0;
    double afterUncoupledLength = uncoupled_length(after, afterParallelLength);
    if (after.scoped()
        && skew <= maxSkewBoard
        && (Double.isNaN(afterGap) || afterGap >= p_min_pair_gap)
        && (Double.isNaN(afterGap) || afterGap <= p_max_pair_gap)
        && (!p_require_parallel_evidence || afterParallelLength > 0.0)
        && afterParallelRatio >= p_min_parallel_length_ratio
        && uncoupled_length_is_within_limit(afterUncoupledLength, p_max_uncoupled_length)) {
      MissingMemberCandidateEvaluation evaluation = new MissingMemberCandidateEvaluation(
          p_candidate,
          true,
          "accepted",
          skew,
          afterGap,
          afterParallelLength,
          afterParallelRatio,
          afterUncoupledLength,
          afterIncompletes,
          afterClearanceViolations);
      if (p_commit) {
        board.pop_snapshot();
      } else {
        board.undo(null);
      }
      return evaluation;
    }

    String reason = String.format(Locale.US,
        "scoped=%s, skew=%.3f mm, max skew=%.3f mm, gap=%.3f mm, min gap=%.3f mm, max gap=%.3f mm, parallel=%.3f mm, parallel ratio=%.3f, min parallel ratio=%.3f, uncoupled=%.3f mm, max uncoupled=%.3f mm",
        after.scoped(),
        board_to_mm(board, skew),
        board_to_mm(board, maxSkewBoard),
        board_to_mm(board, afterGap),
        board_to_mm(board, p_min_pair_gap),
        board_to_mm(board, p_max_pair_gap),
        board_to_mm(board, afterParallelLength),
        afterParallelRatio,
        p_min_parallel_length_ratio,
        board_to_mm(board, afterUncoupledLength),
        board_to_mm(board, p_max_uncoupled_length));
    board.undo(null);
    return new MissingMemberCandidateEvaluation(
        p_candidate,
        false,
        reason,
        skew,
        afterGap,
        afterParallelLength,
        afterParallelRatio,
        afterUncoupledLength,
        afterIncompletes,
        afterClearanceViolations);
  }

  private Point insert_forced_coupled_trace(Point[] p_points, int p_layer, TraceStyle p_style, int p_net_no) {
    Polyline polyline = new Polyline(p_points);
    return board.insert_forced_trace_polyline(
        polyline,
        p_style.halfWidth(),
        p_layer,
        new int[] { p_net_no },
        p_style.clearanceClass(),
        COUPLED_INSERT_MAX_TRACE_RECURSION_DEPTH,
        COUPLED_INSERT_MAX_VIA_RECURSION_DEPTH,
        COUPLED_INSERT_MAX_SPRING_OVER_RECURSION_DEPTH,
        Integer.MAX_VALUE,
        COUPLED_INSERT_PULL_TIGHT_ACCURACY,
        true,
        null);
  }

  private List<CoupledCandidate> ranked_coupled_candidates(
      List<CoupledCandidate> p_candidates,
      int p_limit,
      double p_target_center_spacing) {
    if (p_candidates.size() <= p_limit) {
      p_candidates.sort(Comparator.<CoupledCandidate>comparingDouble(
          candidate -> raw_coupled_candidate_score(candidate, p_target_center_spacing)).reversed());
      return p_candidates;
    }
    List<CoupledCandidate> result = new ArrayList<>(p_candidates);
    result.sort(Comparator.<CoupledCandidate>comparingDouble(
        candidate -> raw_coupled_candidate_score(candidate, p_target_center_spacing)).reversed());

    List<CoupledCandidate> selected = new ArrayList<>(p_limit);
    Set<CoupledCandidate> selectedSet = new HashSet<>();
    Map<String, Integer> selectedByFamily = new HashMap<>();
    for (CoupledCandidate candidate : result) {
      int count = selectedByFamily.getOrDefault(candidate.family(), 0);
      if (count >= MIN_COUPLED_CANDIDATES_PER_FAMILY) {
        continue;
      }
      selected.add(candidate);
      selectedSet.add(candidate);
      selectedByFamily.put(candidate.family(), count + 1);
      if (selected.size() >= p_limit) {
        return selected;
      }
    }
    for (CoupledCandidate candidate : result) {
      if (selectedSet.add(candidate)) {
        selected.add(candidate);
        if (selected.size() >= p_limit) {
          break;
        }
      }
    }
    selected.sort(Comparator.<CoupledCandidate>comparingDouble(
        candidate -> raw_coupled_candidate_score(candidate, p_target_center_spacing)).reversed());
    return selected;
  }

  private double raw_coupled_candidate_score(CoupledCandidate p_candidate, double p_target_center_spacing) {
    double firstLength = point_path_length(p_candidate.first());
    double secondLength = point_path_length(p_candidate.second());
    double shortest = Math.min(firstLength, secondLength);
    double longest = Math.max(firstLength, secondLength);
    double skew = longest - shortest;
    double pairLength = paired_point_path_length(p_candidate.first(), p_candidate.second(), p_candidate.centerSpacing());
    double parallelRatio = longest > 0.0 ? pairLength / longest : 0.0;
    double gap = nearest_polyline_distance(p_candidate.first(), p_candidate.second());
    double gapPenalty = Double.isFinite(gap) ? Math.abs(gap - p_target_center_spacing) : p_target_center_spacing;
    double familyBonus = p_candidate.family().startsWith("flow_through_") ? 25000.0 : 0.0;
    return familyBonus + (parallelRatio * 10000.0) + pairLength - (skew * 1000.0) - (gapPenalty * 100.0);
  }

  private static double point_path_length(Point[] p_points) {
    double result = 0.0;
    for (int i = 0; i < p_points.length - 1; i++) {
      result += p_points[i].to_float().distance(p_points[i + 1].to_float());
    }
    return result;
  }

  private static double paired_point_path_length(Point[] p_first, Point[] p_second, double p_max_center_distance) {
    double result = 0.0;
    for (int i = 0; i < p_first.length - 1; i++) {
      FloatPoint a = p_first[i].to_float();
      FloatPoint b = p_first[i + 1].to_float();
      for (int j = 0; j < p_second.length - 1; j++) {
        FloatPoint c = p_second[j].to_float();
        FloatPoint d = p_second[j + 1].to_float();
        if (!segments_parallel(a, b, c, d)) {
          continue;
        }
        double distance = segment_distance(a, b, c, d);
        if (distance <= Math.max(p_max_center_distance * 1.75, p_max_center_distance + 1.0)) {
          result += parallel_overlap_length(a, b, c, d);
        }
      }
    }
    return result;
  }

  private void insert_exact_coupled_trace(Point[] p_points, int p_layer, TraceStyle p_style, int p_net_no) {
    insert_exact_coupled_trace(p_points, p_layer, p_style, p_net_no, FixedState.UNFIXED);
  }

  private void insert_exact_coupled_trace(
      Point[] p_points,
      int p_layer,
      TraceStyle p_style,
      int p_net_no,
      FixedState p_fixed_state) {
    board.insert_trace(
        p_points,
        p_layer,
        p_style.halfWidth(),
        new int[] { p_net_no },
        p_style.clearanceClass(),
        p_fixed_state);
  }

  private void insert_missing_member_trace(
      Point[] p_points,
      int p_layer,
      TraceStyle p_style,
      TraceStyle p_endpoint_escape_style,
      double p_endpoint_escape_length,
      int p_net_no,
      FixedState p_fixed_state) {
    if (p_endpoint_escape_style == null
        || p_endpoint_escape_style.halfWidth() <= 0
        || p_endpoint_escape_style.halfWidth() >= p_style.halfWidth()
        || p_endpoint_escape_length <= 0.0
        || p_points.length < 2) {
      insert_exact_coupled_trace(p_points, p_layer, p_style, p_net_no, p_fixed_state);
      return;
    }

    double totalLength = point_path_length(p_points);
    if (totalLength <= p_endpoint_escape_length * 2.0) {
      insert_exact_coupled_trace(p_points, p_layer, p_endpoint_escape_style, p_net_no, p_fixed_state);
      return;
    }

    Point firstEndpoint = point_along_polyline(p_points, p_endpoint_escape_length, false);
    Point lastEndpoint = point_along_polyline(p_points, p_endpoint_escape_length, true);
    if (firstEndpoint == null || lastEndpoint == null || firstEndpoint.equals(lastEndpoint)) {
      insert_exact_coupled_trace(p_points, p_layer, p_style, p_net_no, p_fixed_state);
      return;
    }

    insert_exact_coupled_trace(
        new Point[] { p_points[0], firstEndpoint },
        p_layer,
        p_endpoint_escape_style,
        p_net_no,
        p_fixed_state);
    insert_exact_coupled_trace(
        middle_neckdown_points(p_points, firstEndpoint, lastEndpoint, p_endpoint_escape_length, totalLength - p_endpoint_escape_length),
        p_layer,
        p_style,
        p_net_no,
        p_fixed_state);
    insert_exact_coupled_trace(
        new Point[] { lastEndpoint, p_points[p_points.length - 1] },
        p_layer,
        p_endpoint_escape_style,
        p_net_no,
        p_fixed_state);
  }

  private static Point[] middle_neckdown_points(
      Point[] p_points,
      Point p_start,
      Point p_end,
      double p_start_distance,
      double p_end_distance) {
    List<Point> result = new ArrayList<>();
    append_distinct(result, p_start);
    double walked = 0.0;
    for (int i = 1; i < p_points.length - 1; i++) {
      walked += p_points[i - 1].to_float().distance(p_points[i].to_float());
      if (walked > p_start_distance && walked < p_end_distance) {
        append_distinct(result, p_points[i]);
      }
    }
    append_distinct(result, p_end);
    return result.toArray(Point[]::new);
  }

  private List<CoupledCandidate> coupled_candidates_with_pin_gateways(
      Pin p_first_from,
      Pin p_first_to,
      Pin p_second_from,
      Pin p_second_to,
      List<PolylineTrace> p_first_traces,
      List<PolylineTrace> p_second_traces,
      TraceStyle p_first_style,
      TraceStyle p_second_style,
      int p_layer,
      double p_center_spacing) {
    List<CoupledCandidate> result = new ArrayList<>();
    add_flow_through_skew_compensated_candidates(
        result,
        p_first_from.get_center().to_float(),
        p_first_to.get_center().to_float(),
        p_second_from.get_center().to_float(),
        p_second_to.get_center().to_float(),
        p_first_style,
        p_second_style,
        p_center_spacing);
    result.addAll(coupled_candidates(
        p_first_from.get_center(),
        p_first_to.get_center(),
        p_second_from.get_center(),
        p_second_to.get_center(),
        p_first_traces,
        p_second_traces,
        p_layer,
        p_center_spacing));

    double[] escapeLengths = {
        mm_to_board(board, 1.0),
        mm_to_board(board, 2.0),
        mm_to_board(board, 3.0),
    };
    for (double escapeLength : escapeLengths) {
      Point firstFromEscape = pin_escape_point(p_first_from, escapeLength);
      Point firstToEscape = pin_escape_point(p_first_to, escapeLength);
      Point secondFromEscape = pin_escape_point(p_second_from, escapeLength);
      Point secondToEscape = pin_escape_point(p_second_to, escapeLength);
      Point firstFromDirectedEscape = directional_pin_escape_point(
          p_first_from,
          midpoint(p_first_to.get_center().to_float(), p_second_to.get_center().to_float()),
          escapeLength);
      Point secondFromDirectedEscape = directional_pin_escape_point(
          p_second_from,
          midpoint(p_first_to.get_center().to_float(), p_second_to.get_center().to_float()),
          escapeLength);
      Point firstToDirectedEscape = directional_pin_escape_point(
          p_first_to,
          midpoint(p_first_from.get_center().to_float(), p_second_from.get_center().to_float()),
          escapeLength);
      Point secondToDirectedEscape = directional_pin_escape_point(
          p_second_to,
          midpoint(p_first_from.get_center().to_float(), p_second_from.get_center().to_float()),
          escapeLength);
      if (firstFromEscape == null || firstToEscape == null || secondFromEscape == null || secondToEscape == null) {
        continue;
      }
      for (CoupledCandidate candidate : coupled_candidates(
          firstFromEscape,
          firstToEscape,
          secondFromEscape,
          secondToEscape,
          p_first_traces,
          p_second_traces,
          p_layer,
          p_center_spacing)) {
        Point[] first = wrap_with_pin_gateways(
            p_first_from.get_center(),
            candidate.first(),
            p_first_to.get_center());
        Point[] second = wrap_with_pin_gateways(
            p_second_from.get_center(),
            candidate.second(),
            p_second_to.get_center());
        if (candidate_geometry_is_safe(first, second, p_center_spacing, candidate.family().startsWith("order_transition_"))) {
          result.add(new CoupledCandidate(
              first,
              second,
              p_center_spacing,
              "pin_gateway_" + candidate.family()));
        }
      }

      if (firstFromDirectedEscape != null
          && secondFromDirectedEscape != null
          && firstToDirectedEscape != null
          && secondToDirectedEscape != null) {
        for (CoupledCandidate candidate : coupled_candidates(
            firstFromDirectedEscape,
            firstToDirectedEscape,
            secondFromDirectedEscape,
            secondToDirectedEscape,
            p_first_traces,
            p_second_traces,
            p_layer,
            p_center_spacing)) {
          Point[] first = wrap_with_pin_gateways(
              p_first_from.get_center(),
              candidate.first(),
              p_first_to.get_center());
          Point[] second = wrap_with_pin_gateways(
              p_second_from.get_center(),
              candidate.second(),
              p_second_to.get_center());
          if (candidate_geometry_is_safe(first, second, p_center_spacing, candidate.family().startsWith("order_transition_"))) {
            result.add(new CoupledCandidate(
                first,
                second,
                p_center_spacing,
                "directed_pin_escape_" + candidate.family()));
          }
        }
      }

      Point firstFromPairEscape = paired_pin_row_escape_point(p_first_from, p_second_from, escapeLength);
      Point secondFromPairEscape = paired_pin_row_escape_point(p_second_from, p_first_from, escapeLength);
      if (firstFromPairEscape != null && secondFromPairEscape != null && firstToEscape != null && secondToEscape != null) {
        for (CoupledCandidate candidate : coupled_candidates(
            firstFromPairEscape,
            firstToEscape,
            secondFromPairEscape,
            secondToEscape,
            p_first_traces,
            p_second_traces,
            p_layer,
            p_center_spacing)) {
          Point[] first = wrap_with_pin_gateways(
              p_first_from.get_center(),
              candidate.first(),
              p_first_to.get_center());
          Point[] second = wrap_with_pin_gateways(
              p_second_from.get_center(),
              candidate.second(),
              p_second_to.get_center());
          if (candidate_geometry_is_safe(first, second, p_center_spacing, candidate.family().startsWith("order_transition_"))) {
            result.add(new CoupledCandidate(
                first,
                second,
                p_center_spacing,
                "source_pin_row_escape_" + candidate.family()));
          }
        }
      }

      Point firstToPairEscape = paired_pin_row_escape_point(p_first_to, p_second_to, escapeLength);
      Point secondToPairEscape = paired_pin_row_escape_point(p_second_to, p_first_to, escapeLength);
      if (firstToPairEscape != null && secondToPairEscape != null && firstFromEscape != null && secondFromEscape != null) {
        for (CoupledCandidate candidate : coupled_candidates(
            firstFromEscape,
            firstToPairEscape,
            secondFromEscape,
            secondToPairEscape,
            p_first_traces,
            p_second_traces,
            p_layer,
            p_center_spacing)) {
          Point[] first = wrap_with_pin_gateways(
              p_first_from.get_center(),
              candidate.first(),
              p_first_to.get_center());
          Point[] second = wrap_with_pin_gateways(
              p_second_from.get_center(),
              candidate.second(),
              p_second_to.get_center());
          if (candidate_geometry_is_safe(first, second, p_center_spacing, candidate.family().startsWith("order_transition_"))) {
            result.add(new CoupledCandidate(
                first,
                second,
                p_center_spacing,
                "target_pin_row_escape_" + candidate.family()));
          }
        }
      }

      if (firstFromPairEscape != null
          && secondFromPairEscape != null
          && firstToPairEscape != null
          && secondToPairEscape != null) {
        for (CoupledCandidate candidate : coupled_candidates(
            firstFromPairEscape,
            firstToPairEscape,
            secondFromPairEscape,
            secondToPairEscape,
            p_first_traces,
            p_second_traces,
            p_layer,
            p_center_spacing)) {
          Point[] first = wrap_with_pin_gateways(
              p_first_from.get_center(),
              candidate.first(),
              p_first_to.get_center());
          Point[] second = wrap_with_pin_gateways(
              p_second_from.get_center(),
              candidate.second(),
              p_second_to.get_center());
          if (candidate_geometry_is_safe(first, second, p_center_spacing, candidate.family().startsWith("order_transition_"))) {
            result.add(new CoupledCandidate(
                first,
                second,
                p_center_spacing,
                "both_pin_row_escape_" + candidate.family()));
          }
        }
      }
    }

    return result;
  }

  private void add_flow_through_skew_compensated_candidates(
      List<CoupledCandidate> p_result,
      FloatPoint p_first_from,
      FloatPoint p_first_to,
      FloatPoint p_second_from,
      FloatPoint p_second_to,
      TraceStyle p_first_style,
      TraceStyle p_second_style,
      double p_center_spacing) {
    FloatPoint sourceCenter = midpoint(p_first_from, p_second_from);
    FloatPoint targetCenter = midpoint(p_first_to, p_second_to);
    double axisX = targetCenter.x - sourceCenter.x;
    double axisY = targetCenter.y - sourceCenter.y;
    double axisLength = Math.sqrt(axisX * axisX + axisY * axisY);
    if (axisLength < mm_to_board(board, 2.0)) {
      return;
    }
    double ux = axisX / axisLength;
    double uy = axisY / axisLength;

    double firstDx = p_first_to.x - p_first_from.x;
    double firstDy = p_first_to.y - p_first_from.y;
    double secondDx = p_second_to.x - p_second_from.x;
    double secondDy = p_second_to.y - p_second_from.y;
    double firstLength = Math.sqrt(firstDx * firstDx + firstDy * firstDy);
    double secondLength = Math.sqrt(secondDx * secondDx + secondDy * secondDy);
    if (firstLength < mm_to_board(board, 2.0) || secondLength < mm_to_board(board, 2.0)) {
      return;
    }
    double firstUx = firstDx / firstLength;
    double firstUy = firstDy / firstLength;
    double secondUx = secondDx / secondLength;
    double secondUy = secondDy / secondLength;
    if ((firstUx * secondUx + firstUy * secondUy) < 0.95) {
      return;
    }

    add_explicit_coupled_candidate(
        p_result,
        new FloatPoint[] { p_first_from, p_first_to },
        new FloatPoint[] { p_second_from, p_second_to },
        p_center_spacing,
        "flow_through_direct");

    boolean tuneFirst = firstLength < secondLength;
    FloatPoint shortFrom = tuneFirst ? p_first_from : p_second_from;
    FloatPoint shortTo = tuneFirst ? p_first_to : p_second_to;
    FloatPoint longFrom = tuneFirst ? p_second_from : p_first_from;
    FloatPoint longTo = tuneFirst ? p_second_to : p_first_to;
    double shortDx = shortTo.x - shortFrom.x;
    double shortDy = shortTo.y - shortFrom.y;
    double shortLength = Math.sqrt(shortDx * shortDx + shortDy * shortDy);
    if (shortLength < mm_to_board(board, 2.0)) {
      return;
    }
    double shortUx = shortDx / shortLength;
    double shortUy = shortDy / shortLength;
    double outwardX = -shortUy;
    double outwardY = shortUx;
    FloatPoint shortMid = midpoint(shortFrom, shortTo);
    FloatPoint longMid = midpoint(longFrom, longTo);
    double towardCompanionX = longMid.x - shortMid.x;
    double towardCompanionY = longMid.y - shortMid.y;
    if ((outwardX * towardCompanionX + outwardY * towardCompanionY) > 0.0) {
      outwardX = -outwardX;
      outwardY = -outwardY;
    }

    double firstSourceProjection = projection_along_axis(p_first_from, sourceCenter, ux, uy);
    double secondSourceProjection = projection_along_axis(p_second_from, sourceCenter, ux, uy);
    double firstTargetProjection = projection_along_axis(p_first_to, sourceCenter, ux, uy);
    double secondTargetProjection = projection_along_axis(p_second_to, sourceCenter, ux, uy);
    double sourceMismatch = Math.abs(firstSourceProjection - secondSourceProjection);
    double targetMismatch = Math.abs(firstTargetProjection - secondTargetProjection);
    boolean nearTarget = targetMismatch >= sourceMismatch;

    TraceStyle tunedStyle = tuneFirst ? p_first_style : p_second_style;
    double traceWidth = tunedStyle == null ? 0.0 : tunedStyle.halfWidth() * 2.0;
    double targetShortLength = Math.max(firstLength, secondLength) + traceWidth * 2.0;
    double minBumpWidth = Math.max(
        mm_to_board(board, FLOW_THROUGH_MIN_BUMP_WIDTH_MM),
        Math.max(traceWidth * 4.0, p_center_spacing * 0.50));
    double minSpacing = Math.max(
        mm_to_board(board, FLOW_THROUGH_MIN_BUMP_SPACING_MM),
        Math.max(traceWidth * 4.0, p_center_spacing * 0.50));
    double minPlateau = Math.max(mm_to_board(board, FLOW_THROUGH_MIN_PLATEAU_MM), traceWidth * 4.0);
    double minHeight = Math.max(
        mm_to_board(board, FLOW_THROUGH_MIN_BUMP_HEIGHT_MM),
        traceWidth);
    double maxHeight = Math.max(minHeight, mm_to_board(board, FLOW_THROUGH_MAX_BUMP_HEIGHT_MM));
    List<FlowThroughBumpCandidate> bumpCandidates = flow_through_primitive_candidates(
        shortFrom,
        shortTo,
        outwardX,
        outwardY,
        shortLength,
        targetShortLength,
        minBumpWidth,
        minSpacing,
        minPlateau,
        minHeight,
        maxHeight,
        nearTarget);
    double targetResidualSkew = mm_to_board(board, 0.10);
    bumpCandidates.sort(Comparator
        .comparing((FlowThroughBumpCandidate candidate) -> candidate.residualSkew() > targetResidualSkew)
        .thenComparing(Comparator.comparingDouble(FlowThroughBumpCandidate::height))
        .thenComparing(Comparator.comparingInt(FlowThroughBumpCandidate::bumpCount).reversed())
        .thenComparingDouble(FlowThroughBumpCandidate::residualSkew));
    int accepted = 0;
    for (FlowThroughBumpCandidate bumpCandidate : bumpCandidates) {
      boolean added;
      if (tuneFirst) {
        added = add_explicit_coupled_candidate(
            p_result,
            float_points(bumpCandidate.points()),
            new FloatPoint[] { p_second_from, p_second_to },
            p_center_spacing,
            "flow_through_skew_compensated");
      } else {
        added = add_explicit_coupled_candidate(
            p_result,
            new FloatPoint[] { p_first_from, p_first_to },
            float_points(bumpCandidate.points()),
            p_center_spacing,
            "flow_through_skew_compensated");
      }
      if (!added) {
        continue;
      }
      accepted++;
      if (accepted >= 8) {
        break;
      }
    }
  }

  private List<FlowThroughBumpCandidate> flow_through_primitive_candidates(
      FloatPoint p_from,
      FloatPoint p_to,
      double p_outward_x,
      double p_outward_y,
      double p_short_length,
      double p_target_short_length,
      double p_min_bump_width,
      double p_min_spacing,
      double p_min_plateau,
      double p_min_height,
      double p_max_height,
      boolean p_near_target) {
    List<FlowThroughBumpCandidate> result = new ArrayList<>();
    double desiredExtra = p_target_short_length - p_short_length;
    if (desiredExtra <= 0.0) {
      return result;
    }
    int maxCount = max_fit_flow_through_bump_count(
        p_from,
        p_to,
        p_outward_x,
        p_outward_y,
        p_min_bump_width,
        p_min_spacing,
        p_min_plateau,
        p_min_height,
        p_near_target);
    for (int bumpCount = maxCount; bumpCount >= 2; bumpCount--) {
      FlowThroughBumpCandidate best = null;
      double lowHeight = p_min_height;
      double highHeight = p_max_height;
      for (int iteration = 0; iteration < 28; iteration++) {
        double height = (lowHeight + highHeight) / 2.0;
        FlowThroughBumpCandidate uniform = flow_through_candidate_for_heights(
            p_from,
            p_to,
            p_outward_x,
            p_outward_y,
            p_short_length,
            p_target_short_length,
            repeated_heights(bumpCount, height),
            p_min_bump_width,
            p_min_spacing,
            p_min_plateau,
            p_max_height,
            p_near_target);
        if (uniform == null) {
          highHeight = height;
          continue;
        }
        best = better_flow_through_candidate(best, uniform);
        double addedLength = point_path_length(uniform.points()) - p_short_length;
        if (addedLength < desiredExtra) {
          lowHeight = height;
        } else {
          highHeight = height;
        }
      }
      if (best != null) {
        result.add(best);
        result.addAll(trimmed_finish_flow_through_candidates(
            best,
            p_from,
            p_to,
            p_outward_x,
            p_outward_y,
            p_short_length,
            p_target_short_length,
            p_min_bump_width,
            p_min_spacing,
            p_min_plateau,
            p_min_height,
            p_max_height,
            p_near_target));
      }
    }
    return result;
  }

  int flow_through_primitive_candidate_count_for_test(
      FloatPoint p_from,
      FloatPoint p_to,
      double p_outward_x,
      double p_outward_y,
      double p_short_length,
      double p_target_short_length,
      double p_min_bump_width,
      double p_min_spacing,
      double p_min_plateau,
      double p_min_height,
      double p_max_height,
      boolean p_near_target) {
    return flow_through_primitive_candidates(
        p_from,
        p_to,
        p_outward_x,
        p_outward_y,
        p_short_length,
        p_target_short_length,
        p_min_bump_width,
        p_min_spacing,
        p_min_plateau,
        p_min_height,
        p_max_height,
        p_near_target).size();
  }

  int max_fit_flow_through_bump_count_for_test(
      FloatPoint p_from,
      FloatPoint p_to,
      double p_outward_x,
      double p_outward_y,
      double p_min_bump_width,
      double p_min_spacing,
      double p_min_plateau,
      double p_min_height,
      boolean p_near_target) {
    return max_fit_flow_through_bump_count(
        p_from,
        p_to,
        p_outward_x,
        p_outward_y,
        p_min_bump_width,
        p_min_spacing,
        p_min_plateau,
        p_min_height,
        p_near_target);
  }

  private List<FlowThroughBumpCandidate> trimmed_finish_flow_through_candidates(
      FlowThroughBumpCandidate p_base,
      FloatPoint p_from,
      FloatPoint p_to,
      double p_outward_x,
      double p_outward_y,
      double p_short_length,
      double p_target_short_length,
      double p_min_bump_width,
      double p_min_spacing,
      double p_min_plateau,
      double p_min_height,
      double p_max_height,
      boolean p_near_target) {
    List<FlowThroughBumpCandidate> result = new ArrayList<>();
    int bumpCount = p_base.bumpCount();
    if (bumpCount < 2) {
      return result;
    }
    double fullHeight = Math.max(p_min_height, p_base.maxExcursion());
    double lowHeight = p_min_height;
    double highHeight = Math.min(p_max_height, fullHeight);
    FlowThroughBumpCandidate best = null;
    for (int iteration = 0; iteration < 24; iteration++) {
      double finishHeight = (lowHeight + highHeight) / 2.0;
      double[] heights = repeated_heights(bumpCount, fullHeight);
      heights[bumpCount - 1] = finishHeight;
      FlowThroughBumpCandidate candidate = flow_through_candidate_for_heights(
          p_from,
          p_to,
          p_outward_x,
          p_outward_y,
          p_short_length,
          p_target_short_length,
          heights,
          p_min_bump_width,
          p_min_spacing,
          p_min_plateau,
          p_max_height,
          p_near_target);
      if (candidate == null) {
        highHeight = finishHeight;
        continue;
      }
      best = better_flow_through_candidate(best, candidate);
      double addedLength = point_path_length(candidate.points()) - p_short_length;
      if ((p_short_length + addedLength) < p_target_short_length) {
        lowHeight = finishHeight;
      } else {
        highHeight = finishHeight;
      }
    }
    if (best != null) {
      result.add(best);
    }
    return result;
  }

  private FlowThroughBumpCandidate flow_through_candidate_for_heights(
      FloatPoint p_from,
      FloatPoint p_to,
      double p_outward_x,
      double p_outward_y,
      double p_short_length,
      double p_target_short_length,
      double[] p_heights,
      double p_min_bump_width,
      double p_min_spacing,
      double p_min_plateau,
      double p_max_height,
      boolean p_near_target) {
    double maxHeight = max_height(p_heights);
    double spacing = rounded_bump_spacing(maxHeight, p_min_plateau, p_min_spacing);
    Point[] tuned = rounded_outward_bump_path(
        p_from,
        p_to,
        p_outward_x,
        p_outward_y,
        p_heights,
        p_min_bump_width,
        p_min_plateau,
        spacing,
        p_near_target);
    if (tuned == null) {
      return null;
    }
    FlowThroughBumpShape shape = validate_flow_through_bump_shape(
        tuned,
        p_from,
        p_to,
        p_outward_x,
        p_outward_y,
        p_heights.length,
        min_height(p_heights),
        p_min_plateau,
        p_min_spacing,
        Math.min(p_max_height, spacing * 3.0));
    if (shape == null) {
      return null;
    }
    double addedLength = point_path_length(tuned) - p_short_length;
    double residualSkew = Math.abs((p_short_length + addedLength) - p_target_short_length);
    return new FlowThroughBumpCandidate(tuned, residualSkew, shape);
  }

  private int max_fit_flow_through_bump_count(
      FloatPoint p_from,
      FloatPoint p_to,
      double p_outward_x,
      double p_outward_y,
      double p_min_bump_width,
      double p_min_spacing,
      double p_min_plateau,
      double p_min_height,
      boolean p_near_target) {
    int result = 0;
    double length = p_from.distance(p_to);
    double spacing = rounded_bump_spacing(p_min_height, p_min_plateau, p_min_spacing);
    double width = Math.max(p_min_bump_width, rounded_bump_width(p_min_height, p_min_plateau, spacing));
    if (!Double.isFinite(spacing) || !Double.isFinite(width)) {
      return 0;
    }
    double margin = Math.max(mm_to_board(board, 0.30), spacing * 0.50);
    for (int bumpCount = 2; bumpCount <= 12; bumpCount++) {
      double span = (width * bumpCount) + (spacing * (bumpCount - 1.0));
      if (span + margin * 2.0 >= length) {
        break;
      }
      result = bumpCount;
    }
    return result;
  }

  private static FlowThroughBumpCandidate better_flow_through_candidate(
      FlowThroughBumpCandidate p_current,
      FlowThroughBumpCandidate p_candidate) {
    return p_current == null || p_candidate.score() < p_current.score() ? p_candidate : p_current;
  }

  private static double[] repeated_heights(int p_count, double p_height) {
    double[] result = new double[p_count];
    for (int i = 0; i < result.length; i++) {
      result[i] = p_height;
    }
    return result;
  }

  private static double min_height(double[] p_heights) {
    double result = Double.POSITIVE_INFINITY;
    for (double height : p_heights) {
      result = Math.min(result, height);
    }
    return result;
  }

  private static double max_height(double[] p_heights) {
    double result = 0.0;
    for (double height : p_heights) {
      result = Math.max(result, height);
    }
    return result;
  }

  Point[] rounded_outward_bump_path(
      FloatPoint p_from,
      FloatPoint p_to,
      double p_outward_x,
      double p_outward_y,
      int p_bump_count,
      double p_height,
      double p_width,
      double p_min_plateau,
      double p_spacing,
      boolean p_near_target) {
    return rounded_outward_bump_path(
        p_from,
        p_to,
        p_outward_x,
        p_outward_y,
        repeated_heights(p_bump_count, p_height),
        p_width,
        p_min_plateau,
        p_spacing,
        p_near_target);
  }

  private Point[] rounded_outward_bump_path(
      FloatPoint p_from,
      FloatPoint p_to,
      double p_outward_x,
      double p_outward_y,
      double[] p_heights,
      double p_min_bump_width,
      double p_min_plateau,
      double p_spacing,
      boolean p_near_target) {
    double dx = p_to.x - p_from.x;
    double dy = p_to.y - p_from.y;
    double length = Math.sqrt(dx * dx + dy * dy);
    if (length <= 0.0 || p_heights.length <= 0) {
      return null;
    }
    double ux = dx / length;
    double uy = dy / length;
    double margin = Math.max(mm_to_board(board, 0.30), p_spacing * 0.50);
    double[] widths = new double[p_heights.length];
    double[] fillets = new double[p_heights.length];
    double totalSpan = (p_heights.length - 1.0) * p_spacing;
    for (int i = 0; i < p_heights.length; i++) {
      double height = p_heights[i];
      double width = Math.max(p_min_bump_width, rounded_bump_width(height, p_min_plateau, p_spacing));
      double fillet = rounded_bump_fillet(height, p_min_plateau, p_spacing);
      double plateau = width - (height * 2.0);
      if (!Double.isFinite(width)
          || !Double.isFinite(fillet)
          || height <= 0.0
          || plateau < p_min_plateau
          || plateau - (2.0 * fillet) < p_min_plateau) {
        return null;
      }
      widths[i] = width;
      fillets[i] = fillet;
      totalSpan += width;
    }
    if (totalSpan + margin * 2.0 >= length) {
      return null;
    }
    double startDistance = p_near_target ? length - margin - totalSpan : margin;
    double rampScale = 1.0 / Math.sqrt(2.0);
    double rampUpX = (ux + p_outward_x) * rampScale;
    double rampUpY = (uy + p_outward_y) * rampScale;
    double rampDownX = (ux - p_outward_x) * rampScale;
    double rampDownY = (uy - p_outward_y) * rampScale;
    List<Point> points = new ArrayList<>();
    append_distinct(points, p_from.round());
    double bumpStart = startDistance;
    for (int i = 0; i < p_heights.length; i++) {
      double height = p_heights[i];
      double width = widths[i];
      double fillet = fillets[i];
      double bumpEnd = bumpStart + width;
      FloatPoint baseStart = shift_point(p_from, ux * bumpStart, uy * bumpStart);
      FloatPoint topStart = shift_point(
          p_from,
          ux * (bumpStart + height) + p_outward_x * height,
          uy * (bumpStart + height) + p_outward_y * height);
      FloatPoint plateauEnd = shift_point(
          p_from,
          ux * (bumpEnd - height) + p_outward_x * height,
          uy * (bumpEnd - height) + p_outward_y * height);
      FloatPoint baseEnd = shift_point(p_from, ux * bumpEnd, uy * bumpEnd);
      append_distinct(points, shift_point(baseStart, -ux * fillet, -uy * fillet).round());
      append_rounded_corner(points, baseStart, ux, uy, rampUpX, rampUpY, fillet);
      append_distinct(points, shift_point(topStart, -rampUpX * fillet, -rampUpY * fillet).round());
      append_rounded_corner(points, topStart, rampUpX, rampUpY, ux, uy, fillet);
      append_distinct(points, shift_point(plateauEnd, -ux * fillet, -uy * fillet).round());
      append_rounded_corner(points, plateauEnd, ux, uy, rampDownX, rampDownY, fillet);
      append_distinct(points, shift_point(baseEnd, -rampDownX * fillet, -rampDownY * fillet).round());
      append_rounded_corner(points, baseEnd, rampDownX, rampDownY, ux, uy, fillet);
      bumpStart = bumpEnd + p_spacing;
    }
    append_distinct(points, p_to.round());
    return points.size() >= 2 ? points.toArray(Point[]::new) : null;
  }

  private static void append_rounded_corner(
      List<Point> p_points,
      FloatPoint p_corner,
      double p_in_x,
      double p_in_y,
      double p_out_x,
      double p_out_y,
      double p_tangent_length) {
    FloatPoint start = shift_point(p_corner, -p_in_x * p_tangent_length, -p_in_y * p_tangent_length);
    FloatPoint end = shift_point(p_corner, p_out_x * p_tangent_length, p_out_y * p_tangent_length);
    FloatPoint center = circular_fillet_center(start, p_in_x, p_in_y, end, p_out_x, p_out_y);
    if (center == null) {
      append_distinct(p_points, end.round());
      return;
    }
    double startAngle = Math.atan2(start.y - center.y, start.x - center.x);
    double endAngle = Math.atan2(end.y - center.y, end.x - center.x);
    double sweep = endAngle - startAngle;
    while (sweep <= -Math.PI) {
      sweep += 2.0 * Math.PI;
    }
    while (sweep > Math.PI) {
      sweep -= 2.0 * Math.PI;
    }
    double radius = center.distance(start);
    if (radius <= 0.0) {
      append_distinct(p_points, end.round());
      return;
    }
    for (int sample = 1; sample <= FLOW_THROUGH_ROUNDED_RAMP_SAMPLES; sample++) {
      double t = (double) sample / FLOW_THROUGH_ROUNDED_RAMP_SAMPLES;
      double angle = startAngle + sweep * t;
      FloatPoint point = new FloatPoint(
          center.x + Math.cos(angle) * radius,
          center.y + Math.sin(angle) * radius);
      append_distinct(p_points, point.round());
    }
  }

  private static FloatPoint circular_fillet_center(
      FloatPoint p_start,
      double p_in_x,
      double p_in_y,
      FloatPoint p_end,
      double p_out_x,
      double p_out_y) {
    FloatPoint best = null;
    double bestSweep = Double.POSITIVE_INFINITY;
    for (int startSign : new int[] { -1, 1 }) {
      double startNormalX = -p_in_y * startSign;
      double startNormalY = p_in_x * startSign;
      for (int endSign : new int[] { -1, 1 }) {
        double endNormalX = -p_out_y * endSign;
        double endNormalY = p_out_x * endSign;
        double determinant = (startNormalX * -endNormalY) - (startNormalY * -endNormalX);
        if (Math.abs(determinant) <= 1e-9) {
          continue;
        }
        double dx = p_end.x - p_start.x;
        double dy = p_end.y - p_start.y;
        double startScale = ((dx * -endNormalY) - (dy * -endNormalX)) / determinant;
        FloatPoint center = shift_point(p_start, startNormalX * startScale, startNormalY * startScale);
        double startRadius = center.distance(p_start);
        double endRadius = center.distance(p_end);
        if (startRadius <= 0.0 || Math.abs(startRadius - endRadius) > Math.max(1e-6, startRadius * 0.01)) {
          continue;
        }
        double startAngle = Math.atan2(p_start.y - center.y, p_start.x - center.x);
        double endAngle = Math.atan2(p_end.y - center.y, p_end.x - center.x);
        double sweep = Math.abs(normalized_radian_delta(endAngle - startAngle));
        if (sweep < bestSweep) {
          bestSweep = sweep;
          best = center;
        }
      }
    }
    return best;
  }

  private static double normalized_radian_delta(double p_angle) {
    double result = p_angle;
    while (result <= -Math.PI) {
      result += 2.0 * Math.PI;
    }
    while (result > Math.PI) {
      result -= 2.0 * Math.PI;
    }
    return result;
  }

  boolean valid_flow_through_bump_shape_for_test(
      Point[] p_points,
      FloatPoint p_from,
      FloatPoint p_to,
      double p_outward_x,
      double p_outward_y,
      int p_bump_count,
      double p_expected_height,
      double p_min_plateau,
      double p_min_self_spacing,
      double p_max_excursion) {
    return validate_flow_through_bump_shape(
        p_points,
        p_from,
        p_to,
        p_outward_x,
        p_outward_y,
        p_bump_count,
        p_expected_height,
        p_min_plateau,
        p_min_self_spacing,
        p_max_excursion) != null;
  }

  private FlowThroughBumpShape validate_flow_through_bump_shape(
      Point[] p_points,
      FloatPoint p_from,
      FloatPoint p_to,
      double p_outward_x,
      double p_outward_y,
      int p_bump_count,
      double p_expected_height,
      double p_min_plateau,
      double p_min_self_spacing,
      double p_max_excursion) {
    if (p_points.length < 2 || p_bump_count <= 0) {
      return null;
    }
    double dx = p_to.x - p_from.x;
    double dy = p_to.y - p_from.y;
    double length = Math.sqrt(dx * dx + dy * dy);
    if (length <= 0.0) {
      return null;
    }
    double ux = dx / length;
    double uy = dy / length;
    double minOutward = Math.max(0.0, p_expected_height * 0.80);
    double maxOutward = Math.max(minOutward, p_max_excursion);
    int plateauCount = 0;
    boolean inPlateau = false;
    double currentPlateauLength = 0.0;
    double minPlateau = Double.POSITIVE_INFINITY;
    double maxExcursion = 0.0;
    List<double[]> bumpIntervals = new ArrayList<>();
    double intervalStart = Double.NaN;
    double intervalEnd = Double.NaN;
    for (int i = 0; i < p_points.length; i++) {
      FloatPoint point = p_points[i].to_float();
      double along = projection_along_axis(point, p_from, ux, uy);
      double excursion = ((point.x - p_from.x) * p_outward_x) + ((point.y - p_from.y) * p_outward_y);
      if (excursion < -mm_to_board(board, 0.01) || excursion > maxOutward + mm_to_board(board, 0.01)) {
        return null;
      }
      if (along < -mm_to_board(board, 0.01) || along > length + mm_to_board(board, 0.01)) {
        return null;
      }
      maxExcursion = Math.max(maxExcursion, excursion);
      if (excursion > mm_to_board(board, 0.02)) {
        if (Double.isNaN(intervalStart)) {
          intervalStart = along;
        }
        intervalEnd = along;
      } else if (!Double.isNaN(intervalStart)) {
        bumpIntervals.add(new double[] { intervalStart, intervalEnd });
        intervalStart = Double.NaN;
        intervalEnd = Double.NaN;
      }
    }
    if (!Double.isNaN(intervalStart)) {
      bumpIntervals.add(new double[] { intervalStart, intervalEnd });
    }
    if (bumpIntervals.size() != p_bump_count) {
      return null;
    }
    for (int i = 1; i < bumpIntervals.size(); i++) {
      if (bumpIntervals.get(i)[0] - bumpIntervals.get(i - 1)[1] < p_min_self_spacing) {
        return null;
      }
    }
    for (int i = 0; i < p_points.length - 1; i++) {
      FloatPoint start = p_points[i].to_float();
      FloatPoint end = p_points[i + 1].to_float();
      double sx = end.x - start.x;
      double sy = end.y - start.y;
      double segmentLength = Math.sqrt(sx * sx + sy * sy);
      if (segmentLength <= 0.0) {
        continue;
      }
      double segmentAlong = Math.abs((sx * ux) + (sy * uy));
      double segmentOutward = (sx * p_outward_x) + (sy * p_outward_y);
      double segmentForward = (sx * ux) + (sy * uy);
      double startExcursion = ((start.x - p_from.x) * p_outward_x) + ((start.y - p_from.y) * p_outward_y);
      double endExcursion = ((end.x - p_from.x) * p_outward_x) + ((end.y - p_from.y) * p_outward_y);
      double angle = Math.toDegrees(Math.atan2(Math.abs(segmentOutward), segmentAlong));
      boolean baseline = Math.abs(startExcursion) <= mm_to_board(board, 0.02)
          && Math.abs(endExcursion) <= mm_to_board(board, 0.02);
      boolean plateau = startExcursion >= minOutward
          && endExcursion >= minOutward
          && angle <= FLOW_THROUGH_MAX_RAMP_ANGLE_ERROR_DEG;
      boolean ramp = Math.abs(angle - 45.0) <= FLOW_THROUGH_MAX_RAMP_ANGLE_ERROR_DEG
          || angle <= FLOW_THROUGH_MAX_ROUNDED_SEGMENT_ANGLE_DEG;
      if (baseline) {
        if (inPlateau) {
          minPlateau = Math.min(minPlateau, currentPlateauLength);
          currentPlateauLength = 0.0;
          inPlateau = false;
        }
        continue;
      }
      if (plateau) {
        if (!inPlateau) {
          plateauCount++;
          inPlateau = true;
        }
        currentPlateauLength += segmentLength;
        continue;
      }
      if (inPlateau) {
        minPlateau = Math.min(minPlateau, currentPlateauLength);
        currentPlateauLength = 0.0;
        inPlateau = false;
      }
      if (segmentForward < -mm_to_board(board, 0.01) || !ramp || segmentOutward == 0.0) {
        return null;
      }
    }
    if (inPlateau) {
      minPlateau = Math.min(minPlateau, currentPlateauLength);
    }
    if (plateauCount != p_bump_count || minPlateau < p_min_plateau) {
      return null;
    }
    for (int i = 1; i < p_points.length - 1; i++) {
      double turn = path_turn_angle_degrees(
          p_points[i - 1].to_float(),
          p_points[i].to_float(),
          p_points[i + 1].to_float());
      if (turn > FLOW_THROUGH_MAX_BEND_ANGLE_DEG) {
        return null;
      }
      double beforeExcursion = ((p_points[i - 1].to_float().x - p_from.x) * p_outward_x)
          + ((p_points[i - 1].to_float().y - p_from.y) * p_outward_y);
      double cornerExcursion = ((p_points[i].to_float().x - p_from.x) * p_outward_x)
          + ((p_points[i].to_float().y - p_from.y) * p_outward_y);
      double afterExcursion = ((p_points[i + 1].to_float().x - p_from.x) * p_outward_x)
          + ((p_points[i + 1].to_float().y - p_from.y) * p_outward_y);
      boolean onRoundedTransition = Math.abs(beforeExcursion - cornerExcursion) > mm_to_board(board, 0.01)
          || Math.abs(afterExcursion - cornerExcursion) > mm_to_board(board, 0.01);
      if (onRoundedTransition && turn > FLOW_THROUGH_MAX_ROUNDED_TURN_DEG) {
        return null;
      }
    }
    return new FlowThroughBumpShape(p_bump_count, p_expected_height, minPlateau, maxExcursion);
  }

  private static double path_turn_angle_degrees(FloatPoint p_before, FloatPoint p_corner, FloatPoint p_after) {
    double ax = p_corner.x - p_before.x;
    double ay = p_corner.y - p_before.y;
    double bx = p_after.x - p_corner.x;
    double by = p_after.y - p_corner.y;
    double aLength = Math.sqrt(ax * ax + ay * ay);
    double bLength = Math.sqrt(bx * bx + by * by);
    if (aLength <= 0.0 || bLength <= 0.0) {
      return 0.0;
    }
    double cos = ((ax * bx) + (ay * by)) / (aLength * bLength);
    cos = Math.max(-1.0, Math.min(1.0, cos));
    return Math.toDegrees(Math.acos(cos));
  }

  private static FloatPoint[] float_points(Point[] p_points) {
    FloatPoint[] result = new FloatPoint[p_points.length];
    for (int i = 0; i < p_points.length; i++) {
      result[i] = p_points[i].to_float();
    }
    return result;
  }

  private static double projection_along_axis(FloatPoint p_point, FloatPoint p_origin, double p_axis_x, double p_axis_y) {
    return (p_point.x - p_origin.x) * p_axis_x + (p_point.y - p_origin.y) * p_axis_y;
  }

  private double rounded_bump_width(double p_height, double p_min_plateau, double p_spacing) {
    double fillet = rounded_bump_fillet(p_height, p_min_plateau, p_spacing);
    if (!Double.isFinite(fillet)) {
      return Double.POSITIVE_INFINITY;
    }
    return (2.0 * p_height) + p_min_plateau + (2.0 * fillet);
  }

  private double rounded_bump_spacing(double p_height, double p_min_plateau, double p_min_self_spacing) {
    double spacing = p_min_self_spacing;
    for (int i = 0; i < 4; i++) {
      double fillet = rounded_bump_fillet(p_height, p_min_plateau, spacing);
      if (!Double.isFinite(fillet)) {
        return Double.POSITIVE_INFINITY;
      }
      spacing = p_min_self_spacing + (2.0 * fillet);
    }
    return spacing;
  }

  private double rounded_bump_fillet(double p_height, double p_min_plateau, double p_spacing) {
    double minRadius = Math.max(mm_to_board(board, 0.02), p_min_plateau / 8.0);
    double maxRadius = Math.min(p_height / 2.0, p_spacing / 2.0);
    if (maxRadius < minRadius) {
      return Double.POSITIVE_INFINITY;
    }
    double radius = clamp((p_spacing * FLOW_THROUGH_KICAD_CORNER_RADIUS_PERCENT) / 200.0, minRadius, maxRadius);
    double fillet = radius * Math.tan(Math.toRadians(22.5));
    if (radius < minRadius || fillet <= mm_to_board(board, 0.01) || fillet > p_height * 0.45) {
      return Double.POSITIVE_INFINITY;
    }
    return fillet;
  }

  private static double clamp(double p_value, double p_min, double p_max) {
    return Math.max(p_min, Math.min(p_max, p_value));
  }

  private List<MissingMemberCandidate> missing_member_candidates_from_companion_trace(
      Pin p_missing_from,
      Pin p_missing_to,
      List<PolylineTrace> p_companion_traces,
      int p_layer,
      double p_center_spacing) {
    List<MissingMemberCandidate> result = new ArrayList<>();
    for (PolylineTrace companion : p_companion_traces) {
      if (companion.get_layer() != p_layer) {
        continue;
      }
      Point[] companionCorners = companion.polyline().corner_arr();
      if (companionCorners.length < 2) {
        continue;
      }
      for (boolean reverse : new boolean[] { false, true }) {
        Point[] orderedCompanion = reverse ? reversed_points(companionCorners) : companionCorners;
        double[] endpointTrims = {
            0.0,
            mm_to_board(board, 0.80),
            mm_to_board(board, 1.50),
            mm_to_board(board, 2.50),
            mm_to_board(board, 4.00),
        };
        for (double endpointTrim : endpointTrims) {
          Point[] companionSpine = endpointTrim <= 0.0
              ? orderedCompanion
              : trim_polyline_endpoints(orderedCompanion, endpointTrim);
          if (companionSpine == null || companionSpine.length < 2) {
            continue;
          }
          for (int sign : new int[] { 1, -1 }) {
            Point[] offset = offset_companion_polyline(companionSpine, p_center_spacing * sign);
            if (offset == null || offset.length < 2) {
              continue;
            }
            Point[] candidate = wrap_with_pin_gateways(
                p_missing_from.get_center(),
                offset,
                p_missing_to.get_center());
            if (candidate_geometry_is_safe(candidate, companionSpine, p_center_spacing, true)) {
              result.add(new MissingMemberCandidate(
                  candidate,
                  p_center_spacing,
                  reverse
                      ? "companion_offset_trimmed_reverse"
                      : "companion_offset_trimmed"));
            }

            Point fromExit = pin_escape_point(
                p_missing_from,
                offset,
                Math.max(1, companion.get_half_width()),
                p_layer);
            Point toExit = pin_escape_point(
                p_missing_to,
                offset,
                Math.max(1, companion.get_half_width()),
                p_layer);
            Point[] escapedCandidate = wrap_with_pin_gateways(fromExit, offset, toExit);
            if (candidate_geometry_is_safe(escapedCandidate, companionSpine, p_center_spacing, true)) {
              result.add(new MissingMemberCandidate(
                  escapedCandidate,
                  p_center_spacing,
                  reverse
                      ? "companion_offset_pin_exit_only_trimmed_reverse"
                      : "companion_offset_pin_exit_only_trimmed"));
            }
            Point[] escapedWithPads = wrap_with_pin_gateways(
                p_missing_from.get_center(),
                escapedCandidate,
                p_missing_to.get_center());
            if (candidate_geometry_is_safe(escapedWithPads, companionSpine, p_center_spacing, true)) {
              result.add(new MissingMemberCandidate(
                  escapedWithPads,
                  p_center_spacing,
                  reverse
                      ? "companion_offset_pin_exit_trimmed_reverse"
                      : "companion_offset_pin_exit_trimmed"));
            }
          }
        }
      }
    }
    return result;
  }

  private static Point[] reversed_points(Point[] p_points) {
    Point[] result = new Point[p_points.length];
    for (int i = 0; i < p_points.length; i++) {
      result[i] = p_points[p_points.length - 1 - i];
    }
    return result;
  }

  private static Point[] trim_polyline_endpoints(Point[] p_points, double p_trim) {
    if (p_points.length < 2 || p_trim <= 0.0) {
      return p_points;
    }
    List<Point> result = new ArrayList<>();
    Point start = point_along_polyline(p_points, p_trim, false);
    Point end = point_along_polyline(p_points, p_trim, true);
    if (start == null || end == null || start.to_float().distance(end.to_float()) <= 0.0) {
      return null;
    }
    append_distinct(result, start);
    double walked = 0.0;
    for (int i = 1; i < p_points.length - 1; i++) {
      double fromStart = walked + p_points[i - 1].to_float().distance(p_points[i].to_float());
      double toEnd = remaining_polyline_length(p_points, i);
      if (fromStart > p_trim && toEnd > p_trim) {
        append_distinct(result, p_points[i]);
      }
      walked = fromStart;
    }
    append_distinct(result, end);
    return result.size() >= 2 ? result.toArray(Point[]::new) : null;
  }

  private static Point point_along_polyline(Point[] p_points, double p_distance, boolean p_from_end) {
    double remaining = p_distance;
    for (int offset = 0; offset < p_points.length - 1; offset++) {
      int i = p_from_end ? p_points.length - 1 - offset : offset;
      int j = p_from_end ? i - 1 : i + 1;
      FloatPoint from = p_points[i].to_float();
      FloatPoint to = p_points[j].to_float();
      double segmentLength = from.distance(to);
      if (segmentLength <= 0.0) {
        continue;
      }
      if (remaining <= segmentLength) {
        double ratio = remaining / segmentLength;
        return new FloatPoint(
            from.x + (to.x - from.x) * ratio,
            from.y + (to.y - from.y) * ratio).round();
      }
      remaining -= segmentLength;
    }
    return null;
  }

  private static double remaining_polyline_length(Point[] p_points, int p_index) {
    double result = 0.0;
    for (int i = p_index; i < p_points.length - 1; i++) {
      result += p_points[i].to_float().distance(p_points[i + 1].to_float());
    }
    return result;
  }

  private static Point[] offset_companion_polyline(Point[] p_points, double p_offset) {
    List<Point> result = new ArrayList<>();
    for (int i = 0; i < p_points.length; i++) {
      FloatPoint current = p_points[i].to_float();
      FloatPoint normal = offset_normal_at_corner(p_points, i);
      if (normal == null) {
        return null;
      }
      append_distinct(result, shift_point(current, normal.x * p_offset, normal.y * p_offset).round());
    }
    return result.toArray(Point[]::new);
  }

  private static FloatPoint offset_normal_at_corner(Point[] p_points, int p_index) {
    FloatPoint normal = null;
    if (p_index > 0) {
      normal = add_normals(normal, segment_normal(p_points[p_index - 1].to_float(), p_points[p_index].to_float()));
    }
    if (p_index < p_points.length - 1) {
      normal = add_normals(normal, segment_normal(p_points[p_index].to_float(), p_points[p_index + 1].to_float()));
    }
    if (normal == null) {
      return null;
    }
    double length = Math.sqrt(normal.x * normal.x + normal.y * normal.y);
    if (length <= 0.0) {
      return null;
    }
    return new FloatPoint(normal.x / length, normal.y / length);
  }

  private static FloatPoint add_normals(FloatPoint p_first, FloatPoint p_second) {
    if (p_second == null) {
      return p_first;
    }
    if (p_first == null) {
      return p_second;
    }
    return new FloatPoint(p_first.x + p_second.x, p_first.y + p_second.y);
  }

  private static FloatPoint segment_normal(FloatPoint p_from, FloatPoint p_to) {
    double dx = p_to.x - p_from.x;
    double dy = p_to.y - p_from.y;
    double length = Math.sqrt(dx * dx + dy * dy);
    if (length <= 0.0) {
      return null;
    }
    return new FloatPoint(-dy / length, dx / length);
  }

  private Point paired_pin_row_escape_point(Pin p_pin, Pin p_mate_pin, double p_escape_length) {
    if (p_pin == null
        || p_mate_pin == null
        || p_pin.get_component_no() <= 0
        || p_pin.get_component_no() != p_mate_pin.get_component_no()
        || board.components == null) {
      return null;
    }
    Component component = board.components.get(p_pin.get_component_no());
    if (component == null || component.get_location() == null) {
      return null;
    }
    FloatPoint pinCenter = p_pin.get_center().to_float();
    FloatPoint mateCenter = p_mate_pin.get_center().to_float();
    FloatPoint componentCenter = component.get_location().to_float();
    FloatPoint pairCenter = midpoint(pinCenter, mateCenter);
    double pairDx = mateCenter.x - pinCenter.x;
    double pairDy = mateCenter.y - pinCenter.y;
    double dx = pairCenter.x - componentCenter.x;
    double dy = pairCenter.y - componentCenter.y;
    if (Math.abs(pairDx) < Math.abs(pairDy) * 0.35 && Math.abs(dx) > 0.0) {
      return shift_point(pinCenter, Math.signum(dx) * p_escape_length, 0.0).round();
    }
    if (Math.abs(pairDy) < Math.abs(pairDx) * 0.35 && Math.abs(dy) > 0.0) {
      return shift_point(pinCenter, 0.0, Math.signum(dy) * p_escape_length).round();
    }
    double length = Math.sqrt(dx * dx + dy * dy);
    if (length <= 0.0) {
      return null;
    }
    return shift_point(pinCenter, dx / length * p_escape_length, dy / length * p_escape_length).round();
  }

  private static Point directional_pin_escape_point(Pin p_pin, FloatPoint p_toward, double p_escape_length) {
    if (p_pin == null || p_toward == null) {
      return null;
    }
    FloatPoint pinCenter = p_pin.get_center().to_float();
    double dx = p_toward.x - pinCenter.x;
    double dy = p_toward.y - pinCenter.y;
    double length = Math.sqrt(dx * dx + dy * dy);
    if (length <= 0.0) {
      return null;
    }
    return shift_point(pinCenter, dx / length * p_escape_length, dy / length * p_escape_length).round();
  }

  private Point pin_escape_point(Pin p_pin, double p_escape_length) {
    if (p_pin == null || p_pin.get_component_no() <= 0 || board.components == null) {
      return null;
    }
    Component component = board.components.get(p_pin.get_component_no());
    if (component == null || component.get_location() == null) {
      return null;
    }
    FloatPoint pinCenter = p_pin.get_center().to_float();
    FloatPoint componentCenter = component.get_location().to_float();
    double dx = pinCenter.x - componentCenter.x;
    double dy = pinCenter.y - componentCenter.y;
    double length = Math.sqrt(dx * dx + dy * dy);
    if (length <= 0.0) {
      return null;
    }
    return shift_point(pinCenter, dx / length * p_escape_length, dy / length * p_escape_length).round();
  }

  private static Point[] wrap_with_pin_gateways(Point p_start, Point[] p_body, Point p_end) {
    List<Point> points = new ArrayList<>();
    append_distinct(points, p_start);
    for (Point point : p_body) {
      append_distinct(points, point);
    }
    append_distinct(points, p_end);
    return points.toArray(Point[]::new);
  }

  private List<CoupledCandidate> coupled_candidates(
      Point p_first_from,
      Point p_first_to,
      Point p_second_from,
      Point p_second_to,
      List<PolylineTrace> p_first_traces,
      List<PolylineTrace> p_second_traces,
      int p_layer,
      double p_center_spacing) {
    List<CoupledCandidate> result = new ArrayList<>();
    FloatPoint firstFrom = p_first_from.to_float();
    FloatPoint firstTo = p_first_to.to_float();
    FloatPoint secondFrom = p_second_from.to_float();
    FloatPoint secondTo = p_second_to.to_float();
    FloatPoint centerFrom = midpoint(firstFrom, secondFrom);
    FloatPoint centerTo = midpoint(firstTo, secondTo);
    double dx = centerTo.x - centerFrom.x;
    double dy = centerTo.y - centerFrom.y;
    double length = Math.sqrt(dx * dx + dy * dy);
    if (length <= 0) {
      return result;
    }
    double halfSpacing = p_center_spacing / 2.0;

    for (FloatPoint normal : coupled_normals(firstFrom, firstTo, secondFrom, secondTo, dx, dy, length)) {
      double normalX = normal.x;
      double normalY = normal.y;
      FloatPoint midHorizontal = new FloatPoint(centerTo.x, centerFrom.y);
      FloatPoint midVertical = new FloatPoint(centerFrom.x, centerTo.y);
      add_coupled_candidate(result, firstFrom, firstTo, secondFrom, secondTo, centerFrom, centerTo,
          new FloatPoint[] { centerFrom, centerTo }, normalX, normalY, halfSpacing, p_center_spacing);
      add_coupled_candidate(result, firstFrom, firstTo, secondFrom, secondTo, centerFrom, centerTo,
          new FloatPoint[] { centerFrom, midHorizontal, centerTo }, normalX, normalY, halfSpacing, p_center_spacing);
      add_coupled_candidate(result, firstFrom, firstTo, secondFrom, secondTo, centerFrom, centerTo,
          new FloatPoint[] { centerFrom, midVertical, centerTo }, normalX, normalY, halfSpacing, p_center_spacing);
      add_center_corridor_candidates(
          result,
          firstFrom,
          firstTo,
          secondFrom,
          secondTo,
          centerFrom,
          centerTo,
          dx,
          dy,
          length,
          normalX,
          normalY,
          halfSpacing,
          p_center_spacing);
      add_meander_corridor_candidates(
          result,
          firstFrom,
          firstTo,
          secondFrom,
          secondTo,
          centerFrom,
          centerTo,
          dx,
          dy,
          length,
          normalX,
          normalY,
          halfSpacing,
          p_center_spacing);
      add_gateway_corridor_candidates(
          result,
          firstFrom,
          firstTo,
          secondFrom,
          secondTo,
          centerFrom,
          centerTo,
          dx,
          dy,
          length,
          normalX,
          normalY,
          halfSpacing,
          p_center_spacing);
      add_gateway_corridor_candidates(
          result,
          firstFrom,
          firstTo,
          secondFrom,
          secondTo,
          centerFrom,
          centerTo,
          dx,
          dy,
          length,
          -normalX,
          -normalY,
          halfSpacing,
          p_center_spacing);
      add_external_loop_candidates(
          result,
          firstFrom,
          firstTo,
          secondFrom,
          secondTo,
          centerFrom,
          centerTo,
          dx,
          dy,
          length,
          normalX,
          normalY,
          halfSpacing,
          p_center_spacing);
      add_order_transition_candidates(
          result,
          firstFrom,
          firstTo,
          secondFrom,
          secondTo,
          centerFrom,
          centerTo,
          dx,
          dy,
          length,
          halfSpacing,
          p_center_spacing);

      add_trace_spine_candidates(
          result,
          p_first_traces,
          true,
          p_layer,
          firstFrom,
          firstTo,
          secondFrom,
          secondTo,
          centerFrom,
          centerTo,
          normalX,
          normalY,
          halfSpacing,
          p_center_spacing);
      add_trace_spine_candidates(
          result,
          p_second_traces,
          false,
          p_layer,
          firstFrom,
          firstTo,
          secondFrom,
          secondTo,
          centerFrom,
          centerTo,
          normalX,
          normalY,
          halfSpacing,
          p_center_spacing);

      double[] shiftDistances = {
          0.0,
          p_center_spacing,
          -p_center_spacing,
          mm_to_board(board, 0.6),
          -mm_to_board(board, 0.6),
          mm_to_board(board, 1.2),
          -mm_to_board(board, 1.2),
      };
      double[] insetDistances = {
          Math.min(length * 0.08, mm_to_board(board, 0.25)),
          Math.min(length * 0.15, mm_to_board(board, 0.50)),
          Math.min(length * 0.25, mm_to_board(board, 1.0)),
          Math.min(length * 0.35, mm_to_board(board, 2.0)),
      };
      for (double shift : shiftDistances) {
        for (double inset : insetDistances) {
          if (inset <= 0 || inset * 2 >= length) {
            continue;
          }
          FloatPoint insetFrom = shift_point(
              centerFrom,
              (dx / length) * inset + normalX * shift,
              (dy / length) * inset + normalY * shift);
          FloatPoint insetTo = shift_point(
              centerTo,
              -(dx / length) * inset + normalX * shift,
              -(dy / length) * inset + normalY * shift);
          FloatPoint shiftedMidHorizontal = new FloatPoint(insetTo.x, insetFrom.y);
          FloatPoint shiftedMidVertical = new FloatPoint(insetFrom.x, insetTo.y);
          add_coupled_candidate(result, firstFrom, firstTo, secondFrom, secondTo, centerFrom, centerTo,
              new FloatPoint[] { insetFrom, insetTo }, normalX, normalY, halfSpacing, p_center_spacing);
          add_coupled_candidate(result, firstFrom, firstTo, secondFrom, secondTo, centerFrom, centerTo,
              new FloatPoint[] { insetFrom, shiftedMidHorizontal, insetTo }, normalX, normalY, halfSpacing, p_center_spacing);
          add_coupled_candidate(result, firstFrom, firstTo, secondFrom, secondTo, centerFrom, centerTo,
              new FloatPoint[] { insetFrom, shiftedMidVertical, insetTo }, normalX, normalY, halfSpacing, p_center_spacing);
        }
      }
    }
    return result;
  }

  private double[] coupled_candidate_center_spacings(double p_target_center_spacing) {
    return new double[] {
        p_target_center_spacing,
        p_target_center_spacing + mm_to_board(board, 0.014),
        p_target_center_spacing + mm_to_board(board, 0.020),
        p_target_center_spacing + mm_to_board(board, 0.030),
        p_target_center_spacing + mm_to_board(board, 0.04),
        p_target_center_spacing + mm_to_board(board, 0.06),
        p_target_center_spacing + mm_to_board(board, 0.08),
        p_target_center_spacing + mm_to_board(board, 0.12),
    };
  }

  private List<FloatPoint> coupled_normals(
      FloatPoint p_first_from,
      FloatPoint p_first_to,
      FloatPoint p_second_from,
      FloatPoint p_second_to,
      double p_dx,
      double p_dy,
      double p_length) {
    List<FloatPoint> result = new ArrayList<>();
    add_normal(result, -p_dy / p_length, p_dx / p_length, p_first_from, p_second_from);
    add_normal(
        result,
        ((p_first_from.x - p_second_from.x) + (p_first_to.x - p_second_to.x)) / 2.0,
        ((p_first_from.y - p_second_from.y) + (p_first_to.y - p_second_to.y)) / 2.0,
        p_first_from,
        p_second_from);
    add_normal(result, 1.0, 0.0, p_first_from, p_second_from);
    add_normal(result, 0.0, 1.0, p_first_from, p_second_from);
    return result;
  }

  private static void add_normal(
      List<FloatPoint> p_result,
      double p_x,
      double p_y,
      FloatPoint p_first_from,
      FloatPoint p_second_from) {
    double length = Math.sqrt(p_x * p_x + p_y * p_y);
    if (length <= 0.0) {
      return;
    }
    double normalX = p_x / length;
    double normalY = p_y / length;
    if (side_of(p_first_from, p_second_from, normalX, normalY) < 0) {
      normalX = -normalX;
      normalY = -normalY;
    }
    for (FloatPoint existing : p_result) {
      if (Math.abs(existing.x * normalX + existing.y * normalY) > 0.98) {
        return;
      }
    }
    p_result.add(new FloatPoint(normalX, normalY));
  }

  private void add_order_transition_candidates(
      List<CoupledCandidate> p_result,
      FloatPoint p_first_from,
      FloatPoint p_first_to,
      FloatPoint p_second_from,
      FloatPoint p_second_to,
      FloatPoint p_center_from,
      FloatPoint p_center_to,
      double p_dx,
      double p_dy,
      double p_length,
      double p_half_spacing,
      double p_center_spacing) {
    double targetNormalX = p_first_to.x - p_second_to.x;
    double targetNormalY = p_first_to.y - p_second_to.y;
    double targetNormalLength = Math.sqrt(targetNormalX * targetNormalX + targetNormalY * targetNormalY);
    if (targetNormalLength <= 0.0) {
      return;
    }
    targetNormalX /= targetNormalLength;
    targetNormalY /= targetNormalLength;
    if (side_of(p_first_from, p_second_from, targetNormalX, targetNormalY) > 0) {
      return;
    }

    double ux = p_dx / p_length;
    double uy = p_dy / p_length;
    double sideNormalX = -uy;
    double sideNormalY = ux;
    add_dense_endpoint_order_transition_candidates(
        p_result,
        p_first_from,
        p_first_to,
        p_second_from,
        p_second_to,
        p_center_from,
        p_center_to,
        ux,
        uy,
        sideNormalX,
        sideNormalY,
        p_length,
        p_half_spacing,
        p_center_spacing);
    double[] insets = {
        Math.min(p_length * 0.08, mm_to_board(board, 0.80)),
        Math.min(p_length * 0.14, mm_to_board(board, 1.50)),
        Math.min(p_length * 0.22, mm_to_board(board, 2.50)),
    };
    double[] loopDepths = {
        mm_to_board(board, 0.80),
        mm_to_board(board, 1.40),
        mm_to_board(board, 2.20),
        mm_to_board(board, 3.20),
    };
    double[] fanDistances = {
        Math.max(p_center_spacing, mm_to_board(board, 0.45)),
        mm_to_board(board, 0.80),
        mm_to_board(board, 1.20),
        mm_to_board(board, 1.80),
    };
    double[] corridorShifts = {
        0.0,
        p_center_spacing,
        -p_center_spacing,
        mm_to_board(board, 0.80),
        -mm_to_board(board, 0.80),
    };

    for (double inset : insets) {
      if (inset <= 0.0 || inset * 2.0 >= p_length) {
        continue;
      }
      for (double shift : corridorShifts) {
        FloatPoint corridorFrom = shift_point(
            p_center_from,
            ux * inset + targetNormalX * shift,
            uy * inset + targetNormalY * shift);
        FloatPoint corridorTo = shift_point(
            p_center_to,
            -ux * inset + targetNormalX * shift,
            -uy * inset + targetNormalY * shift);
        FloatPoint corridorFirstFrom = shift_point(corridorFrom, targetNormalX * p_half_spacing, targetNormalY * p_half_spacing);
        FloatPoint corridorSecondFrom = shift_point(corridorFrom, -targetNormalX * p_half_spacing, -targetNormalY * p_half_spacing);
        FloatPoint corridorFirstTo = shift_point(corridorTo, targetNormalX * p_half_spacing, targetNormalY * p_half_spacing);
        FloatPoint corridorSecondTo = shift_point(corridorTo, -targetNormalX * p_half_spacing, -targetNormalY * p_half_spacing);
        for (double loopDepth : loopDepths) {
          for (double fan : fanDistances) {
            FloatPoint firstAway = shift_point(p_first_from, -ux * loopDepth, -uy * loopDepth);
            FloatPoint secondAway = shift_point(p_second_from, -ux * loopDepth, -uy * loopDepth);
            FloatPoint firstAround = shift_point(firstAway, targetNormalX * fan, targetNormalY * fan);
            FloatPoint secondAround = shift_point(secondAway, -targetNormalX * fan, -targetNormalY * fan);
            add_explicit_coupled_candidate(
                p_result,
                new FloatPoint[] {
                    p_first_from,
                    firstAway,
                    firstAround,
                    corridorFirstFrom,
                    corridorFirstTo,
                    p_first_to,
                },
                new FloatPoint[] {
                    p_second_from,
                    secondAway,
                    secondAround,
                    corridorSecondFrom,
                    corridorSecondTo,
                    p_second_to,
                },
                p_center_spacing,
                "order_transition_symmetric_loop");
            add_explicit_coupled_candidate(
                p_result,
                new FloatPoint[] {
                    p_first_from,
                    firstAway,
                    firstAround,
                    corridorFirstFrom,
                    corridorFirstTo,
                    p_first_to,
                },
                new FloatPoint[] {
                    p_second_from,
                    secondAround,
                    corridorSecondFrom,
                    corridorSecondTo,
                    p_second_to,
                },
                p_center_spacing,
                "order_transition_first_loop");
            add_explicit_coupled_candidate(
                p_result,
                new FloatPoint[] {
                    p_first_from,
                    firstAround,
                    corridorFirstFrom,
                    corridorFirstTo,
                    p_first_to,
                },
                new FloatPoint[] {
                    p_second_from,
                    secondAway,
                    secondAround,
                    corridorSecondFrom,
                    corridorSecondTo,
                    p_second_to,
                },
                p_center_spacing,
                "order_transition_second_loop");
          }
        }
      }
    }

    double[] sideFans = {
        mm_to_board(board, 1.00),
        mm_to_board(board, 1.60),
        mm_to_board(board, 2.40),
        mm_to_board(board, 3.40),
    };
    double[] sideLoopDepths = {
        mm_to_board(board, 0.50),
        mm_to_board(board, 0.90),
        mm_to_board(board, 1.40),
        mm_to_board(board, 2.10),
    };
    for (double inset : insets) {
      if (inset <= 0.0 || inset * 2.0 >= p_length) {
        continue;
      }
      FloatPoint corridorFrom = shift_point(p_center_from, ux * inset, uy * inset);
      FloatPoint corridorTo = shift_point(p_center_to, -ux * inset, -uy * inset);
      FloatPoint corridorFirstFrom = shift_point(corridorFrom, targetNormalX * p_half_spacing, targetNormalY * p_half_spacing);
      FloatPoint corridorSecondFrom = shift_point(corridorFrom, -targetNormalX * p_half_spacing, -targetNormalY * p_half_spacing);
      FloatPoint corridorFirstTo = shift_point(corridorTo, targetNormalX * p_half_spacing, targetNormalY * p_half_spacing);
      FloatPoint corridorSecondTo = shift_point(corridorTo, -targetNormalX * p_half_spacing, -targetNormalY * p_half_spacing);
      for (int sideSign : new int[] { 1, -1 }) {
        double sx = sideNormalX * sideSign;
        double sy = sideNormalY * sideSign;
        for (double fan : sideFans) {
          for (double loopDepth : sideLoopDepths) {
            FloatPoint firstSide = shift_point(p_first_from, sx * fan, sy * fan);
            FloatPoint secondSide = shift_point(p_second_from, sx * (fan + p_center_spacing), sy * (fan + p_center_spacing));
            FloatPoint secondAround = shift_point(
                secondSide,
                -targetNormalX * Math.max(loopDepth, p_center_spacing),
                -targetNormalY * Math.max(loopDepth, p_center_spacing));
            add_explicit_coupled_candidate(
                p_result,
                new FloatPoint[] {
                    p_first_from,
                    firstSide,
                    corridorFirstFrom,
                    corridorFirstTo,
                    p_first_to,
                },
                new FloatPoint[] {
                    p_second_from,
                    secondSide,
                    secondAround,
                    corridorSecondFrom,
                    corridorSecondTo,
                    p_second_to,
                },
                p_center_spacing,
                "order_transition_lateral_second_loop");
            FloatPoint firstAround = shift_point(
                firstSide,
                targetNormalX * Math.max(loopDepth, p_center_spacing),
                targetNormalY * Math.max(loopDepth, p_center_spacing));
            add_explicit_coupled_candidate(
                p_result,
                new FloatPoint[] {
                    p_first_from,
                    firstSide,
                    firstAround,
                    corridorFirstFrom,
                    corridorFirstTo,
                    p_first_to,
                },
                new FloatPoint[] {
                    p_second_from,
                    secondSide,
                    corridorSecondFrom,
                    corridorSecondTo,
                    p_second_to,
                },
                p_center_spacing,
                "order_transition_lateral_first_loop");
          }
        }
      }
    }
  }

  private void add_dense_endpoint_order_transition_candidates(
      List<CoupledCandidate> p_result,
      FloatPoint p_first_from,
      FloatPoint p_first_to,
      FloatPoint p_second_from,
      FloatPoint p_second_to,
      FloatPoint p_center_from,
      FloatPoint p_center_to,
      double p_ux,
      double p_uy,
      double p_trunk_normal_x,
      double p_trunk_normal_y,
      double p_length,
      double p_half_spacing,
      double p_center_spacing) {
    double sourceNormalX = p_first_from.x - p_second_from.x;
    double sourceNormalY = p_first_from.y - p_second_from.y;
    double sourceNormalLength = Math.sqrt(sourceNormalX * sourceNormalX + sourceNormalY * sourceNormalY);
    double targetNormalX = p_first_to.x - p_second_to.x;
    double targetNormalY = p_first_to.y - p_second_to.y;
    double targetNormalLength = Math.sqrt(targetNormalX * targetNormalX + targetNormalY * targetNormalY);
    if (sourceNormalLength <= 0.0 || targetNormalLength <= 0.0) {
      return;
    }
    sourceNormalX /= sourceNormalLength;
    sourceNormalY /= sourceNormalLength;
    targetNormalX /= targetNormalLength;
    targetNormalY /= targetNormalLength;
    double sourceTangentX = -sourceNormalY;
    double sourceTangentY = sourceNormalX;
    double targetTangentX = -targetNormalY;
    double targetTangentY = targetNormalX;

    double[] insets = {
        Math.min(p_length * 0.10, mm_to_board(board, 1.20)),
        Math.min(p_length * 0.18, mm_to_board(board, 2.20)),
        Math.min(p_length * 0.28, mm_to_board(board, 3.60)),
    };
    double[] trunkShifts = {
        0.0,
        p_center_spacing,
        -p_center_spacing,
        mm_to_board(board, 0.80),
        -mm_to_board(board, 0.80),
        mm_to_board(board, 1.60),
        -mm_to_board(board, 1.60),
    };
    double[] fanDistances = {
        Math.max(p_center_spacing * 1.25, mm_to_board(board, 0.60)),
        Math.max(p_center_spacing * 2.00, mm_to_board(board, 1.00)),
        Math.max(p_center_spacing * 3.00, mm_to_board(board, 1.50)),
        mm_to_board(board, 2.20),
        mm_to_board(board, 3.20),
    };

    for (int trunkSign : new int[] { 1, -1 }) {
      double nx = p_trunk_normal_x * trunkSign;
      double ny = p_trunk_normal_y * trunkSign;
      for (double inset : insets) {
        if (inset <= 0.0 || inset * 2.0 >= p_length) {
          continue;
        }
        for (double shift : trunkShifts) {
          FloatPoint trunkFrom = shift_point(
              p_center_from,
              p_ux * inset + nx * shift,
              p_uy * inset + ny * shift);
          FloatPoint trunkTo = shift_point(
              p_center_to,
              -p_ux * inset + nx * shift,
              -p_uy * inset + ny * shift);
          FloatPoint firstTrunkFrom = shift_point(trunkFrom, nx * p_half_spacing, ny * p_half_spacing);
          FloatPoint secondTrunkFrom = shift_point(trunkFrom, -nx * p_half_spacing, -ny * p_half_spacing);
          FloatPoint firstTrunkTo = shift_point(trunkTo, nx * p_half_spacing, ny * p_half_spacing);
          FloatPoint secondTrunkTo = shift_point(trunkTo, -nx * p_half_spacing, -ny * p_half_spacing);
          for (int sourceSign : new int[] { 1, -1 }) {
            double sx = sourceTangentX * sourceSign;
            double sy = sourceTangentY * sourceSign;
            for (int targetSign : new int[] { 1, -1 }) {
              double tx = targetTangentX * targetSign;
              double ty = targetTangentY * targetSign;
              for (double fan : fanDistances) {
                FloatPoint firstSourceEscape = shift_point(p_first_from, sx * fan, sy * fan);
                FloatPoint secondSourceEscape = shift_point(
                    p_second_from,
                    sx * (fan + p_center_spacing),
                    sy * (fan + p_center_spacing));
                FloatPoint firstTargetEscape = shift_point(p_first_to, tx * fan, ty * fan);
                FloatPoint secondTargetEscape = shift_point(
                    p_second_to,
                    tx * (fan + p_center_spacing),
                    ty * (fan + p_center_spacing));
                add_explicit_coupled_candidate(
                    p_result,
                    new FloatPoint[] {
                        p_first_from,
                        firstSourceEscape,
                        firstTrunkFrom,
                        firstTrunkTo,
                        firstTargetEscape,
                        p_first_to,
                    },
                    new FloatPoint[] {
                        p_second_from,
                        secondSourceEscape,
                        secondTrunkFrom,
                        secondTrunkTo,
                        secondTargetEscape,
                        p_second_to,
                    },
                    p_center_spacing,
                    "order_transition_dense_endpoint_lanes");

                FloatPoint firstSourceElbow = new FloatPoint(firstSourceEscape.x, firstTrunkFrom.y);
                FloatPoint secondSourceElbow = new FloatPoint(secondSourceEscape.x, secondTrunkFrom.y);
                FloatPoint firstTargetElbow = new FloatPoint(firstTargetEscape.x, firstTrunkTo.y);
                FloatPoint secondTargetElbow = new FloatPoint(secondTargetEscape.x, secondTrunkTo.y);
                add_explicit_coupled_candidate(
                    p_result,
                    new FloatPoint[] {
                        p_first_from,
                        firstSourceEscape,
                        firstSourceElbow,
                        firstTrunkFrom,
                        firstTrunkTo,
                        firstTargetElbow,
                        firstTargetEscape,
                        p_first_to,
                    },
                    new FloatPoint[] {
                        p_second_from,
                        secondSourceEscape,
                        secondSourceElbow,
                        secondTrunkFrom,
                        secondTrunkTo,
                        secondTargetElbow,
                        secondTargetEscape,
                        p_second_to,
                    },
                    p_center_spacing,
                    "order_transition_dense_endpoint_lanes_elbow");
              }
            }
          }
        }
      }
    }
  }

  private void add_center_corridor_candidates(
      List<CoupledCandidate> p_result,
      FloatPoint p_first_from,
      FloatPoint p_first_to,
      FloatPoint p_second_from,
      FloatPoint p_second_to,
      FloatPoint p_center_from,
      FloatPoint p_center_to,
      double p_dx,
      double p_dy,
      double p_length,
      double p_normal_x,
      double p_normal_y,
      double p_half_spacing,
      double p_center_spacing) {
    double minFan = mm_to_board(board, 0.35);
    double maxFan = Math.min(p_length * 0.35, mm_to_board(board, 1.5));
    for (double fan : new double[] { minFan, mm_to_board(board, 0.55), mm_to_board(board, 0.8), maxFan }) {
      if (fan <= 0 || fan * 2 >= p_length) {
        continue;
      }
      FloatPoint corridorFrom = shift_point(p_center_from, (p_dx / p_length) * fan, (p_dy / p_length) * fan);
      FloatPoint corridorTo = shift_point(p_center_to, -(p_dx / p_length) * fan, -(p_dy / p_length) * fan);
      add_coupled_candidate(
          p_result,
          p_first_from,
          p_first_to,
          p_second_from,
          p_second_to,
          p_center_from,
          p_center_to,
          new FloatPoint[] { corridorFrom, corridorTo },
          p_normal_x,
          p_normal_y,
          p_half_spacing,
          p_center_spacing);
      FloatPoint elbowFrom = new FloatPoint(corridorTo.x, corridorFrom.y);
      add_coupled_candidate(
          p_result,
          p_first_from,
          p_first_to,
          p_second_from,
          p_second_to,
          p_center_from,
          p_center_to,
          new FloatPoint[] { corridorFrom, elbowFrom, corridorTo },
          p_normal_x,
          p_normal_y,
          p_half_spacing,
          p_center_spacing);
    }
  }

  private void add_meander_corridor_candidates(
      List<CoupledCandidate> p_result,
      FloatPoint p_first_from,
      FloatPoint p_first_to,
      FloatPoint p_second_from,
      FloatPoint p_second_to,
      FloatPoint p_center_from,
      FloatPoint p_center_to,
      double p_dx,
      double p_dy,
      double p_length,
      double p_normal_x,
      double p_normal_y,
      double p_half_spacing,
      double p_center_spacing) {
    if (p_length < mm_to_board(board, 2.0)) {
      return;
    }
    double ux = p_dx / p_length;
    double uy = p_dy / p_length;
    double inset = Math.min(p_length * 0.12, mm_to_board(board, 0.50));
    if (inset <= 0.0 || inset * 2.0 >= p_length) {
      return;
    }
    double usableLength = p_length - (2.0 * inset);
    double[] amplitudes = {
        mm_to_board(board, 0.35),
        mm_to_board(board, 0.60),
        mm_to_board(board, 0.90),
    };
    int[] turns = { 2, 3, 4 };
    for (double amplitude : amplitudes) {
      if (amplitude <= p_half_spacing) {
        continue;
      }
      for (int turnCount : turns) {
        double segmentLength = usableLength / (turnCount + 1.0);
        if (segmentLength < mm_to_board(board, 0.35)) {
          continue;
        }
        List<FloatPoint> centerline = new ArrayList<>();
        centerline.add(shift_point(p_center_from, ux * inset, uy * inset));
        for (int i = 1; i <= turnCount; i++) {
          double along = inset + segmentLength * i;
          double sign = (i % 2 == 0) ? -1.0 : 1.0;
          centerline.add(shift_point(
              p_center_from,
              ux * along + p_normal_x * amplitude * sign,
              uy * along + p_normal_y * amplitude * sign));
        }
        centerline.add(shift_point(p_center_to, -ux * inset, -uy * inset));
        add_coupled_candidate(
            p_result,
            p_first_from,
            p_first_to,
            p_second_from,
            p_second_to,
            p_center_from,
            p_center_to,
            centerline.toArray(FloatPoint[]::new),
            p_normal_x,
            p_normal_y,
            p_half_spacing,
            p_center_spacing);
      }
    }
  }

  private void add_gateway_corridor_candidates(
      List<CoupledCandidate> p_result,
      FloatPoint p_first_from,
      FloatPoint p_first_to,
      FloatPoint p_second_from,
      FloatPoint p_second_to,
      FloatPoint p_center_from,
      FloatPoint p_center_to,
      double p_dx,
      double p_dy,
      double p_length,
      double p_normal_x,
      double p_normal_y,
      double p_half_spacing,
      double p_center_spacing) {
    double ux = p_dx / p_length;
    double uy = p_dy / p_length;
    double[] fanDistances = {
        Math.max(p_center_spacing, mm_to_board(board, 0.45)),
        mm_to_board(board, 0.80),
        mm_to_board(board, 1.20),
        mm_to_board(board, 1.80),
        mm_to_board(board, 2.50),
    };
    double[] insetDistances = {
        Math.min(p_length * 0.08, mm_to_board(board, 0.80)),
        Math.min(p_length * 0.15, mm_to_board(board, 1.50)),
        Math.min(p_length * 0.25, mm_to_board(board, 2.50)),
        Math.min(p_length * 0.35, mm_to_board(board, 4.00)),
    };
    double[] shiftDistances = {
        0.0,
        p_center_spacing,
        -p_center_spacing,
        mm_to_board(board, 0.80),
        -mm_to_board(board, 0.80),
        mm_to_board(board, 1.50),
        -mm_to_board(board, 1.50),
        mm_to_board(board, 2.50),
        -mm_to_board(board, 2.50),
    };

    for (double fan : fanDistances) {
      if (fan <= p_half_spacing) {
        continue;
      }
      for (double inset : insetDistances) {
        if (inset <= 0.0 || inset * 2.0 >= p_length) {
          continue;
        }
        for (double shift : shiftDistances) {

        FloatPoint corridorFrom = shift_point(
            p_center_from,
            ux * inset + p_normal_x * shift,
            uy * inset + p_normal_y * shift);
        FloatPoint corridorTo = shift_point(
            p_center_to,
            -ux * inset + p_normal_x * shift,
            -uy * inset + p_normal_y * shift);
        FloatPoint corridorFirstFrom = shift_point(corridorFrom, p_normal_x * p_half_spacing, p_normal_y * p_half_spacing);
        FloatPoint corridorSecondFrom = shift_point(corridorFrom, -p_normal_x * p_half_spacing, -p_normal_y * p_half_spacing);
        FloatPoint corridorFirstTo = shift_point(corridorTo, p_normal_x * p_half_spacing, p_normal_y * p_half_spacing);
        FloatPoint corridorSecondTo = shift_point(corridorTo, -p_normal_x * p_half_spacing, -p_normal_y * p_half_spacing);
        FloatPoint firstFromEscape = shift_point(p_first_from, p_normal_x * fan, p_normal_y * fan);
        FloatPoint secondFromEscape = shift_point(p_second_from, -p_normal_x * fan, -p_normal_y * fan);
        FloatPoint firstToEscape = shift_point(p_first_to, p_normal_x * fan, p_normal_y * fan);
        FloatPoint secondToEscape = shift_point(p_second_to, -p_normal_x * fan, -p_normal_y * fan);

        add_explicit_coupled_candidate(
            p_result,
            new FloatPoint[] {
                p_first_from,
                firstFromEscape,
                corridorFirstFrom,
                corridorFirstTo,
                firstToEscape,
                p_first_to,
            },
            new FloatPoint[] {
                p_second_from,
                secondFromEscape,
                corridorSecondFrom,
                corridorSecondTo,
                secondToEscape,
                p_second_to,
            },
            p_center_spacing);

        FloatPoint firstFromTurn = new FloatPoint(firstFromEscape.x, corridorFirstFrom.y);
        FloatPoint secondFromTurn = new FloatPoint(secondFromEscape.x, corridorSecondFrom.y);
        FloatPoint firstToTurn = new FloatPoint(firstToEscape.x, corridorFirstTo.y);
        FloatPoint secondToTurn = new FloatPoint(secondToEscape.x, corridorSecondTo.y);
        add_explicit_coupled_candidate(
            p_result,
            new FloatPoint[] {
                p_first_from,
                firstFromEscape,
                firstFromTurn,
                corridorFirstFrom,
                corridorFirstTo,
                firstToTurn,
                firstToEscape,
                p_first_to,
            },
            new FloatPoint[] {
                p_second_from,
                secondFromEscape,
                secondFromTurn,
                corridorSecondFrom,
                corridorSecondTo,
                secondToTurn,
                secondToEscape,
                p_second_to,
            },
            p_center_spacing);
        }
      }
    }
  }

  private void add_external_loop_candidates(
      List<CoupledCandidate> p_result,
      FloatPoint p_first_from,
      FloatPoint p_first_to,
      FloatPoint p_second_from,
      FloatPoint p_second_to,
      FloatPoint p_center_from,
      FloatPoint p_center_to,
      double p_dx,
      double p_dy,
      double p_length,
      double p_normal_x,
      double p_normal_y,
      double p_half_spacing,
      double p_center_spacing) {
    if (p_length < mm_to_board(board, 1.5)) {
      return;
    }

    double ux = p_dx / p_length;
    double uy = p_dy / p_length;
    double[] sideOffsets = {
        mm_to_board(board, 0.80),
        mm_to_board(board, 1.20),
        mm_to_board(board, 1.80),
        mm_to_board(board, 2.50),
        mm_to_board(board, 3.50),
        mm_to_board(board, 4.50),
    };
    double[] extensions = {
        0.0,
        mm_to_board(board, 0.75),
        mm_to_board(board, 1.50),
        mm_to_board(board, 2.50),
        mm_to_board(board, 3.50),
    };

    for (double sideOffset : sideOffsets) {
      if (sideOffset <= p_half_spacing) {
        continue;
      }
      for (double extension : extensions) {
        for (int sideSign : new int[] { 1, -1 }) {
          double sx = p_normal_x * sideOffset * sideSign;
          double sy = p_normal_y * sideOffset * sideSign;
          FloatPoint loopFrom = shift_point(p_center_from, -ux * extension + sx, -uy * extension + sy);
          FloatPoint loopTo = shift_point(p_center_to, ux * extension + sx, uy * extension + sy);
          add_coupled_candidate(
              p_result,
              p_first_from,
              p_first_to,
              p_second_from,
              p_second_to,
              p_center_from,
              p_center_to,
              new FloatPoint[] { p_center_from, loopFrom, loopTo, p_center_to },
              p_normal_x,
              p_normal_y,
              p_half_spacing,
              p_center_spacing);

          double inset = Math.min(Math.max(p_center_spacing, mm_to_board(board, 0.45)), p_length * 0.20);
          if (inset > 0.0 && inset * 2.0 < p_length) {
            FloatPoint entry = shift_point(p_center_from, ux * inset, uy * inset);
            FloatPoint exit = shift_point(p_center_to, -ux * inset, -uy * inset);
            FloatPoint entryLoop = shift_point(entry, sx, sy);
            FloatPoint exitLoop = shift_point(exit, sx, sy);
            add_coupled_candidate(
                p_result,
                p_first_from,
                p_first_to,
                p_second_from,
                p_second_to,
                p_center_from,
                p_center_to,
                new FloatPoint[] { entry, entryLoop, exitLoop, exit },
                p_normal_x,
                p_normal_y,
                p_half_spacing,
                p_center_spacing);
          }
        }
      }
    }
  }

  private static boolean add_explicit_coupled_candidate(
      List<CoupledCandidate> p_result,
      FloatPoint[] p_first,
      FloatPoint[] p_second,
      double p_center_spacing) {
    return add_explicit_coupled_candidate(p_result, p_first, p_second, p_center_spacing, "explicit");
  }

  private static boolean add_explicit_coupled_candidate(
      List<CoupledCandidate> p_result,
      FloatPoint[] p_first,
      FloatPoint[] p_second,
      double p_center_spacing,
      String p_family) {
    List<Point> first = new ArrayList<>();
    List<Point> second = new ArrayList<>();
    for (FloatPoint point : p_first) {
      append_distinct(first, point.round());
    }
    for (FloatPoint point : p_second) {
      append_distinct(second, point.round());
    }
    if (first.size() >= 2 && second.size() >= 2) {
      Point[] firstPoints = first.toArray(Point[]::new);
      Point[] secondPoints = second.toArray(Point[]::new);
      if (candidate_geometry_is_safe(firstPoints, secondPoints, p_center_spacing, p_family.startsWith("order_transition_"))) {
        p_result.add(new CoupledCandidate(firstPoints, secondPoints, p_center_spacing, p_family));
        return true;
      }
    }
    return false;
  }

  private void add_trace_spine_candidates(
      List<CoupledCandidate> p_result,
      List<PolylineTrace> p_traces,
      boolean p_trace_is_first_member,
      int p_layer,
      FloatPoint p_first_from,
      FloatPoint p_first_to,
      FloatPoint p_second_from,
      FloatPoint p_second_to,
      FloatPoint p_center_from,
      FloatPoint p_center_to,
      double p_normal_x,
      double p_normal_y,
      double p_half_spacing,
      double p_center_spacing) {
    for (PolylineTrace trace : p_traces) {
      if (trace.get_layer() != p_layer) {
        continue;
      }
      Point[] corners = trace.polyline().corner_arr();
      if (corners.length < 2) {
        continue;
      }
      FloatPoint[] centerline = trace_spine_centerline(
          corners,
          p_trace_is_first_member,
          p_normal_x,
          p_normal_y,
          p_half_spacing,
          false);
      add_coupled_candidate(
          p_result,
          p_first_from,
          p_first_to,
          p_second_from,
          p_second_to,
          p_center_from,
          p_center_to,
          centerline,
          p_normal_x,
          p_normal_y,
          p_half_spacing,
          p_center_spacing);
      FloatPoint[] reversedCenterline = trace_spine_centerline(
          corners,
          p_trace_is_first_member,
          p_normal_x,
          p_normal_y,
          p_half_spacing,
          true);
      add_coupled_candidate(
          p_result,
          p_first_from,
          p_first_to,
          p_second_from,
          p_second_to,
          p_center_from,
          p_center_to,
          reversedCenterline,
          p_normal_x,
          p_normal_y,
          p_half_spacing,
          p_center_spacing);
    }
  }

  private static FloatPoint[] trace_spine_centerline(
      Point[] p_corners,
      boolean p_trace_is_first_member,
      double p_normal_x,
      double p_normal_y,
      double p_half_spacing,
      boolean p_reverse) {
    FloatPoint[] result = new FloatPoint[p_corners.length];
    double direction = p_trace_is_first_member ? -1.0 : 1.0;
    for (int i = 0; i < p_corners.length; i++) {
      int sourceIndex = p_reverse ? p_corners.length - 1 - i : i;
      FloatPoint corner = p_corners[sourceIndex].to_float();
      result[i] = shift_point(
          corner,
          p_normal_x * p_half_spacing * direction,
          p_normal_y * p_half_spacing * direction);
    }
    return result;
  }

  private void add_coupled_candidate(
      List<CoupledCandidate> p_result,
      FloatPoint p_first_from,
      FloatPoint p_first_to,
      FloatPoint p_second_from,
      FloatPoint p_second_to,
      FloatPoint p_center_from,
      FloatPoint p_center_to,
      FloatPoint[] p_centerline,
      double p_normal_x,
      double p_normal_y,
      double p_half_spacing,
      double p_center_spacing) {
    List<Point> first = new ArrayList<>();
    List<Point> second = new ArrayList<>();
    append_distinct(first, p_first_from.round());
    append_distinct(second, p_second_from.round());
    for (FloatPoint center : p_centerline) {
      append_distinct(first, offset_point(center, p_normal_x, p_normal_y, p_half_spacing));
      append_distinct(second, offset_point(center, -p_normal_x, -p_normal_y, p_half_spacing));
    }
    append_distinct(first, p_first_to.round());
    append_distinct(second, p_second_to.round());
    if (first.size() >= 2 && second.size() >= 2) {
      Point[] firstPoints = first.toArray(Point[]::new);
      Point[] secondPoints = second.toArray(Point[]::new);
      if (candidate_geometry_is_safe(firstPoints, secondPoints, p_center_spacing)) {
        p_result.add(new CoupledCandidate(firstPoints, secondPoints, p_center_spacing));
      }
    }
  }

  private static boolean candidate_geometry_is_safe(Point[] p_first, Point[] p_second, double p_center_spacing) {
    return candidate_geometry_is_safe(p_first, p_second, p_center_spacing, false);
  }

  private static boolean candidate_geometry_is_safe(
      Point[] p_first,
      Point[] p_second,
      double p_center_spacing,
      boolean p_allow_endpoint_transition) {
    if (polylines_intersect(p_first, p_second)) {
      return false;
    }
    if (!p_allow_endpoint_transition
        && (endpoint_escape_moves_away_from_route(p_first, p_center_spacing)
        || endpoint_escape_moves_away_from_route(p_second, p_center_spacing))) {
      return false;
    }
    double minDistance = nearest_polyline_distance(p_first, p_second);
    return !Double.isFinite(minDistance) || minDistance >= p_center_spacing * 0.75;
  }

  private static boolean endpoint_escape_moves_away_from_route(Point[] p_points, double p_center_spacing) {
    if (p_points.length < 3) {
      return false;
    }
    double maxAllowedAwayStep = p_center_spacing * 3.0;
    return endpoint_step_moves_away(p_points[0].to_float(), p_points[1].to_float(), p_points[p_points.length - 1].to_float(), maxAllowedAwayStep)
        || endpoint_step_moves_away(p_points[p_points.length - 1].to_float(), p_points[p_points.length - 2].to_float(), p_points[0].to_float(), maxAllowedAwayStep);
  }

  private static boolean endpoint_step_moves_away(
      FloatPoint p_endpoint,
      FloatPoint p_next,
      FloatPoint p_route_target,
      double p_max_allowed_step) {
    double stepX = p_next.x - p_endpoint.x;
    double stepY = p_next.y - p_endpoint.y;
    double stepLength = Math.sqrt(stepX * stepX + stepY * stepY);
    if (stepLength <= p_max_allowed_step) {
      return false;
    }
    double routeX = p_route_target.x - p_endpoint.x;
    double routeY = p_route_target.y - p_endpoint.y;
    return (stepX * routeX + stepY * routeY) < 0.0;
  }

  private static boolean polylines_intersect(Point[] p_first, Point[] p_second) {
    for (int i = 0; i < p_first.length - 1; i++) {
      FloatPoint a = p_first[i].to_float();
      FloatPoint b = p_first[i + 1].to_float();
      for (int j = 0; j < p_second.length - 1; j++) {
        FloatPoint c = p_second[j].to_float();
        FloatPoint d = p_second[j + 1].to_float();
        if (segments_intersect(a, b, c, d)) {
          return true;
        }
      }
    }
    return false;
  }

  private static boolean segments_intersect(FloatPoint a, FloatPoint b, FloatPoint c, FloatPoint d) {
    double abx = b.x - a.x;
    double aby = b.y - a.y;
    double cdx = d.x - c.x;
    double cdy = d.y - c.y;
    double denominator = abx * cdy - aby * cdx;
    if (Math.abs(denominator) < 1e-9) {
      return false;
    }
    double acx = c.x - a.x;
    double acy = c.y - a.y;
    double t = (acx * cdy - acy * cdx) / denominator;
    double u = (acx * aby - acy * abx) / denominator;
    return t > 1e-6 && t < 1.0 - 1e-6 && u > 1e-6 && u < 1.0 - 1e-6;
  }

  private static double nearest_polyline_distance(Point[] p_first, Point[] p_second) {
    double result = Double.POSITIVE_INFINITY;
    for (int i = 0; i < p_first.length - 1; i++) {
      FloatPoint a = p_first[i].to_float();
      FloatPoint b = p_first[i + 1].to_float();
      for (int j = 0; j < p_second.length - 1; j++) {
        FloatPoint c = p_second[j].to_float();
        FloatPoint d = p_second[j + 1].to_float();
        result = Math.min(result, segment_distance(a, b, c, d));
      }
    }
    return Double.isInfinite(result) ? Double.NaN : result;
  }

  private static IntPoint offset_point(FloatPoint p_point, double p_normal_x, double p_normal_y, double p_distance) {
    return new IntPoint(
        (int) Math.round(p_point.x + p_normal_x * p_distance),
        (int) Math.round(p_point.y + p_normal_y * p_distance));
  }

  private double try_replace_trace(
      PolylineTrace p_trace,
      Point[] p_new_points,
      int p_max_incompletes,
      int p_max_clearance_violations,
      PairMemberMeasurement p_member,
      double p_before_length) {
    if (p_trace.is_user_fixed()) {
      return Double.NaN;
    }
    board.generate_snapshot();
    int netNo = p_trace.get_net_no(0);
    int layer = p_trace.get_layer();
    int halfWidth = p_trace.get_half_width();
    int[] netNoArr = new int[p_trace.net_count()];
    for (int i = 0; i < netNoArr.length; i++) {
      netNoArr[i] = p_trace.get_net_no(i);
    }
    int clearanceClass = p_trace.clearance_class_no();

    boolean removed = board.remove_items(Set.of(p_trace));
    if (!removed) {
      board.undo(null);
      return Double.NaN;
    }

    board.insert_trace(p_new_points, layer, halfWidth, netNoArr, clearanceClass, FixedState.UNFIXED);
    board.combine_traces(netNo);

    double afterLength = measurement_length(p_member);
    int afterIncompletes = incomplete_count(board);
    int afterClearanceViolations = clearance_violation_count(board);
    if (Double.isFinite(afterLength)
        && afterLength > p_before_length
        && afterIncompletes <= p_max_incompletes
        && afterClearanceViolations <= p_max_clearance_violations) {
      board.pop_snapshot();
      return afterLength;
    }

    board.undo(null);
    return Double.NaN;
  }

  private double try_replace_pair_member_trace(
      DifferentialPair p_pair,
      Net p_first_net,
      Net p_second_net,
      PolylineTrace p_trace,
      Point[] p_new_points,
      int p_max_incompletes,
      int p_max_clearance_violations,
      double p_current_skew,
      double p_max_skew) {
    if (p_trace.is_user_fixed()) {
      return Double.NaN;
    }
    board.generate_snapshot();
    int netNo = p_trace.get_net_no(0);
    int layer = p_trace.get_layer();
    int halfWidth = p_trace.get_half_width();
    int[] netNoArr = new int[p_trace.net_count()];
    for (int i = 0; i < netNoArr.length; i++) {
      netNoArr[i] = p_trace.get_net_no(i);
    }
    int clearanceClass = p_trace.clearance_class_no();
    FixedState fixedState = p_trace.get_fixed_state();

    if (!board.remove_items(Set.of(p_trace))) {
      board.undo(null);
      return Double.NaN;
    }

    board.insert_trace(p_new_points, layer, halfWidth, netNoArr, clearanceClass, fixedState);
    board.combine_traces(netNo);

    PairMeasurements after = measure_pair(p_pair, p_first_net, p_second_net);
    double afterSkew = after == null || after.first() == null || after.second() == null
        ? Double.NaN
        : Math.abs(after.first().length() - after.second().length());
    int afterIncompletes = incomplete_count(board);
    int afterClearanceViolations = clearance_violation_count(board);
    if (Double.isFinite(afterSkew)
        && afterSkew <= p_max_skew + mm_to_board(board, 0.01)
        && afterSkew < p_current_skew
        && afterIncompletes <= p_max_incompletes
        && afterClearanceViolations <= p_max_clearance_violations) {
      board.pop_snapshot();
      return afterSkew;
    }

    board.undo(null);
    return Double.NaN;
  }

  private List<PairMeanderCandidate> build_pair_meander_candidates(
      PolylineTrace p_trace,
      List<PolylineTrace> p_companion_traces,
      double p_desired_extra) {
    List<PairMeanderCandidate> result = new ArrayList<>();
    Point[] corners = p_trace.polyline().corner_arr();
    if (corners.length < 2 || p_desired_extra <= 0.0) {
      return result;
    }

    double minHeight = Math.max(
        mm_to_board(board, PAIR_MEANDER_MIN_AMPLITUDE_MM),
        3.0 * p_trace.get_half_width());
    double maxHeight = mm_to_board(board, PAIR_MEANDER_MAX_AMPLITUDE_MM);
    double traceMargin = Math.max(
        mm_to_board(board, PAIR_MEANDER_MIN_SPACING_MM),
        4.0 * p_trace.get_half_width());

    List<PairMeanderSegmentCandidate> segmentCandidates = new ArrayList<>();
    for (int segmentIndex = 0; segmentIndex < corners.length - 1; segmentIndex++) {
      if (!(corners[segmentIndex] instanceof IntPoint from) || !(corners[segmentIndex + 1] instanceof IntPoint to)) {
        continue;
      }
      FloatPoint fromFloat = from.to_float();
      FloatPoint toFloat = to.to_float();
      double dx = toFloat.x - fromFloat.x;
      double dy = toFloat.y - fromFloat.y;
      double segmentLength = Math.sqrt(dx * dx + dy * dy);
      if (segmentLength <= 4.0 * traceMargin) {
        continue;
      }

      double normalX = -dy / segmentLength;
      double normalY = dx / segmentLength;
      int outwardSign = companion_outward_sign(fromFloat, toFloat, p_companion_traces, normalX, normalY);
      double coupledOverlap = nearby_parallel_overlap(fromFloat, toFloat, p_companion_traces);
      if (coupledOverlap <= 0.0) {
        continue;
      }
      segmentCandidates.add(new PairMeanderSegmentCandidate(
          segmentIndex,
          segmentLength,
          normalX * outwardSign,
          normalY * outwardSign,
          coupledOverlap));
    }
    segmentCandidates.sort(Comparator
        .comparingDouble(PairMeanderSegmentCandidate::coupledOverlap).reversed()
        .thenComparing(Comparator.comparingDouble(PairMeanderSegmentCandidate::length).reversed()));

    for (PairMeanderSegmentCandidate segment : segmentCandidates) {
      int[] bumpCounts = { 1, 2, 3, 4 };
      for (int bumpCount : bumpCounts) {
        double usableLength = segment.length() - (2.0 * traceMargin);
        double minBumpWidth = Math.max(
            mm_to_board(board, PAIR_MEANDER_MIN_BUMP_WIDTH_MM),
            4.0 * p_trace.get_half_width());
        if (usableLength <= 0.0 || usableLength / bumpCount < minBumpWidth) {
          continue;
        }
        double targetHeight = Math.max(minHeight, Math.min(p_desired_extra / (2.0 * bumpCount), maxHeight));
        double maxCandidateHeight = Math.min(maxHeight, segment.length() / Math.max(3.0, bumpCount + 1.0));
        for (double height : detour_heights(targetHeight, minHeight, maxCandidateHeight)) {
          if (height <= 0.0) {
            continue;
          }
          Point[] candidate = splice_centered_outward_meanders(
              corners,
              segment.index(),
              segment.normalX(),
              segment.normalY(),
              height,
              bumpCount,
              traceMargin,
              minBumpWidth);
          if (candidate != null) {
            result.add(new PairMeanderCandidate(
                candidate,
                2.0 * height * bumpCount,
                segment.coupledOverlap(),
                segment.length(),
                bumpCount,
                height));
          }
        }
      }
    }
    result.sort(Comparator.comparingDouble(candidate -> candidate.score(p_desired_extra)));
    return result;
  }

  private double nearby_parallel_overlap(
      FloatPoint p_from,
      FloatPoint p_to,
      List<PolylineTrace> p_companion_traces) {
    double maxCenterDistance = mm_to_board(board, 4.0);
    double result = 0.0;
    for (PolylineTrace companion : p_companion_traces) {
      if (companion == null) {
        continue;
      }
      FloatPoint[] companionCorners = companion.polyline().corner_approx_arr();
      for (int i = 0; i < companionCorners.length - 1; i++) {
        FloatPoint companionFrom = companionCorners[i];
        FloatPoint companionTo = companionCorners[i + 1];
        if (!segments_parallel(p_from, p_to, companionFrom, companionTo)) {
          continue;
        }
        if (segment_distance(p_from, p_to, companionFrom, companionTo) > maxCenterDistance) {
          continue;
        }
        result += parallel_overlap_length(p_from, p_to, companionFrom, companionTo);
      }
    }
    return result;
  }

  private int companion_outward_sign(
      FloatPoint p_from,
      FloatPoint p_to,
      List<PolylineTrace> p_companion_traces,
      double p_normal_x,
      double p_normal_y) {
    FloatPoint mid = midpoint(p_from, p_to);
    FloatPoint nearest = null;
    double nearestDistance = Double.POSITIVE_INFINITY;
    for (PolylineTrace companion : p_companion_traces) {
      FloatPoint candidate = companion.polyline().nearest_point_approx(mid);
      if (candidate == null) {
        continue;
      }
      double distance = candidate.distance_square(mid);
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearest = candidate;
      }
    }
    if (nearest == null) {
      return 1;
    }
    double side = ((nearest.x - mid.x) * p_normal_x) + ((nearest.y - mid.y) * p_normal_y);
    return side >= 0.0 ? -1 : 1;
  }

  private List<Point[]> build_detour_candidates(PolylineTrace p_trace, double p_desired_extra) {
    List<Point[]> result = new ArrayList<>();
    Point[] corners = p_trace.polyline().corner_arr();
    if (corners.length < 2) {
      return result;
    }

    double minHeight = mm_to_board(board, 0.05);
    double maxHeight = mm_to_board(board, 3.0);
    double traceMargin = Math.max(2.0 * p_trace.get_half_width(), mm_to_board(board, 0.10));

    for (int segmentIndex = 0; segmentIndex < corners.length - 1; segmentIndex++) {
      if (!(corners[segmentIndex] instanceof IntPoint from) || !(corners[segmentIndex + 1] instanceof IntPoint to)) {
        continue;
      }
      FloatPoint fromFloat = from.to_float();
      FloatPoint toFloat = to.to_float();
      double dx = toFloat.x - fromFloat.x;
      double dy = toFloat.y - fromFloat.y;
      double segmentLength = Math.sqrt(dx * dx + dy * dy);
      if (segmentLength <= 4.0 * traceMargin) {
        continue;
      }

      double normalX = -dy / segmentLength;
      double normalY = dx / segmentLength;
      double maxCandidateHeight = Math.min(maxHeight, segmentLength / 3.0);
      double targetHeight = Math.max(minHeight, Math.min(p_desired_extra / 2.0, maxCandidateHeight));
      for (double height : detour_heights(targetHeight, minHeight, maxCandidateHeight)) {
        double margin = Math.min(segmentLength * 0.25, Math.max(traceMargin, height));
        double startT = margin / segmentLength;
        double endT = 1.0 - startT;
        if (endT <= startT) {
          startT = 0.33;
          endT = 0.67;
        }

        for (int sign : new int[] { 1, -1 }) {
          Point[] candidate = splice_detour(corners, segmentIndex, startT, endT, normalX * sign, normalY * sign, height);
          if (candidate != null) {
            result.add(candidate);
          }
        }
      }
    }
    return result;
  }

  private Point[] splice_centered_outward_meanders(
      Point[] p_corners,
      int p_segment_index,
      double p_normal_x,
      double p_normal_y,
      double p_height,
      int p_bump_count,
      double p_margin,
      double p_min_bump_width) {
    IntPoint from = (IntPoint) p_corners[p_segment_index];
    IntPoint to = (IntPoint) p_corners[p_segment_index + 1];
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double segmentLength = Math.sqrt(dx * dx + dy * dy);
    if (segmentLength <= 0.0 || p_bump_count <= 0 || p_margin * 2.0 >= segmentLength) {
      return null;
    }
    double ux = dx / segmentLength;
    double uy = dy / segmentLength;
    double usableLength = segmentLength - (2.0 * p_margin);
    double minPlateau = mm_to_board(board, FLOW_THROUGH_MIN_PLATEAU_MM);
    double spacing = rounded_bump_spacing(p_height, minPlateau, p_margin);
    double bumpLength = Math.max(
        p_min_bump_width,
        Math.max(
            rounded_bump_width(p_height, minPlateau, spacing),
            p_height * 2.5));
    double meanderSpan = (bumpLength * p_bump_count) + (spacing * (p_bump_count - 1.0));
    if (bumpLength <= 0.0 || meanderSpan > usableLength) {
      return null;
    }

    FloatPoint roundedFrom = shift_point(from.to_float(), ux * p_margin, uy * p_margin);
    FloatPoint roundedTo = shift_point(to.to_float(), -ux * p_margin, -uy * p_margin);
    Point[] rounded = rounded_outward_bump_path(
        roundedFrom,
        roundedTo,
        p_normal_x,
        p_normal_y,
        p_bump_count,
        p_height,
        bumpLength,
        minPlateau,
        spacing,
        false);
    if (rounded == null) {
      return null;
    }
    for (Point point : rounded) {
      if (!board.bounding_box.contains(point)) {
        return null;
      }
    }

    List<Point> points = new ArrayList<>(p_corners.length + rounded.length);
    for (int i = 0; i <= p_segment_index; i++) {
      append_distinct(points, p_corners[i]);
    }
    for (Point point : rounded) {
      append_distinct(points, point);
    }
    for (int i = p_segment_index + 1; i < p_corners.length; i++) {
      append_distinct(points, p_corners[i]);
    }
    return points.size() < 2 ? null : points.toArray(Point[]::new);
  }

  private static List<Double> detour_heights(double p_target_height, double p_min_height, double p_max_height) {
    List<Double> result = new ArrayList<>();
    for (double factor : new double[] { 1.0, 0.75, 0.5, 0.25, 1.25, 1.5 }) {
      double height = Math.max(p_min_height, Math.min(p_target_height * factor, p_max_height));
      boolean duplicate = false;
      for (double existing : result) {
        if (Math.abs(existing - height) < 1.0) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        result.add(height);
      }
    }
    return result;
  }

  private Point[] splice_detour(Point[] p_corners, int p_segment_index, double p_start_t, double p_end_t,
      double p_normal_x, double p_normal_y, double p_height) {
    IntPoint from = (IntPoint) p_corners[p_segment_index];
    IntPoint to = (IntPoint) p_corners[p_segment_index + 1];
    double dx = to.x - from.x;
    double dy = to.y - from.y;

    IntPoint q0 = new IntPoint((int) Math.round(from.x + dx * p_start_t),
        (int) Math.round(from.y + dy * p_start_t));
    IntPoint q1 = new IntPoint((int) Math.round(from.x + dx * p_end_t),
        (int) Math.round(from.y + dy * p_end_t));
    IntPoint q0Offset = new IntPoint((int) Math.round(q0.x + p_normal_x * p_height),
        (int) Math.round(q0.y + p_normal_y * p_height));
    IntPoint q1Offset = new IntPoint((int) Math.round(q1.x + p_normal_x * p_height),
        (int) Math.round(q1.y + p_normal_y * p_height));

    if (!board.bounding_box.contains(q0Offset) || !board.bounding_box.contains(q1Offset)) {
      return null;
    }

    List<Point> points = new ArrayList<>(p_corners.length + 4);
    for (int i = 0; i <= p_segment_index; i++) {
      append_distinct(points, p_corners[i]);
    }
    append_distinct(points, q0);
    append_distinct(points, q0Offset);
    append_distinct(points, q1Offset);
    append_distinct(points, q1);
    for (int i = p_segment_index + 1; i < p_corners.length; i++) {
      append_distinct(points, p_corners[i]);
    }

    if (points.size() < 2) {
      return null;
    }
    return points.toArray(Point[]::new);
  }

  private static void append_distinct(List<Point> p_points, Point p_point) {
    if (p_points.isEmpty() || !p_points.get(p_points.size() - 1).equals(p_point)) {
      p_points.add(p_point);
    }
  }

  private List<PolylineTrace> collect_unfixed_traces(PairMemberMeasurement p_member) {
    List<PolylineTrace> result = new ArrayList<>();
    Collection<Item> netItems = board.get_connectable_items(p_member.netNo());
    for (Item item : netItems) {
      if (item instanceof PolylineTrace trace
          && !trace.is_user_fixed()
          && trace.net_count() == 1
          && (!p_member.scoped() || p_member.traceIds().contains(trace.get_id_no()))) {
        result.add(trace);
      }
    }
    return result;
  }

  private List<PolylineTrace> collect_unfixed_net_traces(int p_net_no) {
    List<PolylineTrace> result = new ArrayList<>();
    Collection<Item> netItems = board.get_connectable_items(p_net_no);
    for (Item item : netItems) {
      if (item instanceof PolylineTrace trace
          && !trace.is_user_fixed()
          && trace.net_count() == 1) {
        result.add(trace);
      }
    }
    return result;
  }

  private List<PolylineTrace> collect_pair_traces(PairMemberMeasurement p_member) {
    List<PolylineTrace> result = new ArrayList<>();
    Collection<Item> netItems = board.get_connectable_items(p_member.netNo());
    for (Item item : netItems) {
      if (item instanceof PolylineTrace trace
          && trace.net_count() == 1
          && (!p_member.scoped() || p_member.traceIds().contains(trace.get_id_no()))) {
        result.add(trace);
      }
    }
    return result;
  }

  private PairMeasurements measure_pair(DifferentialPair p_pair, Net p_first_net, Net p_second_net) {
    if (p_pair.has_scoped_pins()) {
      PairMemberMeasurement first = scoped_member_measurement(
          p_first_net.net_number, p_first_net.name, p_pair.first_from_pin(), p_pair.first_to_pin());
      PairMemberMeasurement second = scoped_member_measurement(
          p_second_net.net_number, p_second_net.name, p_pair.second_from_pin(), p_pair.second_to_pin());
      if (first != null && second != null) {
        return new PairMeasurements(first, second, true);
      }
    }
    return new PairMeasurements(total_member_measurement(p_first_net), total_member_measurement(p_second_net), false);
  }

  private PairMemberMeasurement total_member_measurement(Net p_net) {
    return new PairMemberMeasurement(p_net.net_number, p_net.name, p_net.get_trace_length(), false, null, null, Set.of());
  }

  private PairMemberMeasurement scoped_member_measurement(int p_net_no, String p_net_name, String p_from_pin, String p_to_pin) {
    Pin fromPin = find_pin(p_net_no, p_from_pin);
    Pin toPin = find_pin(p_net_no, p_to_pin);
    if (fromPin == null || toPin == null) {
      return null;
    }
    PathSearchResult path = geometric_scoped_route_path(p_net_no, fromPin, toPin);
    if (path == null) {
      path = shortest_route_path(p_net_no, fromPin, toPin);
    }
    if (path == null) {
      return null;
    }
    return new PairMemberMeasurement(
        p_net_no, p_net_name, path.length(), true, p_from_pin, p_to_pin, path.traceIds());
  }

  private double measurement_length(PairMemberMeasurement p_member) {
    if (!p_member.scoped()) {
      Net net = board.rules.nets.get(p_member.netNo());
      return net == null ? Double.NaN : net.get_trace_length();
    }
    PairMemberMeasurement refreshed = scoped_member_measurement(
        p_member.netNo(), p_member.netName(), p_member.fromPin(), p_member.toPin());
    return refreshed == null ? Double.NaN : refreshed.length();
  }

  private Pin find_pin(int p_net_no, String p_pin_identifier) {
    if (p_pin_identifier == null) {
      return null;
    }
    Net net = board.rules.nets.get(p_net_no);
    if (net == null) {
      return null;
    }
    for (Pin pin : net.get_pins()) {
      Component component = board.components.get(pin.get_component_no());
      if (component == null) {
        continue;
      }
      String pinName = pin.name();
      if (pinName != null && p_pin_identifier.equals(component.name + "-" + pinName)) {
        return pin;
      }
    }
    return null;
  }

  private PathSearchResult shortest_route_path(int p_net_no, Item p_start, Item p_target) {
    PriorityQueue<PathNode> queue = new PriorityQueue<>(Comparator.comparingDouble(PathNode::distance));
    Map<Item, Double> distanceByItem = new HashMap<>();
    Map<Item, Item> previousByItem = new HashMap<>();
    distanceByItem.put(p_start, 0.0);
    queue.add(new PathNode(p_start, 0.0));

    while (!queue.isEmpty()) {
      PathNode current = queue.poll();
      double bestDistance = distanceByItem.getOrDefault(current.item(), Double.POSITIVE_INFINITY);
      if (current.distance() > bestDistance) {
        continue;
      }
      if (current.item() == p_target) {
        break;
      }
      for (Item contact : current.item().get_normal_contacts()) {
        if (!contact.contains_net(p_net_no)) {
          continue;
        }
        double edgeLength = contact instanceof Trace trace ? trace.get_length() : 0.0;
        double candidateDistance = current.distance() + edgeLength;
        if (candidateDistance < distanceByItem.getOrDefault(contact, Double.POSITIVE_INFINITY)) {
          distanceByItem.put(contact, candidateDistance);
          previousByItem.put(contact, current.item());
          queue.add(new PathNode(contact, candidateDistance));
        }
      }
    }

    Double pathLength = distanceByItem.get(p_target);
    if (pathLength == null) {
      return null;
    }
    Set<Integer> traceIds = new HashSet<>();
    Item current = p_target;
    while (current != null) {
      if (current instanceof PolylineTrace) {
        traceIds.add(current.get_id_no());
      }
      current = previousByItem.get(current);
    }
    return new PathSearchResult(pathLength, Set.copyOf(traceIds));
  }

  private PathSearchResult geometric_scoped_route_path(int p_net_no, Pin p_start, Pin p_target) {
    FloatPoint startCenter = p_start.get_center().to_float();
    FloatPoint targetCenter = p_target.get_center().to_float();
    Map<String, List<GeometricPathEdge>> graph = new HashMap<>();
    double tolerance = mm_to_board(board, 0.10);
    double pinContactTolerance = mm_to_board(board, 0.65);
    List<FloatPoint> startContacts = new ArrayList<>();
    List<FloatPoint> targetContacts = new ArrayList<>();

    for (Item item : board.get_connectable_items(p_net_no)) {
      if (!(item instanceof PolylineTrace trace) || !trace.contains_net(p_net_no)) {
        continue;
      }
      FloatPoint[] corners = trace.polyline().corner_approx_arr();
      if (corners.length < 2) {
        continue;
      }
      for (int index = 0; index < corners.length - 1; index++) {
        List<GeometricPathPoint> points = new ArrayList<>();
        FloatPoint from = corners[index];
        FloatPoint to = corners[index + 1];
        points.add(new GeometricPathPoint(from, 0.0));
        points.add(new GeometricPathPoint(to, from.distance(to)));
        add_pin_contact_point(points, startContacts, p_start, from, to, pinContactTolerance);
        add_pin_contact_point(points, targetContacts, p_target, from, to, pinContactTolerance);
        points.sort(Comparator.comparingDouble(GeometricPathPoint::offset));
        for (int pointIndex = 0; pointIndex < points.size() - 1; pointIndex++) {
          FloatPoint a = points.get(pointIndex).point();
          FloatPoint b = points.get(pointIndex + 1).point();
          double length = a.distance(b);
          if (length <= tolerance / 10.0) {
            continue;
          }
          add_geometric_edge(graph, a, b, length, trace.get_id_no());
        }
      }
    }

    Set<String> startKeys = geometric_node_keys(startContacts);
    Set<String> targetKeys = geometric_node_keys(targetContacts);
    if (startKeys.isEmpty() || targetKeys.isEmpty()) {
      return null;
    }

    PriorityQueue<GeometricPathNode> queue = new PriorityQueue<>(Comparator.comparingDouble(GeometricPathNode::distance));
    Map<String, Double> distanceByNode = new HashMap<>();
    Map<String, GeometricPreviousEdge> previousByNode = new HashMap<>();
    for (String startKey : startKeys) {
      distanceByNode.put(startKey, 0.0);
      queue.add(new GeometricPathNode(startKey, 0.0));
    }
    String reachedTarget = null;
    while (!queue.isEmpty()) {
      GeometricPathNode current = queue.poll();
      double bestDistance = distanceByNode.getOrDefault(current.key(), Double.POSITIVE_INFINITY);
      if (current.distance() > bestDistance) {
        continue;
      }
      if (targetKeys.contains(current.key())) {
        reachedTarget = current.key();
        break;
      }
      for (GeometricPathEdge edge : graph.getOrDefault(current.key(), List.of())) {
        double candidateDistance = current.distance() + edge.length();
        if (candidateDistance < distanceByNode.getOrDefault(edge.to(), Double.POSITIVE_INFINITY)) {
          distanceByNode.put(edge.to(), candidateDistance);
          previousByNode.put(edge.to(), new GeometricPreviousEdge(current.key(), edge.traceId()));
          queue.add(new GeometricPathNode(edge.to(), candidateDistance));
        }
      }
    }

    if (reachedTarget == null) {
      return null;
    }
    Double pathLength = distanceByNode.get(reachedTarget);
    if (pathLength == null) {
      return null;
    }
    Set<Integer> traceIds = new HashSet<>();
    String current = reachedTarget;
    while (!startKeys.contains(current)) {
      GeometricPreviousEdge previous = previousByNode.get(current);
      if (previous == null) {
        break;
      }
      traceIds.add(previous.traceId());
      current = previous.from();
    }
    return new PathSearchResult(pathLength, Set.copyOf(traceIds));
  }

  private static void add_pin_contact_point(
      List<GeometricPathPoint> p_points,
      List<FloatPoint> p_contacts,
      Pin p_pin,
      FloatPoint p_from,
      FloatPoint p_to,
      double p_tolerance) {
    FloatPoint pinCenter = p_pin.get_center().to_float();
    double dx = p_to.x - p_from.x;
    double dy = p_to.y - p_from.y;
    double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= 0.0) {
      return;
    }
    double t = ((pinCenter.x - p_from.x) * dx + (pinCenter.y - p_from.y) * dy) / lengthSquared;
    double segmentLength = Math.sqrt(lengthSquared);
    FloatPoint contact = null;
    double contactDistance = 0.0;
    if (t >= -1e-6 && t <= 1.0 + 1e-6) {
      t = Math.max(0.0, Math.min(1.0, t));
      FloatPoint projected = new FloatPoint(p_from.x + (t * dx), p_from.y + (t * dy));
      if (projected.distance(pinCenter) <= p_tolerance) {
        contactDistance = t * segmentLength;
        contact = projected;
      }
    }
    if (contact == null) {
      double fromDistance = p_from.distance(pinCenter);
      double toDistance = p_to.distance(pinCenter);
      if (fromDistance <= p_tolerance || toDistance <= p_tolerance) {
        if (fromDistance <= toDistance) {
          contact = p_from;
          contactDistance = 0.0;
        } else {
          contact = p_to;
          contactDistance = segmentLength;
        }
      }
    }
    if (contact != null) {
      p_points.add(new GeometricPathPoint(contact, contactDistance));
      p_contacts.add(contact);
    }
  }

  private static Set<String> geometric_node_keys(List<FloatPoint> p_points) {
    Set<String> result = new HashSet<>();
    for (FloatPoint point : p_points) {
      result.add(geometric_node_key(point));
    }
    return result;
  }

  private static void add_geometric_edge(
      Map<String, List<GeometricPathEdge>> p_graph,
      FloatPoint p_from,
      FloatPoint p_to,
      double p_length,
      int p_trace_id) {
    String fromKey = geometric_node_key(p_from);
    String toKey = geometric_node_key(p_to);
    p_graph.computeIfAbsent(fromKey, ignored -> new ArrayList<>()).add(new GeometricPathEdge(toKey, p_length, p_trace_id));
    p_graph.computeIfAbsent(toKey, ignored -> new ArrayList<>()).add(new GeometricPathEdge(fromKey, p_length, p_trace_id));
  }

  private static String geometric_node_key(FloatPoint p_point) {
    return String.format(Locale.US, "%.3f:%.3f", p_point.x, p_point.y);
  }

  private static double longest_segment_length(PolylineTrace p_trace) {
    FloatPoint[] corners = p_trace.polyline().corner_approx_arr();
    double result = 0;
    for (int i = 0; i < corners.length - 1; i++) {
      result = Math.max(result, corners[i].distance(corners[i + 1]));
    }
    return result;
  }

  private RouterIntentSettings.DifferentialPairIntent router_intent_pair(DifferentialPair p_pair) {
    if (job == null
        || job.routerSettings == null
        || job.routerSettings.intent == null
        || job.routerSettings.intent.differentialPairs == null) {
      return null;
    }
    Net firstNet = board.rules.nets.get(p_pair.first_net_no());
    Net secondNet = board.rules.nets.get(p_pair.second_net_no());
    if (firstNet == null || secondNet == null) {
      return null;
    }
    for (RouterIntentSettings.DifferentialPairIntent intentPair : job.routerSettings.intent.differentialPairs) {
      if (intentPair == null) {
        continue;
      }
      if ((firstNet.name.equals(intentPair.positiveNet) && secondNet.name.equals(intentPair.negativeNet))
          || (firstNet.name.equals(intentPair.negativeNet) && secondNet.name.equals(intentPair.positiveNet))) {
        return intentPair;
      }
    }
    return null;
  }

  private boolean net_has_coupled_pair_intent(String p_net_name) {
    if (p_net_name == null
        || job == null
        || job.routerSettings == null
        || job.routerSettings.intent == null
        || job.routerSettings.intent.differentialPairs == null) {
      return false;
    }
    for (RouterIntentSettings.DifferentialPairIntent pair : job.routerSettings.intent.differentialPairs) {
      if (pair == null || !Boolean.TRUE.equals(pair.routeAsCoupledPair)) {
        continue;
      }
      if (p_net_name.equals(pair.positiveNet) || p_net_name.equals(pair.negativeNet)) {
        return true;
      }
    }
    return false;
  }

  private double pair_endpoint_span(DifferentialPair p_pair) {
    if (p_pair == null || board == null || board.rules == null || board.rules.nets == null) {
      return Double.POSITIVE_INFINITY;
    }
    Net firstNet = board.rules.nets.get(p_pair.first_net_no());
    Net secondNet = board.rules.nets.get(p_pair.second_net_no());
    if (firstNet == null || secondNet == null) {
      return Double.POSITIVE_INFINITY;
    }
    Pin firstFrom = find_pin(firstNet.net_number, p_pair.first_from_pin());
    Pin firstTo = find_pin(firstNet.net_number, p_pair.first_to_pin());
    Pin secondFrom = find_pin(secondNet.net_number, p_pair.second_from_pin());
    Pin secondTo = find_pin(secondNet.net_number, p_pair.second_to_pin());
    if (firstFrom == null || firstTo == null || secondFrom == null || secondTo == null) {
      return Double.POSITIVE_INFINITY;
    }
    FloatPoint firstFromPoint = firstFrom.get_center().to_float();
    FloatPoint firstToPoint = firstTo.get_center().to_float();
    FloatPoint secondFromPoint = secondFrom.get_center().to_float();
    FloatPoint secondToPoint = secondTo.get_center().to_float();
    FloatPoint centerFrom = midpoint(firstFromPoint, secondFromPoint);
    FloatPoint centerTo = midpoint(firstToPoint, secondToPoint);
    return centerFrom.distance(centerTo);
  }

  private boolean allows_only_front_layer(RouterIntentSettings.DifferentialPairIntent p_pair) {
    if (p_pair.allowedLayers == null || p_pair.allowedLayers.length != 1) {
      return false;
    }
    return "F.Cu".equals(p_pair.allowedLayers[0])
        && Boolean.TRUE.equals(p_pair.sameLayerRequired)
        && p_pair.maxViasPerNet != null
        && p_pair.maxViasPerNet <= 0;
  }

  private int board_layer_index(String p_layer_name) {
    if (board.layer_structure == null || board.layer_structure.arr == null) {
      return -1;
    }
    for (int i = 0; i < board.layer_structure.arr.length; i++) {
      if (p_layer_name.equals(board.layer_structure.arr[i].name)) {
        return i;
      }
    }
    return -1;
  }

  private static TraceStyle trace_style(PolylineTrace p_trace) {
    return new TraceStyle(p_trace.get_half_width(), p_trace.clearance_class_no());
  }

  private static TraceStyle trace_style(Net p_net, int p_layer) {
    if (p_net == null || p_net.get_class() == null) {
      return new TraceStyle(0, 0);
    }
    return new TraceStyle(
        p_net.get_class().get_trace_half_width(p_layer),
        p_net.get_class().get_trace_clearance_class());
  }

  private TraceStyle endpoint_escape_style(
      RouterIntentSettings.DifferentialPairIntent p_pair,
      TraceStyle p_base_style) {
    if (p_pair == null
        || p_pair.endpointEscapeWidthMm == null
        || p_pair.endpointEscapeWidthMm <= 0.0
        || p_base_style == null
        || p_base_style.halfWidth() <= 0) {
      return null;
    }
    int halfWidth = (int) Math.round(mm_to_board(board, p_pair.endpointEscapeWidthMm) / 2.0);
    if (halfWidth <= 0 || halfWidth >= p_base_style.halfWidth()) {
      return null;
    }
    return new TraceStyle(halfWidth, p_base_style.clearanceClass());
  }

  private double endpoint_escape_length(RouterIntentSettings.DifferentialPairIntent p_pair) {
    if (p_pair == null || p_pair.endpointEscapeLengthMm == null || p_pair.endpointEscapeLengthMm <= 0.0) {
      return 0.0;
    }
    return mm_to_board(board, p_pair.endpointEscapeLengthMm);
  }

  private static String pair_name(Net p_first_net, Net p_second_net) {
    String first = p_first_net == null ? "<missing>" : p_first_net.name;
    String second = p_second_net == null ? "<missing>" : p_second_net.name;
    return first + "/" + second;
  }

  private static FloatPoint midpoint(FloatPoint p_first, FloatPoint p_second) {
    return new FloatPoint((p_first.x + p_second.x) / 2.0, (p_first.y + p_second.y) / 2.0);
  }

  private static FloatPoint shift_point(FloatPoint p_point, double p_dx, double p_dy) {
    return new FloatPoint(p_point.x + p_dx, p_point.y + p_dy);
  }

  private static double side_of(FloatPoint p_first, FloatPoint p_second, double p_normal_x, double p_normal_y) {
    return ((p_first.x - p_second.x) * p_normal_x) + ((p_first.y - p_second.y) * p_normal_y);
  }

  private static double nearest_gap_between_traces(List<PolylineTrace> p_first, List<PolylineTrace> p_second) {
    double result = Double.POSITIVE_INFINITY;
    for (PolylineTrace first : p_first) {
      for (PolylineTrace second : p_second) {
        if (first.get_layer() != second.get_layer()) {
          continue;
        }
        double centerDistance = nearest_polyline_distance(first, second);
        if (Double.isFinite(centerDistance)) {
          result = Math.min(result, Math.max(0.0, centerDistance - first.get_half_width() - second.get_half_width()));
        }
      }
    }
    return Double.isInfinite(result) ? Double.NaN : result;
  }

  private static double parallel_length(
      List<PolylineTrace> p_first,
      List<PolylineTrace> p_second,
      double p_target_center_spacing) {
    double maxGap = Math.max(p_target_center_spacing * 1.75, p_target_center_spacing + 1.0);
    double result = 0.0;
    for (PolylineTrace first : p_first) {
      for (PolylineTrace second : p_second) {
        if (first.get_layer() != second.get_layer()) {
          continue;
        }
        result += parallel_length(first, second, maxGap);
      }
    }
    return result;
  }

  private static double parallel_length(PolylineTrace p_first, PolylineTrace p_second, double p_max_center_distance) {
    FloatPoint[] firstCorners = p_first.polyline().corner_approx_arr();
    FloatPoint[] secondCorners = p_second.polyline().corner_approx_arr();
    double result = 0.0;
    for (int i = 0; i < firstCorners.length - 1; i++) {
      for (int j = 0; j < secondCorners.length - 1; j++) {
        if (!segments_parallel(firstCorners[i], firstCorners[i + 1], secondCorners[j], secondCorners[j + 1])) {
          continue;
        }
        double distance = segment_distance(firstCorners[i], firstCorners[i + 1], secondCorners[j], secondCorners[j + 1]);
        if (distance <= p_max_center_distance) {
          result += parallel_overlap_length(firstCorners[i], firstCorners[i + 1], secondCorners[j], secondCorners[j + 1]);
        }
      }
    }
    return result;
  }

  private static boolean segments_parallel(FloatPoint a, FloatPoint b, FloatPoint c, FloatPoint d) {
    double abx = b.x - a.x;
    double aby = b.y - a.y;
    double cdx = d.x - c.x;
    double cdy = d.y - c.y;
    double abLength = Math.sqrt(abx * abx + aby * aby);
    double cdLength = Math.sqrt(cdx * cdx + cdy * cdy);
    if (abLength <= 0.0 || cdLength <= 0.0) {
      return false;
    }
    double cosine = Math.abs((abx * cdx + aby * cdy) / (abLength * cdLength));
    return cosine >= Math.cos(Math.toRadians(10.0));
  }

  private static double parallel_overlap_length(FloatPoint a, FloatPoint b, FloatPoint c, FloatPoint d) {
    double abLength = a.distance(b);
    double cdLength = c.distance(d);
    if (abLength <= 0.0 || cdLength <= 0.0) {
      return 0.0;
    }
    double cOnAb = projection_fraction(c, a, b);
    double dOnAb = projection_fraction(d, a, b);
    double aOnCd = projection_fraction(a, c, d);
    double bOnCd = projection_fraction(b, c, d);
    double abOverlap = Math.max(0.0, Math.min(1.0, Math.max(cOnAb, dOnAb)) - Math.max(0.0, Math.min(cOnAb, dOnAb))) * abLength;
    double cdOverlap = Math.max(0.0, Math.min(1.0, Math.max(aOnCd, bOnCd)) - Math.max(0.0, Math.min(aOnCd, bOnCd))) * cdLength;
    return Math.min(abOverlap, cdOverlap);
  }

  private static double projection_fraction(FloatPoint p_point, FloatPoint p_start, FloatPoint p_end) {
    double dx = p_end.x - p_start.x;
    double dy = p_end.y - p_start.y;
    double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= 0.0) {
      return 0.0;
    }
    return (((p_point.x - p_start.x) * dx) + ((p_point.y - p_start.y) * dy)) / lengthSquared;
  }

  private static double nearest_polyline_distance(PolylineTrace p_first, PolylineTrace p_second) {
    FloatPoint[] firstCorners = p_first.polyline().corner_approx_arr();
    FloatPoint[] secondCorners = p_second.polyline().corner_approx_arr();
    double result = Double.POSITIVE_INFINITY;
    for (int i = 0; i < firstCorners.length - 1; i++) {
      for (int j = 0; j < secondCorners.length - 1; j++) {
        result = Math.min(result, segment_distance(firstCorners[i], firstCorners[i + 1], secondCorners[j], secondCorners[j + 1]));
      }
    }
    return result;
  }

  private static double segment_distance(FloatPoint a, FloatPoint b, FloatPoint c, FloatPoint d) {
    return Math.min(
        Math.min(point_segment_distance(a, c, d), point_segment_distance(b, c, d)),
        Math.min(point_segment_distance(c, a, b), point_segment_distance(d, a, b)));
  }

  private static double point_segment_distance(FloatPoint p, FloatPoint a, FloatPoint b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= 0.0) {
      return p.distance(a);
    }
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSquared;
    t = Math.max(0.0, Math.min(1.0, t));
    return p.distance(new FloatPoint(a.x + t * dx, a.y + t * dy));
  }

  private record PairMeasurements(PairMemberMeasurement first, PairMemberMeasurement second, boolean scoped) {
  }

  private record CoupledCandidate(Point[] first, Point[] second, double centerSpacing, String family) {
    private CoupledCandidate(Point[] first, Point[] second, double centerSpacing) {
      this(first, second, centerSpacing, "unlabeled");
    }
  }

  private record CoupledCandidateEvaluation(
      CoupledCandidate candidate,
      boolean accepted,
      String reason,
      double skew,
      double gap,
      double parallelLength,
      double parallelRatio,
      double uncoupledLength,
      int incompletes,
      int clearanceViolations) {

    private static CoupledCandidateEvaluation rejected(CoupledCandidate p_candidate, String p_reason) {
      return rejected(p_candidate, p_reason, Integer.MAX_VALUE, Integer.MAX_VALUE);
    }

    private static CoupledCandidateEvaluation rejected(
        CoupledCandidate p_candidate,
        String p_reason,
        int p_incompletes,
        int p_clearance_violations) {
      return new CoupledCandidateEvaluation(
          p_candidate,
          false,
          p_reason,
          Double.POSITIVE_INFINITY,
          Double.NaN,
          0.0,
          0.0,
          Double.POSITIVE_INFINITY,
          p_incompletes,
          p_clearance_violations);
    }

    private double score() {
      double gapPenalty = Double.isFinite(gap) ? Math.abs(gap - candidate.centerSpacing()) : candidate.centerSpacing();
      double familyBonus = candidate.family().startsWith("flow_through_skew_compensated") ? 50000.0 : 0.0;
      return familyBonus + (parallelRatio * 10000.0) + parallelLength - (skew * 1000.0)
          - (uncoupledLength * 100.0) - (gapPenalty * 100.0);
    }

    private boolean hasMeasurements() {
      return Double.isFinite(skew) && Double.isFinite(parallelLength);
    }

    private double rejectionScore() {
      return -(clearanceViolations * 1000000.0) - incompletes;
    }
  }

  private record MissingMemberCandidate(Point[] points, double centerSpacing, String family) {
  }

  private record MissingMemberCandidateEvaluation(
      MissingMemberCandidate candidate,
      boolean accepted,
      String reason,
      double skew,
      double gap,
      double parallelLength,
      double parallelRatio,
      double uncoupledLength,
      int incompletes,
      int clearanceViolations) {

    private static MissingMemberCandidateEvaluation rejected(
        MissingMemberCandidate p_candidate,
        String p_reason,
        int p_incompletes,
        int p_clearance_violations) {
      return new MissingMemberCandidateEvaluation(
          p_candidate,
          false,
          p_reason,
          Double.POSITIVE_INFINITY,
          Double.NaN,
          0.0,
          0.0,
          Double.POSITIVE_INFINITY,
          p_incompletes,
          p_clearance_violations);
    }

    private double score() {
      double gapPenalty = Double.isFinite(gap) ? Math.abs(gap - candidate.centerSpacing()) : candidate.centerSpacing();
      return (parallelRatio * 10000.0) + parallelLength - (skew * 1000.0)
          - (uncoupledLength * 100.0) - (gapPenalty * 100.0);
    }

    private double rejectionScore() {
      return -(clearanceViolations * 1000000.0) - incompletes;
    }
  }

  private record TraceStyle(int halfWidth, int clearanceClass) {
  }

  private record ClearanceCheck(int count, String summary) {
  }

  private record PairMeanderCandidate(
      Point[] points,
      double addedLength,
      double coupledOverlap,
      double segmentLength,
      int bumpCount,
      double height) {
    private double score(double p_desired_extra) {
      return Math.abs(addedLength - p_desired_extra)
          - (coupledOverlap * 0.001)
          + (bumpCount * 0.0001)
          + (height * 0.00001);
    }
  }

  private record PairMeanderSegmentCandidate(
      int index,
      double length,
      double normalX,
      double normalY,
      double coupledOverlap) {
  }

  private record FlowThroughBumpCandidate(
      Point[] points,
      double residualSkew,
      FlowThroughBumpShape shape) {
    private int bumpCount() {
      return shape.bumpCount();
    }

    private double height() {
      return shape.height();
    }

    private double maxExcursion() {
      return shape.maxExcursion();
    }

    private double score() {
      return residualSkew + (shape.height() * 0.0001) - (shape.bumpCount() * 0.00001);
    }
  }

  private record FlowThroughBumpShape(
      int bumpCount,
      double height,
      double minPlateau,
      double maxExcursion) {
  }

  private record PairMemberMeasurement(
      int netNo,
      String netName,
      double length,
      boolean scoped,
      String fromPin,
      String toPin,
      Set<Integer> traceIds) {
  }

  private record PathSearchResult(double length, Set<Integer> traceIds) {
  }

  private record PathNode(Item item, double distance) {
  }

  private record GeometricPathPoint(FloatPoint point, double offset) {
  }

  private record GeometricPathEdge(String to, double length, int traceId) {
  }

  private record GeometricPathNode(String key, double distance) {
  }

  private record GeometricPreviousEdge(String from, int traceId) {
  }

  private static int incomplete_count(RoutingBoard p_board) {
    DesignRulesChecker checker = new DesignRulesChecker(p_board, null);
    checker.calculateAllIncompletes();
    return checker.getIncompleteCount();
  }

  private int incomplete_count_for_net(int p_net_no) {
    DesignRulesChecker checker = new DesignRulesChecker(board, null);
    checker.calculateAllIncompletes();
    return checker.getIncompleteCount(p_net_no);
  }

  private static int clearance_violation_count(RoutingBoard p_board) {
    return new DesignRulesChecker(p_board, null).getAllClearanceViolations().size();
  }

  private static int clearance_violation_count_excluding_pair(RoutingBoard p_board, DifferentialPair p_pair) {
    return clearance_check_excluding_pair(p_board, p_pair, 0).count();
  }

  private static ClearanceCheck clearance_check_excluding_pair(
      RoutingBoard p_board,
      DifferentialPair p_pair,
      int p_summary_limit) {
    List<String> samples = new ArrayList<>();
    int count = 0;
    for (ClearanceViolation violation : new DesignRulesChecker(p_board, null).getAllClearanceViolations()) {
      if (!is_acceptable_pair_contact(violation, p_pair)) {
        count++;
        if (samples.size() < p_summary_limit) {
          samples.add(clearance_violation_label(p_board, violation));
        }
      }
    }
    if (samples.isEmpty()) {
      return new ClearanceCheck(count, "");
    }
    String suffix = count > samples.size()
        ? String.format(Locale.US, " (+%d more)", count - samples.size())
        : "";
    return new ClearanceCheck(count, "conflicts: " + String.join(" | ", samples) + suffix);
  }

  private static String clearance_violation_label(RoutingBoard p_board, ClearanceViolation p_violation) {
    String layerName = p_board.layer_structure.arr[p_violation.layer].name;
    double expectedMm = board_to_mm(p_board, p_violation.expected_clearance);
    double actualMm = board_to_mm(p_board, p_violation.actual_clearance);
    return String.format(Locale.US,
        "%s vs %s on %s expected %.3f mm actual %.3f mm",
        item_label(p_board, p_violation.first_item),
        item_label(p_board, p_violation.second_item),
        layerName,
        expectedMm,
        actualMm);
  }

  private static String item_label(RoutingBoard p_board, Item p_item) {
    if (p_item == null) {
      return "<null>";
    }
    StringBuilder label = new StringBuilder();
    label.append(p_item.getClass().getSimpleName()).append("#").append(p_item.get_id_no());
    if (p_item.get_component_no() > 0 && p_board.components != null) {
      Component component = p_board.components.get(p_item.get_component_no());
      if (component != null) {
        label.append("@").append(component.name);
        if (p_item instanceof Pin pin) {
          label.append(".pin").append(pin.pin_no + 1);
        }
      }
    }
    label.append("[nets=");
    if (p_item.net_count() == 0) {
      label.append("none");
    } else {
      for (int i = 0; i < p_item.net_count(); i++) {
        if (i > 0) {
          label.append(",");
        }
        int netNo = p_item.get_net_no(i);
        Net net = p_board.rules.nets.get(netNo);
        label.append(net == null ? Integer.toString(netNo) : net.name);
      }
    }
    label.append(",cl=").append(p_item.clearance_class_no());
    label.append(",fixed=").append(p_item.get_fixed_state()).append("]");
    return label.toString();
  }

  private static boolean is_acceptable_pair_contact(ClearanceViolation p_violation, DifferentialPair p_pair) {
    return is_same_net_pair_contact(p_violation, p_pair);
  }

  private static boolean is_same_net_pair_contact(ClearanceViolation p_violation, DifferentialPair p_pair) {
    if (p_violation == null || p_pair == null || p_violation.first_item == null || p_violation.second_item == null) {
      return false;
    }
    return shares_pair_net(p_violation.first_item, p_violation.second_item, p_pair.first_net_no())
        || shares_pair_net(p_violation.first_item, p_violation.second_item, p_pair.second_net_no());
  }

  private static boolean shares_pair_net(Item p_first, Item p_second, int p_net_no) {
    return p_first.contains_net(p_net_no) && p_second.contains_net(p_net_no);
  }

  private static double mm_to_board(RoutingBoard p_board, double p_mm) {
    return p_mm * p_board.communication.get_resolution(Unit.MM);
  }

  private static double board_to_mm(RoutingBoard p_board, double p_board_value) {
    double mmResolution = p_board.communication.get_resolution(Unit.MM);
    if (mmResolution <= 0) {
      return p_board_value;
    }
    return p_board_value / mmResolution;
  }

  private void logInfo(String p_message) {
    if (job != null) {
      job.logInfo(p_message);
    } else {
      FRLogger.info(p_message);
    }
  }
}
