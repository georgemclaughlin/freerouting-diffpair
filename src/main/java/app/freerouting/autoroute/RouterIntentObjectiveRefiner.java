package app.freerouting.autoroute;

import app.freerouting.board.Component;
import app.freerouting.board.ConductionArea;
import app.freerouting.board.FixedState;
import app.freerouting.board.Item;
import app.freerouting.board.Pin;
import app.freerouting.board.PolylineTrace;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.Trace;
import app.freerouting.board.Unit;
import app.freerouting.board.Via;
import app.freerouting.core.RoutingJob;
import app.freerouting.drc.ClearanceViolation;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Bounded, transactional refinement of routed objectives that ordinary net-by-net routing missed.
 *
 * <p>Every candidate starts from a deep copy, removes only explicitly scoped {@link
 * FixedState#UNFIXED} route copper, and reconnects it through the existing maze/locate/insert
 * pipeline. A candidate is committed only when it fully satisfies the scoped objective without
 * adding an incomplete, a clearance violation, or changing retained fixed copper.
 */
public final class RouterIntentObjectiveRefiner {

  private static final int MAX_LENGTH_GROUP_NETS = 4;
  private static final int MAX_ORDER_CANDIDATES = 8;
  private static final Duration MAX_REFINEMENT_DURATION = Duration.ofSeconds(120);
  private static final double EPSILON_MM = 1e-6;
  private static final double PARALLEL_SINE_TOLERANCE = Math.sin(Math.toRadians(10.0));

  private RouterIntentObjectiveRefiner() {
  }

  public record Result(RoutingBoard board, int acceptedCandidateCount) {
    public boolean accepted() {
      return acceptedCandidateCount > 0;
    }
  }

  public record RetainedCopper(Set<Integer> itemIds, String signature) {
    public RetainedCopper {
      itemIds = Set.copyOf(itemIds);
    }
  }

  public static RetainedCopper captureRetainedCopper(RoutingBoard board) {
    if (board == null) {
      return new RetainedCopper(Set.of(), "");
    }
    Set<Integer> itemIds = new HashSet<>();
    for (Item item : board.get_items()) {
      if (item.get_fixed_state() != FixedState.UNFIXED
          && (item instanceof Trace || item instanceof Via || item instanceof ConductionArea)) {
        itemIds.add(item.get_id_no());
      }
    }
    return new RetainedCopper(itemIds, retainedCopperSignature(board, itemIds));
  }

  public static Result refine(RoutingJob job) {
    return refine(job, captureRetainedCopper(job == null ? null : job.board));
  }

  public static Result refine(RoutingJob job, RetainedCopper retainedCopper) {
    if (job == null
        || job.board == null
        || job.routerSettings == null
        || job.routerSettings.intent == null
        || job.thread == null
        || job.thread.is_stop_auto_router_requested()) {
      return new Result(job == null ? null : job.board, 0);
    }

    if (retainedCopper == null
        || !retainedCopper.signature().equals(retainedCopperSignature(job.board, retainedCopper.itemIds()))) {
      job.logInfo("Router-intent objective refinement skipped because retained source copper changed during routing.");
      return new Result(job.board, 0);
    }

    RouterIntentSettings intent = job.routerSettings.intent;
    if (!intent.hasDifferentialPairIntents() && !intent.hasRouteLengthMatchIntents()) {
      return new Result(job.board, 0);
    }

    BoardSafety initialSafety = BoardSafety.measure(job.board, Set.of(), retainedCopper);
    if (initialSafety.incompleteCount() != 0) {
      job.logInfo("Router-intent objective refinement skipped because the ordinary route is incomplete.");
      return new Result(job.board, 0);
    }

    long deadlineNanos = System.nanoTime() + MAX_REFINEMENT_DURATION.toNanos();
    RoutingBoard acceptedBoard = job.board;
    ObjectiveProtection objectiveProtection = ObjectiveProtection.capturePassing(acceptedBoard, intent);
    int acceptedCount = 0;

    if (intent.routeLengthMatches != null) {
      for (RouterIntentSettings.RouteLengthMatchIntent match : intent.routeLengthMatches) {
        if (deadlineReached(job, deadlineNanos)) {
          break;
        }
        RoutingBoard refined = refineLengthMatch(
            job, acceptedBoard, match, retainedCopper, objectiveProtection, deadlineNanos);
        if (refined != acceptedBoard) {
          acceptedBoard = refined;
          acceptedCount++;
        }
        objectiveProtection.rememberIfPassing(acceptedBoard, match);
      }
    }

    if (intent.differentialPairs != null) {
      for (RouterIntentSettings.DifferentialPairIntent pair : intent.differentialPairs) {
        if (deadlineReached(job, deadlineNanos)) {
          break;
        }
        RoutingBoard refined = refineDifferentialPair(
            job, acceptedBoard, pair, retainedCopper, objectiveProtection, deadlineNanos);
        if (refined != acceptedBoard) {
          acceptedBoard = refined;
          acceptedCount++;
        }
        objectiveProtection.rememberIfPassing(acceptedBoard, pair);
      }
    }

    acceptedBoard.finish_autoroute();
    return new Result(acceptedBoard, acceptedCount);
  }

  private static RoutingBoard refineLengthMatch(
      RoutingJob job,
      RoutingBoard base,
      RouterIntentSettings.RouteLengthMatchIntent match,
      RetainedCopper retainedCopper,
      ObjectiveProtection objectiveProtection,
      long deadlineNanos) {
    if (match == null
        || match.id == null
        || match.id.isBlank()
        || match.nets == null
        || match.nets.length < 2
        || match.nets.length > MAX_LENGTH_GROUP_NETS
        || match.maxSkewMm == null
        || !Double.isFinite(match.maxSkewMm)
        || match.maxSkewMm < 0.0) {
      return base;
    }

    List<ExactConnection> connections = exactTwoPinConnections(base, match.nets);
    if (connections.size() != match.nets.length) {
      job.logDebug("Router-intent length refinement '" + match.id
          + "' is unsupported: every net must resolve to one exact two-pin connection.");
      return base;
    }

    LengthMetrics baseline = LengthMetrics.measure(base, match.nets);
    if (!baseline.valid() || baseline.skewMm() <= match.maxSkewMm + EPSILON_MM) {
      return base;
    }

    Set<String> objectiveNets = new LinkedHashSet<>(Arrays.asList(match.nets));
    BoardSafety baselineSafety = BoardSafety.measure(base, objectiveNets, retainedCopper);
    List<List<String>> orders = boundedLengthOrders(baseline, match.nets);
    RoutingBoard bestBoard = null;
    LengthMetrics bestMetrics = null;

    for (List<String> order : orders) {
      if (deadlineReached(job, deadlineNanos)) {
        break;
      }
      RoutingBoard candidate = base.deepCopy();
      if (candidate == null) {
        continue;
      }
      List<ExactConnection> candidateConnections = resolveConnections(candidate, connections);
      if (candidateConnections.size() != connections.size()) {
        job.logDebug("Router-intent length refinement '" + match.id
            + "' could not re-resolve exact endpoints on a candidate copy.");
        candidate.finish_autoroute();
        continue;
      }
      Map<Item, FixedState> protectedItems = protectRetainedCopper(candidate, retainedCopper);
      protectNonObjectiveCopper(candidate, objectiveNets, protectedItems);
      if (!ripExactConnections(job, match.id, candidate, candidateConnections, retainedCopper)) {
        restoreFixedStates(protectedItems);
        candidate.finish_autoroute();
        continue;
      }

      boolean routed = false;
      try {
        BatchAutorouter candidateRouter = candidateRouter(job, candidate);
        Map<String, ExactConnection> byNet = connectionsByNet(candidateConnections);
        for (int cycle = 0; cycle < 2 && !connectionsComplete(candidate, candidateConnections); cycle++) {
          for (String netName : order) {
            ExactConnection connection = byNet.get(netName);
            if (connection == null
                || !candidateRouter.routeExactConnection(connection.from(), connection.to())) {
              job.logDebug("Router-intent length refinement '" + match.id
                  + "' failed to route exact candidate net '" + netName + "' in order " + order
                  + " (cycle " + (cycle + 1) + ").");
              break;
            }
          }
        }
        routed = connectionsComplete(candidate, candidateConnections);
        if (routed) {
          for (String netName : objectiveNets) {
            protectNet(candidate, netName, protectedItems);
          }
          routed = candidateRouter.routeRemainingConnections(2);
        }
      } finally {
        restoreFixedStates(protectedItems);
        candidate.finish_autoroute();
      }
      if (!routed) {
        continue;
      }

      LengthMetrics metrics = LengthMetrics.measure(candidate, match.nets);
      BoardSafety safety = BoardSafety.measure(candidate, objectiveNets, retainedCopper);
      job.logDebug(String.format(
          Locale.ROOT,
          "Router-intent length refinement '%s' candidate %s: routed=%s, skew=%.4f, incompletes=%d, clearances=%d, objective_clearances=%d, fixed_signature_match=%s.",
          match.id,
          order,
          routed,
          metrics.skewMm(),
          safety.incompleteCount(),
          safety.clearanceViolationCount(),
          safety.objectiveClearanceViolationCount(),
          safety.fixedCopperSignature().equals(baselineSafety.fixedCopperSignature())));
      if (!safety.fixedCopperSignature().equals(baselineSafety.fixedCopperSignature())) {
        job.logDebug("Router-intent length refinement '" + match.id + "' retained-copper diff: "
            + retainedCopperDifference(base, candidate, retainedCopper.itemIds()));
      }
      if (!metrics.valid()
          || metrics.skewMm() > match.maxSkewMm + EPSILON_MM
          || !objectiveProtection.preserves(candidate)
          || !safety.acceptableAgainst(baselineSafety)) {
        continue;
      }
      if (bestMetrics == null || metrics.compareTo(bestMetrics, candidate, bestBoard) < 0) {
        bestBoard = candidate;
        bestMetrics = metrics;
      }
    }

    if (bestBoard == null || bestMetrics == null) {
      job.logDebug("Router-intent length refinement '" + match.id + "' produced no accepted candidate.");
      return base;
    }
    job.logInfo(String.format(
        Locale.ROOT,
        "Router-intent length refinement '%s' accepted an actual-board skew improvement %.4f mm -> %.4f mm.",
        match.id,
        baseline.skewMm(),
        bestMetrics.skewMm()));
    return bestBoard;
  }

  private static RoutingBoard refineDifferentialPair(
      RoutingJob job,
      RoutingBoard base,
      RouterIntentSettings.DifferentialPairIntent pair,
      RetainedCopper retainedCopper,
      ObjectiveProtection objectiveProtection,
      long deadlineNanos) {
    if (pair == null
        || pair.id == null
        || pair.id.isBlank()
        || pair.positiveNet == null
        || pair.negativeNet == null
        || !Boolean.TRUE.equals(pair.routeAsCoupledPair)
        || pair.targetGapMm == null
        || !Double.isFinite(pair.targetGapMm)
        || pair.targetGapMm < 0.0) {
      return base;
    }

    PairMetrics baseline = PairMetrics.measure(base, pair);
    if (baseline.hardPass(pair)) {
      return base;
    }

    ExactConnection positive = exactDeclaredConnection(
        base, pair.positiveNet, pair.positiveFrom, pair.positiveTo);
    ExactConnection negative = exactDeclaredConnection(
        base, pair.negativeNet, pair.negativeFrom, pair.negativeTo);
    if (positive == null || negative == null) {
      job.logDebug("Router-intent pair refinement '" + pair.id
          + "' is unsupported: both members must resolve to exact declared two-pin connections.");
      return base;
    }

    Set<String> objectiveNets = Set.of(pair.positiveNet, pair.negativeNet);
    BoardSafety baselineSafety = BoardSafety.measure(base, objectiveNets, retainedCopper);
    RoutingBoard bestBoard = null;
    PairMetrics bestMetrics = null;

    for (PairCandidatePlan plan : boundedPairPlans(positive, negative)) {
      if (deadlineReached(job, deadlineNanos)) {
        job.logDebug("Router-intent pair refinement '" + pair.id
            + "' stopped before candidate generation: stop_requested="
            + job.thread.is_stop_auto_router_requested()
            + ", deadline_reached=" + (System.nanoTime() >= deadlineNanos) + ".");
        break;
      }
      RoutingBoard candidate = base.deepCopy();
      if (candidate == null) {
        job.logDebug("Router-intent pair refinement '" + pair.id + "' could not deep-copy the board.");
        continue;
      }
      List<ExactConnection> candidateConnections = resolveConnections(candidate, plan.order());
      if (candidateConnections.size() != plan.order().size()) {
        job.logDebug("Router-intent pair refinement '" + pair.id
            + "' could not re-resolve exact endpoints on a candidate copy.");
        candidate.finish_autoroute();
        continue;
      }
      Map<Item, FixedState> protectedItems = protectRetainedCopper(candidate, retainedCopper);
      protectNonObjectiveCopper(candidate, objectiveNets, protectedItems);
      if (!ripExactConnections(job, pair.id, candidate, candidateConnections, retainedCopper)) {
        restoreFixedStates(protectedItems);
        candidate.finish_autoroute();
        continue;
      }

      boolean routed = false;
      try {
        BatchAutorouter candidateRouter = candidateRouter(job, candidate);
        ExactConnection leader = candidateConnections.get(0);
        ExactConnection follower = candidateConnections.get(1);
        if (!candidateRouter.routeExactConnection(leader.from(), leader.to(), true)) {
          job.logDebug("Router-intent pair refinement '" + pair.id
              + "' failed to route leader '" + leader.netName() + "' with " + plan.description() + ".");
          continue;
        }
        protectNet(candidate, leader.netName(), protectedItems);
        double pairLocateGapMm = pair.targetGapMm
            + plan.searchMode().gapToleranceFraction()
                * (pair.gapToleranceMm == null ? 0.0 : pair.gapToleranceMm);
        routed = candidateRouter.routeExactConnection(
            follower.from(),
            follower.to(),
            plan.searchMode().locateGuideSide(),
            pairLocateGapMm);
        if (!routed) {
          job.logDebug("Router-intent pair refinement '" + pair.id
              + "' failed to route follower '" + follower.netName()
              + "' behind leader '" + leader.netName() + "' with " + plan.description() + ".");
        }
        if (routed) {
          protectNet(candidate, follower.netName(), protectedItems);
          routed = candidateRouter.routeRemainingConnections(2);
        }
      } finally {
        restoreFixedStates(protectedItems);
        candidate.finish_autoroute();
      }
      if (!routed) {
        continue;
      }

      PairMetrics metrics = PairMetrics.measure(candidate, pair);
      BoardSafety safety = BoardSafety.measure(candidate, objectiveNets, retainedCopper);
      job.logDebug(String.format(
          Locale.ROOT,
          "Router-intent pair refinement '%s' leader=%s %s: routed=%s, hard_pass=%s, positive_length=%.4f, negative_length=%.4f, coupled=%.4f, uncoupled=%.4f, ratio=%.4f, gap=%.4f, skew=%.4f, positive_vias=%d %s, negative_vias=%d %s, incompletes=%d, clearances=%d, objective_clearances=%d, fixed_signature_match=%s.",
          pair.id,
          candidateConnections.get(0).netName(),
          plan.description(),
          routed,
          metrics.hardPass(pair),
          metrics.positiveLengthMm(),
          metrics.negativeLengthMm(),
          metrics.coupledLengthMm(),
          metrics.uncoupledLengthMm(),
          metrics.parallelRatio(),
          metrics.representativeGapMm(),
          metrics.skewMm(),
          metrics.positiveViaSpans().size(),
          metrics.positiveViaSpans(),
          metrics.negativeViaSpans().size(),
          metrics.negativeViaSpans(),
          safety.incompleteCount(),
          safety.clearanceViolationCount(),
          safety.objectiveClearanceViolationCount(),
          safety.fixedCopperSignature().equals(baselineSafety.fixedCopperSignature())));
      if (!safety.fixedCopperSignature().equals(baselineSafety.fixedCopperSignature())) {
        job.logDebug("Router-intent pair refinement '" + pair.id + "' retained-copper diff: "
            + retainedCopperDifference(base, candidate, retainedCopper.itemIds()));
      }
      if (!metrics.hardPass(pair)
          || !objectiveProtection.preserves(candidate)
          || !safety.acceptableAgainst(baselineSafety)) {
        continue;
      }
      if (bestMetrics == null || metrics.compareTo(bestMetrics, candidate, bestBoard) < 0) {
        bestBoard = candidate;
        bestMetrics = metrics;
      }
    }

    if (bestBoard == null || bestMetrics == null) {
      job.logDebug("Router-intent pair refinement '" + pair.id + "' produced no accepted candidate.");
      return base;
    }
    job.logInfo(String.format(
        Locale.ROOT,
        "Router-intent pair refinement '%s' accepted coupled ratio %.4f, gap %.4f mm, skew %.4f mm.",
        pair.id,
        bestMetrics.parallelRatio(),
        bestMetrics.representativeGapMm(),
        bestMetrics.skewMm()));
    return bestBoard;
  }

  private static List<PairCandidatePlan> boundedPairPlans(
      ExactConnection positive,
      ExactConnection negative) {
    List<PairCandidatePlan> result = new ArrayList<>();
    for (List<ExactConnection> order : List.of(
        List.of(positive, negative),
        List.of(negative, positive))) {
      for (PairSearchMode searchMode : PairSearchMode.values()) {
        result.add(new PairCandidatePlan(order, searchMode));
      }
    }
    return List.copyOf(result);
  }

  private static BatchAutorouter candidateRouter(RoutingJob job, RoutingBoard candidate) {
    BatchAutorouter router = new BatchAutorouter(
        job.thread,
        candidate,
        job.routerSettings,
        true,
        true,
        job.routerSettings.get_start_ripup_costs(),
        job.routerSettings.trace_pull_tight_accuracy);
    router.job = job;
    return router;
  }

  private static boolean deadlineReached(RoutingJob job, long deadlineNanos) {
    return System.nanoTime() >= deadlineNanos
        || job.thread == null
        || job.thread.is_stop_auto_router_requested();
  }

  private static List<ExactConnection> exactTwoPinConnections(RoutingBoard board, String[] netNames) {
    List<ExactConnection> result = new ArrayList<>();
    for (String netName : netNames) {
      if (netName == null || netName.isBlank()) {
        return List.of();
      }
      List<Pin> pins = pinsForNet(board, netName);
      if (pins.size() != 2) {
        return List.of();
      }
      pins.sort(Comparator.comparing(pin -> specctraPinRef(board, pin)));
      result.add(new ExactConnection(netName, specctraPinRef(board, pins.get(0)), specctraPinRef(board, pins.get(1)),
          pins.get(0), pins.get(1)));
    }
    return result;
  }

  private static ExactConnection exactDeclaredConnection(
      RoutingBoard board,
      String netName,
      String fromRef,
      String toRef) {
    if (netName == null || fromRef == null || toRef == null) {
      return null;
    }
    List<Pin> pins = pinsForNet(board, netName);
    if (pins.size() != 2) {
      return null;
    }
    Pin from = resolvePin(board, fromRef, netName);
    Pin to = resolvePin(board, toRef, netName);
    if (from == null || to == null || from == to) {
      return null;
    }
    return new ExactConnection(netName, specctraPinRef(board, from), specctraPinRef(board, to), from, to);
  }

  private static List<ExactConnection> resolveConnections(
      RoutingBoard candidate,
      Collection<ExactConnection> sourceConnections) {
    List<ExactConnection> result = new ArrayList<>();
    for (ExactConnection source : sourceConnections) {
      Pin from = resolvePin(candidate, source.fromRef(), source.netName());
      Pin to = resolvePin(candidate, source.toRef(), source.netName());
      if (from == null || to == null || from == to) {
        return List.of();
      }
      result.add(new ExactConnection(source.netName(), source.fromRef(), source.toRef(), from, to));
    }
    return result;
  }

  private static boolean ripExactConnections(
      RoutingJob job,
      String objectiveId,
      RoutingBoard candidate,
      Collection<ExactConnection> connections,
      RetainedCopper retainedCopper) {
    if (connections.isEmpty()) {
      return false;
    }
    LinkedHashSet<Item> routeItems = new LinkedHashSet<>();
    LinkedHashSet<Integer> netNumbers = new LinkedHashSet<>();
    for (ExactConnection connection : connections) {
      Net net = candidate.rules.nets.get(connection.netName(), 1);
      if (net == null || !connection.from().get_connected_set(net.net_number).contains(connection.to())) {
        job.logDebug("Router-intent refinement '" + objectiveId
            + "' cannot rip net '" + connection.netName() + "': declared endpoints are not connected.");
        return false;
      }
      netNumbers.add(net.net_number);
      for (Item item : connection.from().get_connected_set(net.net_number)) {
        if (item == connection.from() || item == connection.to()) {
          continue;
        }
        if (!(item instanceof Trace) && !(item instanceof Via)) {
          job.logDebug("Router-intent refinement '" + objectiveId
              + "' cannot rip net '" + connection.netName() + "': connected item #"
              + item.get_id_no() + " is " + item.getClass().getSimpleName() + ".");
          return false;
        }
        boolean generatedPinExit = item.get_fixed_state() == FixedState.SHOVE_FIXED
            && !retainedCopper.itemIds().contains(item.get_id_no());
        if (item.get_fixed_state() != FixedState.UNFIXED && !generatedPinExit) {
          job.logDebug("Router-intent refinement '" + objectiveId
              + "' cannot rip net '" + connection.netName() + "': route item #"
              + item.get_id_no() + " is " + item.get_fixed_state() + ".");
          return false;
        }
        routeItems.add(item);
      }
    }
    if (routeItems.isEmpty() || !candidate.remove_items(routeItems)) {
      job.logDebug("Router-intent refinement '" + objectiveId
          + "' cannot rip its exact route items.");
      return false;
    }
    for (int netNo : netNumbers) {
      candidate.combine_traces(netNo);
    }
    candidate.finish_autoroute();
    for (ExactConnection connection : connections) {
      Net net = candidate.rules.nets.get(connection.netName(), 1);
      if (net == null || connection.from().get_connected_set(net.net_number).contains(connection.to())) {
        return false;
      }
    }
    return true;
  }

  private static void protectNet(
      RoutingBoard board,
      String netName,
      Map<Item, FixedState> changed) {
    Net net = board.rules.nets.get(netName, 1);
    if (net == null) {
      return;
    }
    for (Item item : board.get_connectable_items(net.net_number)) {
      if ((item instanceof Trace || item instanceof Via)
          && item.get_fixed_state() == FixedState.UNFIXED) {
        changed.put(item, item.get_fixed_state());
        item.set_fixed_state(FixedState.USER_FIXED);
      }
    }
  }

  private static Map<Item, FixedState> protectRetainedCopper(
      RoutingBoard board,
      RetainedCopper retainedCopper) {
    Map<Item, FixedState> changed = new LinkedHashMap<>();
    for (Item item : board.get_items()) {
      if (retainedCopper.itemIds().contains(item.get_id_no())
          && item.get_fixed_state() == FixedState.SHOVE_FIXED) {
        changed.put(item, item.get_fixed_state());
        item.set_fixed_state(FixedState.USER_FIXED);
      }
    }
    return changed;
  }

  private static void protectNonObjectiveCopper(
      RoutingBoard board,
      Set<String> objectiveNets,
      Map<Item, FixedState> changed) {
    Set<Integer> objectiveNetNumbers = netNumbers(board, objectiveNets);
    for (Item item : board.get_items()) {
      if (!(item instanceof Trace || item instanceof Via || item instanceof ConductionArea)
          || containsAnyNet(item, objectiveNetNumbers)
          || item.get_fixed_state().ordinal() >= FixedState.USER_FIXED.ordinal()) {
        continue;
      }
      changed.putIfAbsent(item, item.get_fixed_state());
      item.set_fixed_state(FixedState.USER_FIXED);
    }
  }

  private static void restoreFixedStates(Map<Item, FixedState> changed) {
    for (Map.Entry<Item, FixedState> entry : changed.entrySet()) {
      entry.getKey().set_fixed_state(entry.getValue());
    }
  }

  private static Map<String, ExactConnection> connectionsByNet(Collection<ExactConnection> connections) {
    Map<String, ExactConnection> result = new HashMap<>();
    for (ExactConnection connection : connections) {
      result.put(connection.netName(), connection);
    }
    return result;
  }

  private static boolean connectionsComplete(
      RoutingBoard board,
      Collection<ExactConnection> connections) {
    for (ExactConnection connection : connections) {
      Net net = board.rules.nets.get(connection.netName(), 1);
      if (net == null || !connection.from().get_connected_set(net.net_number).contains(connection.to())) {
        return false;
      }
    }
    return true;
  }

  private static List<Pin> pinsForNet(RoutingBoard board, String netName) {
    Net net = board.rules.nets.get(netName, 1);
    if (net == null) {
      return List.of();
    }
    List<Pin> result = new ArrayList<>();
    for (Pin pin : board.get_pins()) {
      if (pin.contains_net(net.net_number)) {
        result.add(pin);
      }
    }
    return result;
  }

  private static Pin resolvePin(RoutingBoard board, String requestedRef, String netName) {
    for (Pin pin : pinsForNet(board, netName)) {
      String specctraRef = specctraPinRef(board, pin);
      String dottedRef = dottedPinRef(board, pin);
      if (requestedRef.equals(specctraRef) || requestedRef.equals(dottedRef)) {
        return pin;
      }
    }
    return null;
  }

  private static String specctraPinRef(RoutingBoard board, Pin pin) {
    Component component = board.components.get(pin.get_component_no());
    return component == null ? "" : component.name + "-" + pin.name();
  }

  private static String dottedPinRef(RoutingBoard board, Pin pin) {
    Component component = board.components.get(pin.get_component_no());
    return component == null ? "" : component.name + "." + pin.name();
  }

  private static List<List<String>> boundedLengthOrders(LengthMetrics baseline, String[] netNames) {
    List<List<String>> result = new ArrayList<>();
    List<String> provenFirst = baseline.longestShortestRemainingOrder();
    if (!provenFirst.isEmpty()) {
      result.add(provenFirst);
    }
    List<String> sorted = new ArrayList<>(Arrays.asList(netNames));
    sorted.sort(String::compareTo);
    addPermutations(sorted, 0, result);
    LinkedHashSet<List<String>> unique = new LinkedHashSet<>(result);
    return unique.stream().limit(MAX_ORDER_CANDIDATES).toList();
  }

  private static void addPermutations(List<String> values, int index, List<List<String>> result) {
    if (result.size() >= MAX_ORDER_CANDIDATES * 2) {
      return;
    }
    if (index >= values.size()) {
      result.add(List.copyOf(values));
      return;
    }
    for (int i = index; i < values.size(); i++) {
      java.util.Collections.swap(values, index, i);
      addPermutations(values, index + 1, result);
      java.util.Collections.swap(values, index, i);
    }
  }

  private static final class ObjectiveProtection {
    private final Map<String, RouterIntentSettings.RouteLengthMatchIntent> lengthMatches = new LinkedHashMap<>();
    private final Map<String, RouterIntentSettings.DifferentialPairIntent> differentialPairs = new LinkedHashMap<>();

    static ObjectiveProtection capturePassing(RoutingBoard board, RouterIntentSettings intent) {
      ObjectiveProtection result = new ObjectiveProtection();
      if (intent.routeLengthMatches != null) {
        for (RouterIntentSettings.RouteLengthMatchIntent match : intent.routeLengthMatches) {
          result.rememberIfPassing(board, match);
        }
      }
      if (intent.differentialPairs != null) {
        for (RouterIntentSettings.DifferentialPairIntent pair : intent.differentialPairs) {
          result.rememberIfPassing(board, pair);
        }
      }
      return result;
    }

    void rememberIfPassing(RoutingBoard board, RouterIntentSettings.RouteLengthMatchIntent match) {
      if (match != null
          && match.id != null
          && match.nets != null
          && match.nets.length >= 2
          && match.maxSkewMm != null) {
        LengthMetrics metrics = LengthMetrics.measure(board, match.nets);
        if (metrics.valid() && metrics.skewMm() <= match.maxSkewMm + EPSILON_MM) {
          lengthMatches.put(match.id, match);
        }
      }
    }

    void rememberIfPassing(RoutingBoard board, RouterIntentSettings.DifferentialPairIntent pair) {
      if (supportsMeasurement(pair) && PairMetrics.measure(board, pair).hardPass(pair)) {
        differentialPairs.put(pair.id, pair);
      }
    }

    boolean preserves(RoutingBoard board) {
      for (RouterIntentSettings.RouteLengthMatchIntent match : lengthMatches.values()) {
        LengthMetrics metrics = LengthMetrics.measure(board, match.nets);
        if (!metrics.valid() || metrics.skewMm() > match.maxSkewMm + EPSILON_MM) {
          return false;
        }
      }
      for (RouterIntentSettings.DifferentialPairIntent pair : differentialPairs.values()) {
        if (!PairMetrics.measure(board, pair).hardPass(pair)) {
          return false;
        }
      }
      return true;
    }

    private static boolean supportsMeasurement(RouterIntentSettings.DifferentialPairIntent pair) {
      return pair != null
          && pair.id != null
          && pair.positiveNet != null
          && pair.negativeNet != null
          && pair.targetGapMm != null
          && Double.isFinite(pair.targetGapMm)
          && pair.targetGapMm > 0.0;
    }
  }

  private record ExactConnection(
      String netName,
      String fromRef,
      String toRef,
      Pin from,
      Pin to) {
  }

  private record PairCandidatePlan(
      List<ExactConnection> order,
      PairSearchMode searchMode) {

    String description() {
      return "follower_search=" + searchMode.name().toLowerCase(Locale.ROOT)
          + ", force_no_vias=true";
    }
  }

  private enum PairSearchMode {
    LOCATE_UPPER_LEFT(-1, 1.0),
    LOCATE_UPPER_RIGHT(1, 1.0),
    LOCATE_TARGET_LEFT(-1, 0.0),
    LOCATE_TARGET_RIGHT(1, 0.0);

    private final int locateGuideSide;
    private final double gapToleranceFraction;

    PairSearchMode(
        int locateGuideSide,
        double gapToleranceFraction) {
      this.locateGuideSide = locateGuideSide;
      this.gapToleranceFraction = gapToleranceFraction;
    }

    int locateGuideSide() {
      return locateGuideSide;
    }

    double gapToleranceFraction() {
      return gapToleranceFraction;
    }
  }

  private record LengthMetrics(Map<String, Double> lengthsMm, double skewMm, double totalLengthMm) {
    static LengthMetrics measure(RoutingBoard board, String[] netNames) {
      double resolution = board.communication.get_resolution(Unit.MM);
      if (!Double.isFinite(resolution) || resolution <= 0.0) {
        return new LengthMetrics(Map.of(), Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      }
      Map<String, Double> lengths = new LinkedHashMap<>();
      double shortest = Double.POSITIVE_INFINITY;
      double longest = 0.0;
      double total = 0.0;
      for (String netName : netNames) {
        Net net = board.rules.nets.get(netName, 1);
        if (net == null) {
          return new LengthMetrics(Map.of(), Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        double length = net.get_trace_length() / resolution;
        lengths.put(netName, length);
        shortest = Math.min(shortest, length);
        longest = Math.max(longest, length);
        total += length;
      }
      return new LengthMetrics(Map.copyOf(lengths), longest - shortest, total);
    }

    boolean valid() {
      return !lengthsMm.isEmpty()
          && lengthsMm.values().stream().allMatch(length -> Double.isFinite(length) && length > 0.0)
          && Double.isFinite(skewMm);
    }

    List<String> longestShortestRemainingOrder() {
      if (lengthsMm.size() < 2) {
        return List.of();
      }
      List<String> ordered = new ArrayList<>(lengthsMm.keySet());
      ordered.sort(Comparator
          .comparingDouble((String net) -> lengthsMm.get(net))
          .reversed()
          .thenComparing(String::compareTo));
      String longest = ordered.remove(0);
      ordered.sort(Comparator
          .comparingDouble((String net) -> lengthsMm.get(net))
          .thenComparing(String::compareTo));
      String shortest = ordered.remove(0);
      List<String> result = new ArrayList<>();
      result.add(longest);
      result.add(shortest);
      result.addAll(ordered);
      return result;
    }

    int compareTo(LengthMetrics other, RoutingBoard board, RoutingBoard otherBoard) {
      int skewCompare = Double.compare(skewMm, other.skewMm);
      if (skewCompare != 0) {
        return skewCompare;
      }
      int lengthCompare = Double.compare(totalLengthMm, other.totalLengthMm);
      if (lengthCompare != 0) {
        return lengthCompare;
      }
      return board.get_hash().compareTo(otherBoard.get_hash());
    }
  }

  record PairMetrics(
      boolean valid,
      double positiveLengthMm,
      double negativeLengthMm,
      double skewMm,
      double coupledLengthMm,
      double parallelRatio,
      double representativeGapMm,
      double uncoupledLengthMm,
      Set<String> positiveLayers,
      Set<String> negativeLayers,
      List<String> positiveViaSpans,
      List<String> negativeViaSpans,
      Set<Double> positiveWidthsMm,
      Set<Double> negativeWidthsMm) {

    static PairMetrics measure(RoutingBoard board, RouterIntentSettings.DifferentialPairIntent pair) {
      Net positive = board.rules.nets.get(pair.positiveNet, 1);
      Net negative = board.rules.nets.get(pair.negativeNet, 1);
      double resolution = board.communication.get_resolution(Unit.MM);
      if (positive == null || negative == null || !Double.isFinite(resolution) || resolution <= 0.0) {
        return invalid();
      }
      double positiveLength = positive.get_trace_length() / resolution;
      double negativeLength = negative.get_trace_length() / resolution;
      List<RouteSegment> positiveSegments = routeSegments(board, positive.net_number);
      List<RouteSegment> negativeSegments = routeSegments(board, negative.net_number);
      double toleranceMm = pair.gapToleranceMm != null && pair.gapToleranceMm >= 0.0
          ? pair.gapToleranceMm
          : 0.0;
      double lowGap = Math.max(0.0, pair.targetGapMm - toleranceMm) * resolution;
      double highGap = (pair.targetGapMm + toleranceMm) * resolution;
      Coupling positiveCoupling = coupling(
          positiveSegments,
          negativeSegments,
          lowGap,
          highGap);
      Coupling negativeCoupling = coupling(
          negativeSegments,
          positiveSegments,
          lowGap,
          highGap);
      double referenceLength = Math.max(positiveLength, negativeLength);
      double coupledLength = Math.min(
          Math.max(positiveCoupling.length(), negativeCoupling.length()) / resolution,
          referenceLength);
      double ratio = referenceLength > 0.0 ? coupledLength / referenceLength : 0.0;
      double representativeGap = Math.min(
          positiveCoupling.representativeGap(),
          negativeCoupling.representativeGap());
      return new PairMetrics(
          positiveLength > 0.0 && negativeLength > 0.0,
          positiveLength,
          negativeLength,
          Math.abs(positiveLength - negativeLength),
          coupledLength,
          ratio,
          representativeGap / resolution,
          Math.max(0.0, referenceLength - coupledLength),
          layers(board, positive.net_number),
          layers(board, negative.net_number),
          viaSpans(board, positive.net_number),
          viaSpans(board, negative.net_number),
          widths(board, positive.net_number, resolution),
          widths(board, negative.net_number, resolution));
    }

    private static PairMetrics invalid() {
      return new PairMetrics(false, 0.0, 0.0, Double.POSITIVE_INFINITY, 0.0, 0.0,
          Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Set.of(), Set.of(), List.of(), List.of(), Set.of(),
          Set.of());
    }

    boolean hardPass(RouterIntentSettings.DifferentialPairIntent pair) {
      if (!valid || coupledLengthMm <= EPSILON_MM || !Double.isFinite(representativeGapMm)) {
        return false;
      }
      double tolerance = pair.gapToleranceMm != null && pair.gapToleranceMm >= 0.0
          ? pair.gapToleranceMm
          : 0.0;
      if (representativeGapMm < Math.max(0.0, pair.targetGapMm - tolerance) - EPSILON_MM
          || representativeGapMm > pair.targetGapMm + tolerance + EPSILON_MM) {
        return false;
      }
      if (pair.maxSkewMm != null && skewMm > pair.maxSkewMm + EPSILON_MM) {
        return false;
      }
      if (Boolean.TRUE.equals(pair.requireParallelEvidence) && coupledLengthMm <= EPSILON_MM) {
        return false;
      }
      if (pair.minParallelLengthRatio != null
          && parallelRatio + EPSILON_MM < pair.minParallelLengthRatio) {
        return false;
      }
      if (pair.maxUncoupledLengthMm != null
          && uncoupledLengthMm > pair.maxUncoupledLengthMm + EPSILON_MM) {
        return false;
      }
      if (pair.allowedLayers != null && pair.allowedLayers.length > 0) {
        Set<String> allowed = Set.of(pair.allowedLayers);
        if (!allowed.containsAll(positiveLayers) || !allowed.containsAll(negativeLayers)) {
          return false;
        }
      }
      if (Boolean.TRUE.equals(pair.sameLayerRequired)) {
        Set<String> common = new HashSet<>(positiveLayers);
        common.retainAll(negativeLayers);
        Set<String> all = new HashSet<>(positiveLayers);
        all.addAll(negativeLayers);
        if (common.isEmpty() || all.size() != 1) {
          return false;
        }
      }
      if (pair.maxViasPerNet != null
          && (positiveViaSpans.size() > pair.maxViasPerNet
              || negativeViaSpans.size() > pair.maxViasPerNet)) {
        return false;
      }
      if (Boolean.TRUE.equals(pair.matchedViaTransitionsRequired)
          && !positiveViaSpans.equals(negativeViaSpans)) {
        return false;
      }
      if (pair.targetWidthMm != null) {
        Set<Double> allowedWidths = new HashSet<>();
        allowedWidths.add(roundMm(pair.targetWidthMm));
        if (pair.endpointEscapeWidthMm != null) {
          allowedWidths.add(roundMm(pair.endpointEscapeWidthMm));
        }
        if (!allowedWidths.containsAll(positiveWidthsMm) || !allowedWidths.containsAll(negativeWidthsMm)) {
          return false;
        }
      }
      return true;
    }

    int compareTo(PairMetrics other, RoutingBoard board, RoutingBoard otherBoard) {
      int ratioCompare = -Double.compare(parallelRatio, other.parallelRatio);
      if (ratioCompare != 0) {
        return ratioCompare;
      }
      int skewCompare = Double.compare(skewMm, other.skewMm);
      if (skewCompare != 0) {
        return skewCompare;
      }
      return board.get_hash().compareTo(otherBoard.get_hash());
    }
  }

  private record RouteSegment(
      FloatPoint start,
      FloatPoint end,
      int layer,
      double halfWidth,
      double length) {
  }

  private record Coupling(double length, double representativeGap) {
  }

  private static List<RouteSegment> routeSegments(RoutingBoard board, int netNo) {
    List<RouteSegment> result = new ArrayList<>();
    for (Trace trace : board.get_traces()) {
      if (!(trace instanceof PolylineTrace polylineTrace) || !trace.contains_net(netNo)) {
        continue;
      }
      FloatPoint[] corners = polylineTrace.polyline().corner_approx_arr();
      for (int i = 1; i < corners.length; i++) {
        double length = corners[i - 1].distance(corners[i]);
        if (length > 0.0) {
          result.add(new RouteSegment(
              corners[i - 1], corners[i], trace.get_layer(), trace.get_half_width(), length));
        }
      }
    }
    return result;
  }

  private static Coupling coupling(
      List<RouteSegment> first,
      List<RouteSegment> second,
      double lowGap,
      double highGap) {
    double coupledLength = 0.0;
    double representativeGap = Double.POSITIVE_INFINITY;
    for (RouteSegment firstSegment : first) {
      List<double[]> intervals = new ArrayList<>();
      double dx = firstSegment.end().x - firstSegment.start().x;
      double dy = firstSegment.end().y - firstSegment.start().y;
      double firstLength = firstSegment.length();
      double ux = dx / firstLength;
      double uy = dy / firstLength;
      for (RouteSegment secondSegment : second) {
        if (firstSegment.layer() != secondSegment.layer()) {
          continue;
        }
        double secondDx = secondSegment.end().x - secondSegment.start().x;
        double secondDy = secondSegment.end().y - secondSegment.start().y;
        double sine = Math.abs(dx * secondDy - dy * secondDx)
            / (firstLength * secondSegment.length());
        if (sine > PARALLEL_SINE_TOLERANCE) {
          continue;
        }
        double centerDistance = RouterIntentBoardScorer.segmentDistance(
            firstSegment.start(), firstSegment.end(), secondSegment.start(), secondSegment.end());
        double gap = Math.max(0.0, centerDistance - firstSegment.halfWidth() - secondSegment.halfWidth());
        if (gap < lowGap - 1e-6 || gap > highGap + 1e-6) {
          continue;
        }
        double firstProjection = (secondSegment.start().x - firstSegment.start().x) * ux
            + (secondSegment.start().y - firstSegment.start().y) * uy;
        double secondProjection = (secondSegment.end().x - firstSegment.start().x) * ux
            + (secondSegment.end().y - firstSegment.start().y) * uy;
        double start = Math.max(0.0, Math.min(firstProjection, secondProjection));
        double end = Math.min(firstLength, Math.max(firstProjection, secondProjection));
        if (end > start) {
          intervals.add(new double[] {start, end});
          representativeGap = Math.min(representativeGap, gap);
        }
      }
      intervals.sort(Comparator.comparingDouble(interval -> interval[0]));
      double intervalStart = Double.NaN;
      double intervalEnd = Double.NaN;
      for (double[] interval : intervals) {
        if (!Double.isFinite(intervalStart)) {
          intervalStart = interval[0];
          intervalEnd = interval[1];
        } else if (interval[0] <= intervalEnd + 1e-6) {
          intervalEnd = Math.max(intervalEnd, interval[1]);
        } else {
          coupledLength += intervalEnd - intervalStart;
          intervalStart = interval[0];
          intervalEnd = interval[1];
        }
      }
      if (Double.isFinite(intervalStart)) {
        coupledLength += intervalEnd - intervalStart;
      }
    }
    return new Coupling(coupledLength, representativeGap);
  }

  private static Set<String> layers(RoutingBoard board, int netNo) {
    Set<String> result = new LinkedHashSet<>();
    for (Trace trace : board.get_traces()) {
      if (trace.contains_net(netNo)) {
        result.add(board.layer_structure.arr[trace.get_layer()].name);
      }
    }
    return Set.copyOf(result);
  }

  private static List<String> viaSpans(RoutingBoard board, int netNo) {
    List<String> result = new ArrayList<>();
    for (Via via : board.get_vias()) {
      if (via.contains_net(netNo)) {
        result.add(via.first_layer() + "-" + via.last_layer());
      }
    }
    result.sort(String::compareTo);
    return List.copyOf(result);
  }

  private static Set<Double> widths(RoutingBoard board, int netNo, double resolution) {
    Set<Double> result = new HashSet<>();
    for (Trace trace : board.get_traces()) {
      if (trace.contains_net(netNo)) {
        result.add(roundMm(2.0 * trace.get_half_width() / resolution));
      }
    }
    return Set.copyOf(result);
  }

  private static double roundMm(double value) {
    return Math.rint(value * 1_000_000.0) / 1_000_000.0;
  }

  private record BoardSafety(
      int incompleteCount,
      int clearanceViolationCount,
      int objectiveClearanceViolationCount,
      Set<String> clearanceViolationKeys,
      String fixedCopperSignature,
      String nonObjectiveCopperSignature) {

    static BoardSafety measure(
        RoutingBoard board,
        Set<String> objectiveNets,
        RetainedCopper retainedCopper) {
      DesignRulesChecker checker = new DesignRulesChecker(board, null);
      checker.calculateAllIncompletes();
      Set<Integer> objectiveNetNumbers = netNumbers(board, objectiveNets);
      Collection<ClearanceViolation> violations = checker.getAllClearanceViolations();
      Set<String> violationKeys = new HashSet<>();
      int objectiveViolations = 0;
      for (ClearanceViolation violation : violations) {
        violationKeys.add(clearanceViolationKey(violation));
        if (containsAnyNet(violation.first_item, objectiveNetNumbers)
            || containsAnyNet(violation.second_item, objectiveNetNumbers)) {
          objectiveViolations++;
        }
      }
      return new BoardSafety(
          checker.getIncompleteCount(),
          violations.size(),
          objectiveViolations,
          Set.copyOf(violationKeys),
          retainedCopperSignature(board, retainedCopper.itemIds()),
          RouterIntentObjectiveRefiner.nonObjectiveCopperSignature(board, objectiveNetNumbers));
    }

    boolean acceptableAgainst(BoardSafety baseline) {
      return incompleteCount == baseline.incompleteCount
          && hasNoNewClearanceViolations(baseline.clearanceViolationKeys, clearanceViolationKeys)
          && objectiveClearanceViolationCount == 0
          && fixedCopperSignature.equals(baseline.fixedCopperSignature)
          && nonObjectiveCopperSignature.equals(baseline.nonObjectiveCopperSignature);
    }
  }

  static boolean hasNoNewClearanceViolations(Set<String> baseline, Set<String> candidate) {
    return baseline.containsAll(candidate);
  }

  private static String clearanceViolationKey(ClearanceViolation violation) {
    int firstId = violation.first_item.get_id_no();
    int secondId = violation.second_item.get_id_no();
    return Math.min(firstId, secondId) + "-" + Math.max(firstId, secondId) + "-" + violation.layer;
  }

  private static Set<Integer> netNumbers(RoutingBoard board, Set<String> netNames) {
    Set<Integer> result = new HashSet<>();
    for (String netName : netNames) {
      Net net = board.rules.nets.get(netName, 1);
      if (net != null) {
        result.add(net.net_number);
      }
    }
    return result;
  }

  private static boolean containsAnyNet(Item item, Set<Integer> netNumbers) {
    for (int i = 0; i < item.net_count(); i++) {
      if (netNumbers.contains(item.get_net_no(i))) {
        return true;
      }
    }
    return false;
  }

  private static String retainedCopperSignature(RoutingBoard board, Set<Integer> retainedItemIds) {
    return String.join("\n", retainedCopperRecords(board, retainedItemIds).values());
  }

  private static String nonObjectiveCopperSignature(RoutingBoard board, Set<Integer> objectiveNetNumbers) {
    Map<Integer, String> records = new java.util.TreeMap<>();
    for (Item item : board.get_items()) {
      if ((item instanceof Trace || item instanceof Via || item instanceof ConductionArea)
          && !containsAnyNet(item, objectiveNetNumbers)) {
        records.put(item.get_id_no(), copperRecord(item));
      }
    }
    return String.join("\n", records.values());
  }

  private static Map<Integer, String> retainedCopperRecords(
      RoutingBoard board,
      Set<Integer> retainedItemIds) {
    Map<Integer, String> records = new java.util.TreeMap<>();
    for (Item item : board.get_items()) {
      if (!retainedItemIds.contains(item.get_id_no())) {
        continue;
      }
      records.put(item.get_id_no(), copperRecord(item));
    }
    return records;
  }

  private static String copperRecord(Item item) {
    StringBuilder record = new StringBuilder()
        .append(item.get_id_no()).append('|')
        .append(item.getClass().getSimpleName()).append('|')
        .append(item.get_fixed_state()).append('|')
        .append(item.clearance_class_no()).append('|');
    for (int i = 0; i < item.net_count(); i++) {
      if (i > 0) {
        record.append(',');
      }
      record.append(item.get_net_no(i));
    }
    if (item instanceof PolylineTrace trace) {
      record.append('|').append(trace.get_layer()).append('|').append(trace.get_half_width());
      appendCorners(record, trace.polyline().corner_approx_arr());
    } else if (item instanceof Via via) {
      record.append('|').append(via.get_center())
          .append('|').append(via.first_layer()).append('-').append(via.last_layer());
    } else if (item instanceof ConductionArea area) {
      record.append('|').append(area.get_layer());
      appendCorners(record, area.get_area().corner_approx_arr());
    } else {
      IntBox box = item.bounding_box();
      record.append('|').append(box == null ? "null" : box.toString());
    }
    return record.toString();
  }

  private static void appendCorners(StringBuilder record, FloatPoint[] corners) {
    for (FloatPoint corner : corners) {
      record.append('|').append(corner.x).append(',').append(corner.y);
    }
  }

  private static String retainedCopperDifference(
      RoutingBoard expected,
      RoutingBoard actual,
      Set<Integer> retainedItemIds) {
    Map<Integer, String> expectedRecords = retainedCopperRecords(expected, retainedItemIds);
    Map<Integer, String> actualRecords = retainedCopperRecords(actual, retainedItemIds);
    List<String> differences = new ArrayList<>();
    for (int itemId : retainedItemIds.stream().sorted().toList()) {
      String expectedRecord = expectedRecords.get(itemId);
      String actualRecord = actualRecords.get(itemId);
      if (!java.util.Objects.equals(expectedRecord, actualRecord)) {
        differences.add("item#" + itemId
            + " expected=" + (expectedRecord == null ? "missing" : expectedRecord)
            + " actual=" + (actualRecord == null ? "missing" : actualRecord));
      }
      if (differences.size() >= 8) {
        break;
      }
    }
    return differences.isEmpty() ? "signature changed without an item-record difference" : String.join("; ", differences);
  }
}
