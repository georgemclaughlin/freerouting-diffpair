package app.freerouting.autoroute;

import app.freerouting.board.Component;
import app.freerouting.board.FixedState;
import app.freerouting.board.Item;
import app.freerouting.board.Pin;
import app.freerouting.board.PolylineTrace;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.Trace;
import app.freerouting.board.Unit;
import app.freerouting.core.RoutingJob;
import app.freerouting.drc.DesignRulesChecker;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Point;
import app.freerouting.logger.FRLogger;
import app.freerouting.rules.DifferentialPair;
import app.freerouting.rules.Net;
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
 * Performs post-route differential-pair length matching.
 *
 * <p>The maze router still routes individual nets. This stage consumes parsed SPECCTRA pair
 * metadata after routing, lengthens the shorter routed member with a local detour, and accepts
 * the edit only when it improves skew without increasing incompletes or clearance violations.
 */
public class DifferentialPairAutorouter {

  private static final double DEFAULT_MAX_SKEW_MM = 1.0;
  private static final double TARGET_SKEW_FRACTION = 0.50;
  private static final int MAX_ATTEMPTS_PER_PAIR = 8;
  private static final int MAX_CANDIDATES_PER_PASS = 128;

  private final RoutingJob job;
  private final RoutingBoard board;
  private final double maxSkewBoard;

  public DifferentialPairAutorouter(RoutingJob p_job) {
    this(p_job, p_job.board, DEFAULT_MAX_SKEW_MM);
  }

  public DifferentialPairAutorouter(RoutingJob p_job, RoutingBoard p_board, double p_max_skew_mm) {
    this.job = p_job;
    this.board = p_board;
    this.maxSkewBoard = mm_to_board(p_board, p_max_skew_mm);
  }

  /**
   * Runs differential-pair length matching for all parsed net-pair descriptors.
   *
   * @return number of accepted pair-length edits.
   */
  public int run() {
    if (board == null || board.rules == null || board.rules.differential_pairs.count() == 0) {
      return 0;
    }
    int acceptedEdits = 0;
    for (int i = 0; i < board.rules.differential_pairs.count(); i++) {
      DifferentialPair pair = board.rules.differential_pairs.get(i);
      acceptedEdits += match_pair(pair);
    }
    if (acceptedEdits > 0) {
      logInfo("Differential-pair post-route matching accepted " + acceptedEdits + " length adjustment"
          + (acceptedEdits == 1 ? "" : "s") + ".");
    }
    return acceptedEdits;
  }

  private int match_pair(DifferentialPair p_pair) {
    Net firstNet = board.rules.nets.get(p_pair.first_net_no());
    Net secondNet = board.rules.nets.get(p_pair.second_net_no());
    if (firstNet == null || secondNet == null) {
      return 0;
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
    PathSearchResult path = shortest_route_path(p_net_no, fromPin, toPin);
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

  private static double longest_segment_length(PolylineTrace p_trace) {
    FloatPoint[] corners = p_trace.polyline().corner_approx_arr();
    double result = 0;
    for (int i = 0; i < corners.length - 1; i++) {
      result = Math.max(result, corners[i].distance(corners[i + 1]));
    }
    return result;
  }

  private record PairMeasurements(PairMemberMeasurement first, PairMemberMeasurement second, boolean scoped) {
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

  private static int incomplete_count(RoutingBoard p_board) {
    DesignRulesChecker checker = new DesignRulesChecker(p_board, null);
    checker.calculateAllIncompletes();
    return checker.getIncompleteCount();
  }

  private static int clearance_violation_count(RoutingBoard p_board) {
    return new DesignRulesChecker(p_board, null).getAllClearanceViolations().size();
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
