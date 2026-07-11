package app.freerouting.autoroute;

import app.freerouting.board.Item;
import app.freerouting.board.Pin;
import app.freerouting.board.RoutingBoard;
import app.freerouting.core.Padstack;
import app.freerouting.geometry.planar.ConvexShape;
import app.freerouting.geometry.planar.FloatLine;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;
import app.freerouting.geometry.planar.Point;
import app.freerouting.rules.Net;
import app.freerouting.rules.NetClass;
import app.freerouting.rules.ViaInfo;
import app.freerouting.rules.ViaRule;
import app.freerouting.settings.RouterSettings;

import java.util.Arrays;
import java.util.Collection;

/**
 * Structure for controlling the autoroute algorithm.
 */
public class AutorouteControl {

  public final RouterSettings settings;
  /**
   * The horizontal and vertical trace costs on each layer
   */
  public final ExpansionCostFactor[] trace_costs;
  public final double[] bendCosts;
  public final boolean with_neckdown;
  /**
   * Defines for each layer, if it may be used for routing.
   */
  final public boolean[] layer_active;
  final int layer_count;
  /**
   * The currently used trace half widths in the autoroute algorithm on each layer
   */
  final int[] trace_half_width;
  /**
   * The currently used compensated trace half widths in the autoroute algorithm on each layer. Equal to trace_half_width if no clearance compensation is used.
   */
  final int[] compensated_trace_half_width;
  final double[] via_radius_arr;
  /**
   * the additional costs to min_normal via_cost for inserting a via between 2 layers
   */
  final ViaCost[] add_via_costs;
  /**
   * The currently used clearance class for traces in the autoroute algorithm
   */
  public int trace_clearance_class_no;
  /**
   * True, if layer change by inserting of vias is allowed
   */
  public boolean vias_allowed;
  /**
   * True, if vias may drill to the pad of SMD pins
   */
  public boolean attach_smd_allowed;
  /**
   * The minimum cost value of all normal vias
   */
  public double min_normal_via_cost;
  public boolean ripup_allowed;
  public int ripup_costs;
  public int ripup_pass_no;
  /**
   * If true, the autoroute algorithm completes after the first drill
   */
  public boolean is_fanout;
  /** Source pin name for targeted fanout diagnostics. */
  public String fanout_start_pin_name;
  /** Source pin center for targeted fanout diagnostics. */
  public Point fanout_start_pin_center;
  /** Source pin layer for targeted fanout diagnostics and limits. */
  public int fanout_start_pin_layer = -1;
  /**
   * Normally true, if the autorouter contains no fanout pass
   */
  public boolean remove_unconnected_vias;
  /**
   * The currently used net number in the autoroute algorithm
   */
  int net_no;
  /**
   * The currently used clearance class for vias in the autoroute algorithm
   */
  int via_clearance_class;
  /**
   * The possible (partial) vias, which can be used by the autorouter
   */
  ViaRule via_rule;
  /**
   * The array of possible via ranges used bei the autorouter
   */
  ViaMask[] via_info_arr;
  /**
   * The lower bound for the first layer of vias
   */
  int via_lower_bound;
  /**
   * The upper bound for the last layer of vias
   */
  int via_upper_bound;
  double max_via_radius;
  /**
   * The width of the region around changed traces, where traces are pulled tight
   */
  int tidy_region_width;
  /**
   * The pull tight accuracy of traces
   */
  int pull_tight_accuracy;
  /**
   * The maximum recursion depth for shoving traces
   */
  int max_shove_trace_recursion_depth;
  /**
   * The maximum recursion depth for shoving obstacles
   */
  int max_shove_via_recursion_depth;
  /**
   * The maximum recursion depth for traces springing over obstacles
   */
  int max_spring_over_recursion_depth;
  /**
   * The minimal cost value of all cheap vias
   */
  double min_cheap_via_cost;
  IntBox router_intent_local_region;
  IntBox[] router_intent_pair_corridors;
  int[] router_intent_pair_corridor_layers;
  PairCenterlineGuide[] router_intent_pair_centerline_guides;
  int router_intent_pair_sibling_net_no;
  double router_intent_pair_allowed_length;
  FloatPoint[] router_intent_pair_locate_guide_path;
  int router_intent_pair_locate_guide_layer;
  int router_intent_pair_locate_guide_side;
  double router_intent_pair_locate_guide_spacing;
  String router_intent_net_name;
 
  /**
   * Creates a new instance of AutorouteControl for the input net
   */
  public AutorouteControl(RoutingBoard p_board, int p_net_no, RouterSettings p_settings) {
    this(p_board, p_settings, p_settings.get_trace_cost_arr());
    init_net(p_net_no, p_board, p_settings.get_via_costs());
  }
 
  /**
   * Creates a new instance of AutorouteControl for the input net
   */
  public AutorouteControl(RoutingBoard p_board, int p_net_no, RouterSettings p_settings, int p_via_costs, ExpansionCostFactor[] p_trace_cost_arr) {
    this(p_board, p_settings, p_trace_cost_arr);
    init_net(p_net_no, p_board, p_via_costs);
  }
 
  /**
   * Creates a new instance of AutorouteControl
   */
  private AutorouteControl(RoutingBoard p_board, RouterSettings p_settings, ExpansionCostFactor[] p_trace_costs_arr) {
    this.settings = p_settings;
    layer_count = p_board.get_layer_count();
    trace_half_width = new int[layer_count];
    compensated_trace_half_width = new int[layer_count];
    layer_active = new boolean[layer_count];
    vias_allowed = p_settings.get_vias_allowed();
    via_radius_arr = new double[layer_count];
    add_via_costs = new ViaCost[layer_count];
    this.bendCosts = new double[layer_count];
    for (int i = 0; i < layer_count; i++) {
      this.bendCosts[i] = p_settings.get_bend_cost(i);
    }
 
    for (int i = 0; i < layer_count; i++) {
      add_via_costs[i] = new ViaCost(layer_count);
      layer_active[i] = p_settings.get_layer_active(i);
    }
    is_fanout = false;
    fanout_start_pin_name = null;
    fanout_start_pin_center = null;
    fanout_start_pin_layer = -1;
    remove_unconnected_vias = true;
    with_neckdown = p_settings.get_automatic_neckdown();
    tidy_region_width = Integer.MAX_VALUE;
    pull_tight_accuracy = 500;
    max_shove_trace_recursion_depth = 20;
    max_shove_via_recursion_depth = 5;
    max_spring_over_recursion_depth = 5;
    for (int i = 0; i < layer_count; i++) {
      for (int j = 0; j < layer_count; j++) {
        add_via_costs[i].to_layer[j] = 0;
      }
    }
    trace_costs = p_trace_costs_arr;
    attach_smd_allowed = false;
    via_lower_bound = 0;
    via_upper_bound = layer_count;
 
    ripup_allowed = false;
    ripup_costs = 1000;
    ripup_pass_no = 1;
    router_intent_local_region = null;
    router_intent_pair_corridors = new IntBox[0];
    router_intent_pair_corridor_layers = new int[0];
    router_intent_pair_centerline_guides = new PairCenterlineGuide[0];
    router_intent_pair_sibling_net_no = -1;
    router_intent_pair_allowed_length = Double.NaN;
    router_intent_pair_locate_guide_path = new FloatPoint[0];
    router_intent_pair_locate_guide_layer = -1;
    router_intent_pair_locate_guide_side = 0;
    router_intent_pair_locate_guide_spacing = Double.NaN;
    router_intent_net_name = null;
  }

  private void init_net(int p_net_no, RoutingBoard p_board, int p_via_costs) {
    net_no = p_net_no;
    Net curr_net = p_board.rules.nets.get(p_net_no);
    router_intent_net_name = curr_net == null ? null : curr_net.name;
    router_intent_local_region = RouterIntentLocalScope.localRegion(p_board, this.settings.intent, curr_net);
    NetClass curr_net_class;
    if (curr_net != null) {
      curr_net_class = curr_net.get_class();
      trace_clearance_class_no = curr_net_class.get_trace_clearance_class();
      via_rule = curr_net_class.get_via_rule();
    } else {
      trace_clearance_class_no = 1;
      via_rule = p_board.rules.via_rules.firstElement();
      curr_net_class = null;
    }
    for (int i = 0; i < layer_count; i++) {
      if (net_no > 0) {
        trace_half_width[i] = p_board.rules.get_trace_half_width(net_no, i);
      } else {
        trace_half_width[i] = p_board.rules.get_trace_half_width(1, i);
      }
      compensated_trace_half_width[i] = trace_half_width[i] + p_board.rules.clearance_matrix.clearance_compensation_value(trace_clearance_class_no, i);
      if (curr_net_class != null && !curr_net_class.is_active_routing_layer(i)) {
        layer_active[i] = false;
      }
      String layerName = p_board.layer_structure.arr[i].name;
      if (!RouterIntentRoutingPolicy.layerAllowed(this.settings.intent, router_intent_net_name, layerName)) {
        layer_active[i] = false;
      }
    }
    if (!RouterIntentRoutingPolicy.viasAllowed(this.settings.intent, router_intent_net_name)) {
      vias_allowed = false;
    }
    if (via_rule.via_count() > 0) {
      this.via_clearance_class = via_rule.get_via(0).get_clearance_class();
    } else {
      this.via_clearance_class = 1;
    }
    this.via_info_arr = new ViaMask[via_rule.via_count()];
    for (int i = 0; i < via_rule.via_count(); i++) {
      ViaInfo curr_via = via_rule.get_via(i);
      if (curr_via.attach_smd_allowed()) {
        this.attach_smd_allowed = true;
      }
      Padstack curr_via_padstack = curr_via.get_padstack();
      int from_layer = curr_via_padstack.from_layer();
      int to_layer = curr_via_padstack.to_layer();
      for (int j = from_layer; j <= to_layer; j++) {
        ConvexShape curr_shape = curr_via_padstack.get_shape(j);
        double curr_radius;
        if (curr_shape != null) {
          curr_radius = 0.5 * curr_shape.max_width();
        } else {
          curr_radius = 0;
        }
        this.via_radius_arr[j] = Math.max(this.via_radius_arr[j], curr_radius);
      }
      via_info_arr[i] = new ViaMask(from_layer, to_layer, curr_via.attach_smd_allowed());
    }

    boolean pure_smd_net = isPureSmdNet(p_board, p_net_no);
    if (!this.attach_smd_allowed && layer_count > 1 && pure_smd_net) {
      // Pure SMD nets must still be able to escape their component layer, even if the DSN marks
      // every padstack as attach-off. This only relaxes the routing gate for same-net fanout;
      // cross-net DRC remains governed by the padstack's attach flag.
      this.attach_smd_allowed = true;
    }

    for (int j = 0; j < this.layer_count; j++) {
      this.via_radius_arr[j] = Math.max(this.via_radius_arr[j], trace_half_width[j]);
      this.max_via_radius = Math.max(this.max_via_radius, this.via_radius_arr[j]);
    }
    double via_cost_factor = this.max_via_radius;
    via_cost_factor = Math.max(via_cost_factor, 1);
    if (pure_smd_net) {
      // Pure SMD boards need a much cheaper via escape to avoid exhausting the local pad channel
      // before the search commits to a layer change.
      via_cost_factor *= 0.1;
    }
    min_normal_via_cost = p_via_costs * via_cost_factor;
    min_normal_via_cost *= RouterIntentRoutingPolicy.viaCostFactor(
        this.settings.intent,
        curr_net == null ? null : curr_net.name);
    min_cheap_via_cost = 0.8 * min_normal_via_cost;
  }

  double routerIntentLocalScopePenalty(FloatPoint p_point, int p_layer) {
    if (p_point == null || router_intent_local_region == null || p_layer < 0 || p_layer >= trace_costs.length) {
      return 0.0;
    }

    double factor = RouterIntentRoutingPolicy.localScopeExitCostFactor(this.settings.intent, router_intent_net_name);
    if (factor <= 0.0 || RouterIntentLocalScope.pointInside(router_intent_local_region, p_point)) {
      return 0.0;
    }

    FloatPoint nearestPoint = router_intent_local_region.nearest_point(p_point);
    double distance = p_point.distance(nearestPoint);
    double averageTraceCost = (trace_costs[p_layer].horizontal + trace_costs[p_layer].vertical) / 2.0;
    return distance * averageTraceCost * factor;
  }

  void setRouterIntentPairCorridors(IntBox[] p_corridors) {
    setRouterIntentPairCorridors(p_corridors, null, -1);
  }

  void setRouterIntentPairCorridors(IntBox[] p_corridors, int[] p_layers, int p_sibling_net_no) {
    router_intent_pair_corridors = p_corridors != null
        ? Arrays.copyOf(p_corridors, p_corridors.length)
        : new IntBox[0];
    router_intent_pair_corridor_layers = new int[router_intent_pair_corridors.length];
    Arrays.fill(router_intent_pair_corridor_layers, -1);
    if (p_layers != null) {
      int layerCount = Math.min(p_layers.length, router_intent_pair_corridor_layers.length);
      System.arraycopy(p_layers, 0, router_intent_pair_corridor_layers, 0, layerCount);
    }
    router_intent_pair_sibling_net_no = p_sibling_net_no > 0 ? p_sibling_net_no : -1;
  }

  void setRouterIntentPairCenterlineGuides(PairCenterlineGuide[] p_guides) {
    router_intent_pair_centerline_guides = p_guides != null
        ? Arrays.copyOf(p_guides, p_guides.length)
        : new PairCenterlineGuide[0];
  }

  void setRouterIntentPairLocateGuide(
      FloatPoint[] p_path,
      int p_layer,
      int p_side,
      double p_spacing) {
    if (p_path == null
        || p_path.length < 2
        || p_layer < 0
        || p_layer >= layer_count
        || (p_side != -1 && p_side != 1)
        || !Double.isFinite(p_spacing)
        || p_spacing <= 0.0) {
      router_intent_pair_locate_guide_path = new FloatPoint[0];
      router_intent_pair_locate_guide_layer = -1;
      router_intent_pair_locate_guide_side = 0;
      router_intent_pair_locate_guide_spacing = Double.NaN;
      return;
    }
    router_intent_pair_locate_guide_path = Arrays.copyOf(p_path, p_path.length);
    router_intent_pair_locate_guide_layer = p_layer;
    router_intent_pair_locate_guide_side = p_side;
    router_intent_pair_locate_guide_spacing = p_spacing;
  }

  boolean hasRouterIntentPairLocateGuide() {
    return router_intent_pair_locate_guide_path.length >= 2
        && router_intent_pair_locate_guide_layer >= 0
        && router_intent_pair_locate_guide_side != 0
        && Double.isFinite(router_intent_pair_locate_guide_spacing)
        && router_intent_pair_locate_guide_spacing > 0.0;
  }

  double routerIntentPairCorridorPenalty(FloatPoint p_point, int p_layer) {
    if (p_point == null
        || p_layer < 0
        || p_layer >= trace_costs.length) {
      return 0.0;
    }

    double averageTraceCost = (trace_costs[p_layer].horizontal + trace_costs[p_layer].vertical) / 2.0;
    double penalty = 0.0;

    if (router_intent_pair_centerline_guides != null
        && router_intent_pair_centerline_guides.length > 0) {
      double centerlineFactor = RouterIntentRoutingPolicy.differentialPairCenterlineBandCostFactor(
          this.settings.intent,
          router_intent_net_name);
      if (centerlineFactor > 0.0) {
        PairCenterlineGuide nearestGuide = null;
        double nearestGuideDistance = Double.POSITIVE_INFINITY;
        for (PairCenterlineGuide guide : router_intent_pair_centerline_guides) {
          if (guide == null || guide.layer != p_layer) {
            continue;
          }
          double distance = guide.applicableDistance(p_point);
          if (distance < nearestGuideDistance) {
            nearestGuideDistance = distance;
            nearestGuide = guide;
          }
        }
        if (nearestGuide != null && Double.isFinite(nearestGuideDistance)) {
          double boundedDeviation = nearestGuide.boundedBandDeviation(p_point);
          if (boundedDeviation > 0.0 && Double.isFinite(boundedDeviation)) {
            penalty += boundedDeviation * averageTraceCost * centerlineFactor;
          }
        }
      }
    }

    if (router_intent_pair_corridors == null || router_intent_pair_corridors.length == 0) {
      return penalty;
    }
    double factor = RouterIntentRoutingPolicy.differentialPairCorridorExitCostFactor(
        this.settings.intent,
        router_intent_net_name);
    if (factor <= 0.0) {
      return penalty;
    }
    double nearestDistance = Double.MAX_VALUE;
    for (int i = 0; i < router_intent_pair_corridors.length; i++) {
      IntBox corridor = router_intent_pair_corridors[i];
      if (corridor == null || corridor.is_empty()) {
        continue;
      }
      int corridorLayer = i < router_intent_pair_corridor_layers.length
          ? router_intent_pair_corridor_layers[i]
          : -1;
      if (corridorLayer >= 0 && corridorLayer != p_layer) {
        continue;
      }
      nearestDistance = Math.min(nearestDistance, corridor.distance(p_point));
    }
    if (nearestDistance == Double.MAX_VALUE || nearestDistance <= 0.0) {
      return penalty;
    }

    return penalty + nearestDistance * averageTraceCost * factor;
  }

  double routerIntentPairCorridorRipupCostFactor(Item p_obstacle_item) {
    if (!routerIntentPairCorridorReservesObstacle(p_obstacle_item)) {
      return 1.0;
    }

    double factor = RouterIntentRoutingPolicy.differentialPairCorridorObstacleRipupCostFactor(
        this.settings.intent,
        router_intent_net_name);
    return factor > 0.0 && factor < 1.0 ? factor : 1.0;
  }

  void setRouterIntentPairAllowedLength(double p_allowed_length) {
    router_intent_pair_allowed_length = p_allowed_length > 0.0 && Double.isFinite(p_allowed_length)
        ? p_allowed_length
        : Double.NaN;
  }

  double routerIntentPairSkewPenalty(double p_path_length, int p_layer) {
    if (!Double.isFinite(router_intent_pair_allowed_length)
        || !Double.isFinite(p_path_length)
        || p_path_length <= router_intent_pair_allowed_length
        || p_layer < 0
        || p_layer >= trace_costs.length) {
      return 0.0;
    }

    double factor = RouterIntentRoutingPolicy.differentialPairSkewExcessCostFactor(
        this.settings.intent,
        router_intent_net_name);
    if (factor <= 0.0) {
      return 0.0;
    }

    double averageTraceCost = (trace_costs[p_layer].horizontal + trace_costs[p_layer].vertical) / 2.0;
    if (averageTraceCost <= 0.0) {
      return 0.0;
    }
    return (p_path_length - router_intent_pair_allowed_length) * averageTraceCost * factor;
  }

  private boolean routerIntentPairCorridorReservesObstacle(Item p_obstacle_item) {
    if (p_obstacle_item == null
        || router_intent_pair_corridors == null
        || router_intent_pair_corridors.length == 0
        || p_obstacle_item.contains_net(net_no)
        || (router_intent_pair_sibling_net_no > 0 && p_obstacle_item.contains_net(router_intent_pair_sibling_net_no))) {
      return false;
    }

    IntBox obstacleBox = p_obstacle_item.bounding_box();
    if (obstacleBox == null || obstacleBox.is_empty()) {
      return false;
    }

    for (int i = 0; i < router_intent_pair_corridors.length; i++) {
      IntBox corridor = router_intent_pair_corridors[i];
      if (corridor == null || corridor.is_empty() || !corridor.intersects(obstacleBox)) {
        continue;
      }

      int corridorLayer = i < router_intent_pair_corridor_layers.length
          ? router_intent_pair_corridor_layers[i]
          : -1;
      if (corridorLayer < 0 || p_obstacle_item.is_on_layer(corridorLayer)) {
        return true;
      }
    }
    return false;
  }

  static final class PairCenterlineGuide {
    final FloatLine centerline;
    final int layer;
    final double targetCenterSpacing;
    final double tolerance;

    PairCenterlineGuide(
        FloatLine p_centerline,
        int p_layer,
        double p_target_center_spacing,
        double p_tolerance) {
      if (p_centerline == null
          || p_centerline.a == null
          || p_centerline.b == null
          || p_layer < 0
          || !Double.isFinite(p_target_center_spacing)
          || p_target_center_spacing <= 0.0
          || !Double.isFinite(p_tolerance)
          || p_tolerance < 0.0) {
        throw new IllegalArgumentException("invalid differential-pair centerline guide");
      }
      centerline = p_centerline;
      layer = p_layer;
      targetCenterSpacing = p_target_center_spacing;
      tolerance = p_tolerance;
    }

    double bandDeviation(FloatPoint p_point) {
      double centerlineDistance = applicableDistance(p_point);
      if (!Double.isFinite(centerlineDistance)) {
        return Double.POSITIVE_INFINITY;
      }
      double lowerBound = Math.max(0.0, targetCenterSpacing - tolerance);
      double upperBound = targetCenterSpacing + tolerance;
      if (centerlineDistance < lowerBound) {
        return lowerBound - centerlineDistance;
      }
      return Math.max(0.0, centerlineDistance - upperBound);
    }

    double applicableDistance(FloatPoint p_point) {
      if (p_point == null) {
        return Double.POSITIVE_INFINITY;
      }
      double deltaX = centerline.b.x - centerline.a.x;
      double deltaY = centerline.b.y - centerline.a.y;
      double lengthSquared = deltaX * deltaX + deltaY * deltaY;
      if (lengthSquared <= 0.0) {
        return Double.POSITIVE_INFINITY;
      }
      double projection = ((p_point.x - centerline.a.x) * deltaX
          + (p_point.y - centerline.a.y) * deltaY) / lengthSquared;
      if (projection < 0.0 || projection > 1.0) {
        return Double.POSITIVE_INFINITY;
      }
      return centerline.segment_distance(p_point);
    }

    double boundedBandDeviation(FloatPoint p_point) {
      double deviation = bandDeviation(p_point);
      return Double.isFinite(deviation)
          ? Math.min(deviation, targetCenterSpacing)
          : Double.POSITIVE_INFINITY;
    }
  }

  private static boolean isPureSmdNet(RoutingBoard p_board, int p_net_no) {
    Collection<Item> net_items = p_board.get_connectable_items(p_net_no);
    if (net_items.isEmpty()) {
      return false;
    }

    for (Item item : net_items) {
      if (!(item instanceof Pin pin) || pin.first_layer() != pin.last_layer()) {
        return false;
      }
    }

    return true;
  }

  /**
   * horizontal and vertical costs for traces on a board layer
   */
  public static class ExpansionCostFactor {

    /**
     * The horizontal expansion cost factor on a layer of the board
     */
    public final double horizontal;
    /**
     * The vertical expansion cost factor on a layer of the board
     */
    public final double vertical;

    public ExpansionCostFactor(double p_horizontal, double p_vertical) {
      horizontal = p_horizontal;
      vertical = p_vertical;
    }
  }

  /**
   * Array of via costs from one layer to the other layers
   */
  static class ViaCost {

    public int[] to_layer;

    private ViaCost(int p_layer_count) {
      to_layer = new int[p_layer_count];
    }
  }

  static class ViaMask {

    final int from_layer;
    final int to_layer;
    final boolean attach_smd_allowed;

    ViaMask(int p_from_layer, int p_to_layer, boolean p_attach_smd_allowed) {
      from_layer = p_from_layer;
      to_layer = p_to_layer;
      attach_smd_allowed = p_attach_smd_allowed;
    }
  }
}
