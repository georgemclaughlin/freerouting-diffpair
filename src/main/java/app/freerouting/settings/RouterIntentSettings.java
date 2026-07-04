package app.freerouting.settings;

import app.freerouting.util.gson.GsonProvider;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.annotations.SerializedName;
import java.io.IOException;
import java.io.Reader;
import java.io.Serializable;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class RouterIntentSettings implements Serializable, Cloneable {
  private static final String SCHEMA = "kicad-agent-router-intent";
  private static final int SCHEMA_VERSION = 3;
  private static final Set<String> TOP_LEVEL_KEYS = Set.of(
      "schema",
      "schema_version",
      "profile",
      "settings",
      "net_intents",
      "critical_paths",
      "differential_pairs",
      "route_length_matches",
      "local_support",
      "block_ports");
  private static final Set<String> SETTINGS_KEYS = Set.of(
      "deterministic_seed",
      "single_thread_default",
      "route_order",
      "ripup_protection");
  private static final Set<String> ROUTE_ORDER_VALUES = Set.of("priority_then_local_scope");
  private static final Set<String> SETTINGS_RIPUP_PROTECTION_VALUES = Set.of("source_copper_and_critical");
  private static final Set<String> NET_INTENT_KEYS = Set.of(
      "net",
      "priority",
      "scope",
      "preferred_layers",
      "plane_layers",
      "route_class",
      "track_width_mm",
      "clearance_mm",
      "via_diameter_mm",
      "via_drill_mm",
      "ripup_protection",
      "critical_path_ids",
      "differential_pair_ids",
      "route_length_match_ids",
      "local_support_ids",
      "block_port_ids",
      "source_copper_ids");
  private static final Set<String> CRITICAL_PATH_KEYS = Set.of(
      "id",
      "net",
      "priority",
      "from",
      "to",
      "preferred_layers",
      "max_length_mm",
      "max_extra_mm",
      "max_ratio");
  private static final Set<String> DIFFERENTIAL_PAIR_KEYS = Set.of(
      "id",
      "positive_net",
      "negative_net",
      "priority",
      "positive_from",
      "positive_to",
      "negative_from",
      "negative_to",
      "positive_preferred_layers",
      "negative_preferred_layers",
      "max_skew_mm",
      "max_stub_mm",
      "require_parallel_evidence");
  private static final Set<String> ROUTE_LENGTH_MATCH_KEYS = Set.of(
      "id",
      "nets",
      "priority",
      "max_skew_mm");
  private static final Set<String> LOCAL_SUPPORT_KEYS = Set.of(
      "id",
      "kind",
      "nets",
      "pad_refs",
      "priority",
      "preferred_layers",
      "max_distance_mm",
      "max_return_distance_mm");
  private static final Set<String> BLOCK_PORT_KEYS = Set.of(
      "id",
      "block",
      "port",
      "kind",
      "net",
      "pad_ref",
      "x_mm",
      "y_mm",
      "boundary_name",
      "boundary_center_x_mm",
      "boundary_center_y_mm",
      "boundary_width_mm",
      "boundary_height_mm");
  private static final Set<String> PRIORITY_VALUES = Set.of("normal", "high", "critical");
  private static final Set<String> SCOPE_VALUES = Set.of("normal", "local", "global");
  private static final Set<String> RIPUP_PROTECTION_VALUES = Set.of(
      "none",
      "critical",
      "local_support",
      "source_copper");
  private static final Set<String> LOCAL_SUPPORT_KIND_VALUES = Set.of(
      "connector_filtering",
      "local_decoupling",
      "package_net_via_escape",
      "recovery_test_access",
      "return_path",
      "same_net_pad_tie",
      "shield_return",
      "usb2_diff_pair",
      "usb_series_resistor_placement");
  private static final Set<String> BLOCK_PORT_KIND_VALUES = Set.of(
      "power_input",
      "power_output",
      "ground",
      "signal");

  @SerializedName("intent_file")
  public String intentFile;
  @SerializedName("schema")
  public String schema;
  @SerializedName("schema_version")
  public Integer schemaVersion;
  @SerializedName("profile")
  public String profile;
  @SerializedName("settings")
  public RuntimeSettings settings;
  @SerializedName("net_intents")
  public NetIntent[] netIntents;
  @SerializedName("critical_paths")
  public CriticalPathIntent[] criticalPaths;
  @SerializedName("differential_pairs")
  public DifferentialPairIntent[] differentialPairs;
  @SerializedName("route_length_matches")
  public RouteLengthMatchIntent[] routeLengthMatches;
  @SerializedName("local_support")
  public LocalSupportIntent[] localSupport;
  @SerializedName("block_ports")
  public BlockPortIntent[] blockPorts;

  private transient Map<String, NetIntent> netIntentByName;

  public static RouterIntentSettings load(Path path) throws IOException {
    try (Reader reader = Files.newBufferedReader(path)) {
      JsonElement rootElement = JsonParser.parseReader(reader);
      if (!rootElement.isJsonObject()) {
        throw new IllegalArgumentException("router intent payload root must be an object");
      }
      JsonObject root = rootElement.getAsJsonObject();
      validateObjectKeys("router intent", root, TOP_LEVEL_KEYS);
      validateNestedObjectKeys(root);

      RouterIntentSettings result = GsonProvider.GSON.fromJson(root, RouterIntentSettings.class);
      if (result == null) {
        throw new IllegalArgumentException("router intent payload is empty");
      }
      result.intentFile = path.toAbsolutePath().normalize().toString();
      result.validateSchema();
      result.rebuildNetIntentIndex();
      return result;
    }
  }

  private static void validateNestedObjectKeys(JsonObject root) {
    JsonElement settingsElement = root.get("settings");
    if (settingsElement != null && !settingsElement.isJsonNull()) {
      if (!settingsElement.isJsonObject()) {
        throw new IllegalArgumentException("router intent settings must be an object");
      }
      JsonObject settingsObject = settingsElement.getAsJsonObject();
      validateObjectKeys("router intent settings", settingsObject, SETTINGS_KEYS);
      validateOptionalStringValue("router intent settings", settingsObject, "route_order", ROUTE_ORDER_VALUES);
      validateOptionalStringValue(
          "router intent settings",
          settingsObject,
          "ripup_protection",
          SETTINGS_RIPUP_PROTECTION_VALUES);
    }

    validateArrayObjectKeys(root, "net_intents", NET_INTENT_KEYS);
    validateArrayObjectKeys(root, "critical_paths", CRITICAL_PATH_KEYS);
    validateArrayObjectKeys(root, "differential_pairs", DIFFERENTIAL_PAIR_KEYS);
    validateArrayObjectKeys(root, "route_length_matches", ROUTE_LENGTH_MATCH_KEYS);
    validateArrayObjectKeys(root, "local_support", LOCAL_SUPPORT_KEYS);
    validateArrayObjectKeys(root, "block_ports", BLOCK_PORT_KEYS);
    validateArrayStringValues(root, "net_intents", "priority", PRIORITY_VALUES);
    validateArrayStringValues(root, "net_intents", "scope", SCOPE_VALUES);
    validateArrayStringValues(root, "net_intents", "ripup_protection", RIPUP_PROTECTION_VALUES);
    validateArrayStringValues(root, "critical_paths", "priority", PRIORITY_VALUES);
    validateArrayStringValues(root, "differential_pairs", "priority", PRIORITY_VALUES);
    validateArrayStringValues(root, "route_length_matches", "priority", PRIORITY_VALUES);
    validateArrayStringValues(root, "local_support", "kind", LOCAL_SUPPORT_KIND_VALUES);
    validateArrayStringValues(root, "local_support", "priority", PRIORITY_VALUES);
    validateArrayStringValues(root, "block_ports", "kind", BLOCK_PORT_KIND_VALUES);
  }

  private static void validateArrayObjectKeys(JsonObject root, String field, Set<String> allowedKeys) {
    JsonElement arrayElement = root.get(field);
    if (arrayElement == null || arrayElement.isJsonNull()) {
      return;
    }
    if (!arrayElement.isJsonArray()) {
      throw new IllegalArgumentException("router intent " + field + " must be an array");
    }
    for (JsonElement element : arrayElement.getAsJsonArray()) {
      if (!element.isJsonObject()) {
        throw new IllegalArgumentException("router intent " + field + " entries must be objects");
      }
      validateObjectKeys("router intent " + field + " entry", element.getAsJsonObject(), allowedKeys);
    }
  }

  private static void validateObjectKeys(String label, JsonObject object, Set<String> allowedKeys) {
    for (String key : object.keySet()) {
      if (!allowedKeys.contains(key)) {
        throw new IllegalArgumentException(label + " has unsupported field: " + key);
      }
    }
  }

  private static void validateArrayStringValues(
      JsonObject root,
      String arrayField,
      String valueField,
      Set<String> allowedValues) {
    JsonElement arrayElement = root.get(arrayField);
    if (arrayElement == null || arrayElement.isJsonNull()) {
      return;
    }
    for (JsonElement element : arrayElement.getAsJsonArray()) {
      validateOptionalStringValue(
          "router intent " + arrayField + " entry",
          element.getAsJsonObject(),
          valueField,
          allowedValues);
    }
  }

  private static void validateOptionalStringValue(
      String label,
      JsonObject object,
      String field,
      Set<String> allowedValues) {
    JsonElement valueElement = object.get(field);
    if (valueElement == null || valueElement.isJsonNull()) {
      return;
    }
    if (!valueElement.isJsonPrimitive() || !valueElement.getAsJsonPrimitive().isString()) {
      throw new IllegalArgumentException(label + " field " + field + " must be a string");
    }
    String value = valueElement.getAsString();
    if (!allowedValues.contains(value)) {
      throw new IllegalArgumentException(label + " field " + field + " has unsupported value: " + value);
    }
  }

  private void validateSchema() {
    if (!SCHEMA.equals(this.schema)) {
      throw new IllegalArgumentException("unsupported router intent schema: " + this.schema);
    }
    if (!Integer.valueOf(SCHEMA_VERSION).equals(this.schemaVersion)) {
      throw new IllegalArgumentException("unsupported router intent schema_version: " + this.schemaVersion);
    }
  }

  public boolean hasNetIntents() {
    return this.netIntents != null && this.netIntents.length > 0;
  }

  public boolean hasDifferentialPairIntents() {
    return this.differentialPairs != null && this.differentialPairs.length > 0;
  }

  public boolean hasRouteLengthMatchIntents() {
    return this.routeLengthMatches != null && this.routeLengthMatches.length > 0;
  }

  public int priorityRankForNet(String netName) {
    NetIntent intent = netIntent(netName);
    return intent == null ? 0 : intent.priorityRank();
  }

  public int scopeRankForNet(String netName) {
    NetIntent intent = netIntent(netName);
    return intent == null ? 0 : intent.scopeRank();
  }

  public int ripupProtectionRankForNet(String netName) {
    NetIntent intent = netIntent(netName);
    return intent == null ? 0 : intent.ripupProtectionRank();
  }

  public boolean hasLocalScopeIntent(String netName) {
    NetIntent intent = netIntent(netName);
    return intent != null && intent.scope == Scope.LOCAL;
  }

  public boolean hasLocalConfinementIntent(String netName) {
    return hasLocalScopeIntent(netName) || hasBlockPortIntent(netName);
  }

  public String differentialPairGroupForNet(String netName) {
    DifferentialPairIntent pair = differentialPairForNet(netName);
    if (pair != null) {
      if (pair.id != null && !pair.id.isEmpty()) {
        return pair.id;
      }
      return pair.positiveNet + ":" + pair.negativeNet;
    }

    NetIntent intent = netIntent(netName);
    if (intent != null && intent.differentialPairIds != null && intent.differentialPairIds.length > 0) {
      return intent.differentialPairIds[0];
    }
    return null;
  }

  public int differentialPairMemberRankForNet(String netName) {
    DifferentialPairIntent pair = differentialPairForNet(netName);
    if (pair == null) {
      return 2;
    }
    if (netName.equals(pair.positiveNet)) {
      return 0;
    }
    if (netName.equals(pair.negativeNet)) {
      return 1;
    }
    return 2;
  }

  public String differentialPairSiblingNetForNet(String netName) {
    DifferentialPairIntent pair = differentialPairForNet(netName);
    if (pair == null) {
      return null;
    }
    if (netName.equals(pair.positiveNet)) {
      return pair.negativeNet;
    }
    if (netName.equals(pair.negativeNet)) {
      return pair.positiveNet;
    }
    return null;
  }

  public Double differentialPairMaxSkewMmForNet(String netName) {
    DifferentialPairIntent pair = differentialPairForNet(netName);
    return pair == null ? null : pair.maxSkewMm;
  }

  public boolean areDifferentialPairMembers(String leftNetName, String rightNetName) {
    String leftGroup = differentialPairGroupForNet(leftNetName);
    String rightGroup = differentialPairGroupForNet(rightNetName);
    return leftGroup != null && leftGroup.equals(rightGroup);
  }

  public RouteLengthMatchIntent[] routeLengthMatchesForNet(String netName) {
    if (netName == null || this.routeLengthMatches == null || this.routeLengthMatches.length == 0) {
      return new RouteLengthMatchIntent[0];
    }

    List<RouteLengthMatchIntent> result = new ArrayList<>();
    for (RouteLengthMatchIntent match : this.routeLengthMatches) {
      if (match != null && contains(match.nets, netName)) {
        result.add(match);
      }
    }
    return result.toArray(new RouteLengthMatchIntent[0]);
  }

  public String routeLengthMatchGroupForNet(String netName) {
    return routeLengthMatchGroup(routeLengthMatchForNet(netName));
  }

  public String[] routeLengthMatchSiblingNetsForNet(String netName) {
    RouteLengthMatchIntent match = routeLengthMatchForNet(netName);
    if (match == null || match.nets == null || match.nets.length == 0) {
      return new String[0];
    }

    List<String> result = new ArrayList<>();
    for (String net : match.nets) {
      if (net != null && !net.equals(netName)) {
        result.add(net);
      }
    }
    return result.toArray(new String[0]);
  }

  public Double routeLengthMatchMaxSkewMmForNet(String netName) {
    RouteLengthMatchIntent match = routeLengthMatchForNet(netName);
    return match == null ? null : match.maxSkewMm;
  }

  public Priority routeLengthMatchPriorityForNet(String netName) {
    RouteLengthMatchIntent match = routeLengthMatchForNet(netName);
    return match == null ? null : match.priority;
  }

  public LocalSupportIntent[] localSupportForNet(String netName) {
    if (netName == null || this.localSupport == null || this.localSupport.length == 0) {
      return new LocalSupportIntent[0];
    }

    List<LocalSupportIntent> result = new ArrayList<>();
    for (LocalSupportIntent support : this.localSupport) {
      if (support != null && contains(support.nets, netName)) {
        result.add(support);
      }
    }
    return result.toArray(new LocalSupportIntent[0]);
  }

  public boolean hasBlockPortIntent(String netName) {
    return blockPortsForNet(netName).length > 0;
  }

  public BlockPortIntent[] blockPortsForNet(String netName) {
    if (netName == null || this.blockPorts == null || this.blockPorts.length == 0) {
      return new BlockPortIntent[0];
    }

    List<BlockPortIntent> result = new ArrayList<>();
    for (BlockPortIntent blockPort : this.blockPorts) {
      if (blockPort != null && netName.equals(blockPort.net)) {
        result.add(blockPort);
      }
    }
    return result.toArray(new BlockPortIntent[0]);
  }

  public boolean hasPreferredLayerIntent(String netName) {
    NetIntent intent = netIntent(netName);
    return intent != null && intent.preferredLayers != null && intent.preferredLayers.length > 0;
  }

  public boolean isPreferredLayerForNet(String netName, String layerName) {
    NetIntent intent = netIntent(netName);
    if (intent == null || intent.preferredLayers == null || intent.preferredLayers.length == 0) {
      return true;
    }
    for (String preferredLayer : intent.preferredLayers) {
      if (preferredLayer != null && preferredLayer.equals(layerName)) {
        return true;
      }
    }
    return false;
  }

  public boolean hasPlaneLayerIntent(String netName) {
    NetIntent intent = netIntent(netName);
    return intent != null && intent.planeLayers != null && intent.planeLayers.length > 0;
  }

  public boolean isPlaneLayerForNet(String netName, String layerName) {
    NetIntent intent = netIntent(netName);
    if (layerName == null || intent == null || intent.planeLayers == null || intent.planeLayers.length == 0) {
      return false;
    }
    for (String planeLayer : intent.planeLayers) {
      if (layerName.equals(planeLayer)) {
        return true;
      }
    }
    return false;
  }

  private DifferentialPairIntent differentialPairForNet(String netName) {
    if (netName == null || this.differentialPairs == null || this.differentialPairs.length == 0) {
      return null;
    }
    for (DifferentialPairIntent pair : this.differentialPairs) {
      if (pair == null) {
        continue;
      }
      if (netName.equals(pair.positiveNet) || netName.equals(pair.negativeNet)) {
        return pair;
      }
    }
    return null;
  }

  private RouteLengthMatchIntent routeLengthMatchForNet(String netName) {
    if (netName == null || this.routeLengthMatches == null || this.routeLengthMatches.length == 0) {
      return null;
    }
    for (RouteLengthMatchIntent match : this.routeLengthMatches) {
      if (match != null && contains(match.nets, netName)) {
        return match;
      }
    }
    return null;
  }

  private NetIntent netIntent(String netName) {
    if (netName == null) {
      return null;
    }
    if (this.netIntentByName == null) {
      rebuildNetIntentIndex();
    }
    return this.netIntentByName.get(netName);
  }

  private void rebuildNetIntentIndex() {
    Map<String, NetIntent> index = new HashMap<>();
    if (this.netIntents != null) {
      for (NetIntent intent : this.netIntents) {
        if (intent != null && intent.net != null) {
          index.put(intent.net, intent);
        }
      }
    }
    this.netIntentByName = index;
  }

  private static boolean contains(String[] values, String expected) {
    if (values == null) {
      return false;
    }
    for (String value : values) {
      if (expected.equals(value)) {
        return true;
      }
    }
    return false;
  }

  private static String routeLengthMatchGroup(RouteLengthMatchIntent match) {
    if (match == null) {
      return null;
    }
    if (match.id != null && !match.id.isEmpty()) {
      return match.id;
    }
    if (match.nets == null || match.nets.length == 0) {
      return null;
    }
    List<String> nets = new ArrayList<>();
    for (String net : match.nets) {
      if (net != null && !net.isEmpty()) {
        nets.add(net);
      }
    }
    return nets.isEmpty() ? null : String.join(":", nets);
  }

  @Override
  public RouterIntentSettings clone() {
    try {
      RouterIntentSettings result = (RouterIntentSettings) super.clone();
      result.settings = this.settings != null ? this.settings.clone() : null;
      result.netIntents = this.netIntents != null ? this.netIntents.clone() : null;
      result.criticalPaths = this.criticalPaths != null ? this.criticalPaths.clone() : null;
      result.differentialPairs = this.differentialPairs != null ? this.differentialPairs.clone() : null;
      result.routeLengthMatches = this.routeLengthMatches != null ? this.routeLengthMatches.clone() : null;
      result.localSupport = this.localSupport != null ? this.localSupport.clone() : null;
      result.blockPorts = this.blockPorts != null ? this.blockPorts.clone() : null;
      result.rebuildNetIntentIndex();
      return result;
    } catch (CloneNotSupportedException e) {
      throw new AssertionError(e);
    }
  }

  public static class RuntimeSettings implements Serializable, Cloneable {
    @SerializedName("deterministic_seed")
    public Integer deterministicSeed;
    @SerializedName("single_thread_default")
    public Boolean singleThreadDefault;
    @SerializedName("route_order")
    public RouteOrder routeOrder;
    @SerializedName("ripup_protection")
    public RuntimeRipupProtection ripupProtection;

    @Override
    public RuntimeSettings clone() {
      try {
        return (RuntimeSettings) super.clone();
      } catch (CloneNotSupportedException e) {
        throw new AssertionError(e);
      }
    }
  }

  public static class NetIntent implements Serializable {
    @SerializedName("net")
    public String net;
    @SerializedName("priority")
    public Priority priority;
    @SerializedName("scope")
    public Scope scope;
    @SerializedName("preferred_layers")
    public String[] preferredLayers;
    @SerializedName("plane_layers")
    public String[] planeLayers;
    @SerializedName("route_class")
    public String routeClass;
    @SerializedName("track_width_mm")
    public Double trackWidthMm;
    @SerializedName("clearance_mm")
    public Double clearanceMm;
    @SerializedName("via_diameter_mm")
    public Double viaDiameterMm;
    @SerializedName("via_drill_mm")
    public Double viaDrillMm;
    @SerializedName("ripup_protection")
    public RipupProtection ripupProtection;
    @SerializedName("critical_path_ids")
    public String[] criticalPathIds;
    @SerializedName("differential_pair_ids")
    public String[] differentialPairIds;
    @SerializedName("route_length_match_ids")
    public String[] routeLengthMatchIds;
    @SerializedName("local_support_ids")
    public String[] localSupportIds;
    @SerializedName("block_port_ids")
    public String[] blockPortIds;
    @SerializedName("source_copper_ids")
    public String[] sourceCopperIds;

    public int priorityRank() {
      return priority == null ? 0 : priority.rank;
    }

    public int scopeRank() {
      return scope == null ? 0 : scope.rank;
    }

    public int ripupProtectionRank() {
      return ripupProtection == null ? 0 : ripupProtection.rank;
    }
  }

  public static class CriticalPathIntent implements Serializable {
    @SerializedName("id")
    public String id;
    @SerializedName("net")
    public String net;
    @SerializedName("priority")
    public Priority priority;
    @SerializedName("from")
    public String from;
    @SerializedName("to")
    public String to;
    @SerializedName("preferred_layers")
    public String[] preferredLayers;
    @SerializedName("max_length_mm")
    public Double maxLengthMm;
    @SerializedName("max_extra_mm")
    public Double maxExtraMm;
    @SerializedName("max_ratio")
    public Double maxRatio;
  }

  public static class DifferentialPairIntent implements Serializable {
    @SerializedName("id")
    public String id;
    @SerializedName("positive_net")
    public String positiveNet;
    @SerializedName("negative_net")
    public String negativeNet;
    @SerializedName("priority")
    public Priority priority;
    @SerializedName("positive_from")
    public String positiveFrom;
    @SerializedName("positive_to")
    public String positiveTo;
    @SerializedName("negative_from")
    public String negativeFrom;
    @SerializedName("negative_to")
    public String negativeTo;
    @SerializedName("positive_preferred_layers")
    public String[] positivePreferredLayers;
    @SerializedName("negative_preferred_layers")
    public String[] negativePreferredLayers;
    @SerializedName("max_skew_mm")
    public Double maxSkewMm;
    @SerializedName("max_stub_mm")
    public Double maxStubMm;
    @SerializedName("require_parallel_evidence")
    public Boolean requireParallelEvidence;
  }

  public static class RouteLengthMatchIntent implements Serializable {
    @SerializedName("id")
    public String id;
    @SerializedName("nets")
    public String[] nets;
    @SerializedName("priority")
    public Priority priority;
    @SerializedName("max_skew_mm")
    public Double maxSkewMm;
  }

  public static class LocalSupportIntent implements Serializable {
    @SerializedName("id")
    public String id;
    @SerializedName("kind")
    public LocalSupportKind kind;
    @SerializedName("nets")
    public String[] nets;
    @SerializedName("pad_refs")
    public String[] padRefs;
    @SerializedName("priority")
    public Priority priority;
    @SerializedName("preferred_layers")
    public String[] preferredLayers;
    @SerializedName("max_distance_mm")
    public Double maxDistanceMm;
    @SerializedName("max_return_distance_mm")
    public Double maxReturnDistanceMm;
  }

  public static class BlockPortIntent implements Serializable {
    @SerializedName("id")
    public String id;
    @SerializedName("block")
    public String block;
    @SerializedName("port")
    public String port;
    @SerializedName("kind")
    public BlockPortKind kind;
    @SerializedName("net")
    public String net;
    @SerializedName("pad_ref")
    public String padRef;
    @SerializedName("x_mm")
    public Double xMm;
    @SerializedName("y_mm")
    public Double yMm;
    @SerializedName("boundary_name")
    public String boundaryName;
    @SerializedName("boundary_center_x_mm")
    public Double boundaryCenterXMm;
    @SerializedName("boundary_center_y_mm")
    public Double boundaryCenterYMm;
    @SerializedName("boundary_width_mm")
    public Double boundaryWidthMm;
    @SerializedName("boundary_height_mm")
    public Double boundaryHeightMm;
  }

  public enum RouteOrder {
    @SerializedName("priority_then_local_scope")
    PRIORITY_THEN_LOCAL_SCOPE
  }

  public enum RuntimeRipupProtection {
    @SerializedName("source_copper_and_critical")
    SOURCE_COPPER_AND_CRITICAL
  }

  public enum Priority {
    @SerializedName("normal")
    NORMAL(1),
    @SerializedName("high")
    HIGH(2),
    @SerializedName("critical")
    CRITICAL(3);

    private final int rank;

    Priority(int rank) {
      this.rank = rank;
    }
  }

  public enum Scope {
    @SerializedName("normal")
    NORMAL(0),
    @SerializedName("global")
    GLOBAL(1),
    @SerializedName("local")
    LOCAL(2);

    private final int rank;

    Scope(int rank) {
      this.rank = rank;
    }
  }

  public enum RipupProtection {
    @SerializedName("none")
    NONE(0),
    @SerializedName("critical")
    CRITICAL(1),
    @SerializedName("local_support")
    LOCAL_SUPPORT(2),
    @SerializedName("source_copper")
    SOURCE_COPPER(3);

    private final int rank;

    RipupProtection(int rank) {
      this.rank = rank;
    }
  }

  public enum LocalSupportKind {
    @SerializedName("connector_filtering")
    CONNECTOR_FILTERING,
    @SerializedName("local_decoupling")
    LOCAL_DECOUPLING,
    @SerializedName("package_net_via_escape")
    PACKAGE_NET_VIA_ESCAPE,
    @SerializedName("recovery_test_access")
    RECOVERY_TEST_ACCESS,
    @SerializedName("return_path")
    RETURN_PATH,
    @SerializedName("same_net_pad_tie")
    SAME_NET_PAD_TIE,
    @SerializedName("shield_return")
    SHIELD_RETURN,
    @SerializedName("usb2_diff_pair")
    USB2_DIFF_PAIR,
    @SerializedName("usb_series_resistor_placement")
    USB_SERIES_RESISTOR_PLACEMENT
  }

  public enum BlockPortKind {
    @SerializedName("power_input")
    POWER_INPUT,
    @SerializedName("power_output")
    POWER_OUTPUT,
    @SerializedName("ground")
    GROUND,
    @SerializedName("signal")
    SIGNAL
  }
}
