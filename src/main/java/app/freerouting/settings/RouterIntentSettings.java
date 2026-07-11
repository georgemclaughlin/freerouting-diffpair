package app.freerouting.settings;

import app.freerouting.util.gson.GsonProvider;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.annotations.SerializedName;
import java.io.IOException;
import java.io.Serializable;
import java.io.StringReader;
import java.math.BigDecimal;
import java.nio.ByteBuffer;
import java.nio.charset.CharacterCodingException;
import java.nio.charset.CodingErrorAction;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class RouterIntentSettings implements Serializable, Cloneable {
  private static final String SCHEMA = "kicad-agent-router-intent";
  private static final int SCHEMA_VERSION = 6;
  private static final Gson VALIDATION_GSON = new GsonBuilder().serializeNulls().create();
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
      "net_type",
      "priority",
      "scope",
      "route_order_rank",
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
      "allowed_layers",
      "same_layer_required",
      "max_vias_per_net",
      "matched_via_transitions_required",
      "route_as_coupled_pair",
      "target_width_mm",
      "target_gap_mm",
      "gap_tolerance_mm",
      "endpoint_escape_width_mm",
      "endpoint_escape_length_mm",
      "max_skew_mm",
      "max_stub_mm",
      "min_parallel_length_ratio",
      "max_uncoupled_length_mm",
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
  private static final Set<String> NET_TYPE_VALUES = Set.of(
      "normal",
      "critical_net",
      "differential_pair_member",
      "route_length_match_member",
      "local_support",
      "block_port",
      "source_copper");
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
  private transient String intentSha256;

  public static RouterIntentSettings load(Path path) throws IOException {
    byte[] payload = Files.readAllBytes(path);
    String decodedPayload;
    try {
      decodedPayload = StandardCharsets.UTF_8.newDecoder()
          .onMalformedInput(CodingErrorAction.REPORT)
          .onUnmappableCharacter(CodingErrorAction.REPORT)
          .decode(ByteBuffer.wrap(payload))
          .toString();
    } catch (CharacterCodingException e) {
      throw new IllegalArgumentException("router intent payload must be valid UTF-8", e);
    }

    try (StringReader reader = new StringReader(decodedPayload)) {
      JsonElement rootElement = JsonParser.parseReader(reader);
      if (!rootElement.isJsonObject()) {
        throw new IllegalArgumentException("router intent payload root must be an object");
      }
      JsonObject root = rootElement.getAsJsonObject();
      validatePayloadObject(root);

      RouterIntentSettings result = GsonProvider.GSON.fromJson(root, RouterIntentSettings.class);
      if (result == null) {
        throw new IllegalArgumentException("router intent payload is empty");
      }
      result.intentFile = path.toAbsolutePath().normalize().toString();
      result.intentSha256 = RouterApplicationReceipt.sha256(payload);
      result.validateForApplicationReceipt();
      result.rebuildNetIntentIndex();
      return result;
    }
  }

  public String intentSha256() {
    return this.intentSha256;
  }

  /** Revalidates the mutable intent object before its support receipt is emitted. */
  void validateForApplicationReceipt() {
    JsonObject root = VALIDATION_GSON.toJsonTree(this).getAsJsonObject();
    root.remove("intent_file");
    validatePayloadObject(root);
  }

  private static void validatePayloadObject(JsonObject root) {
    validateExactObjectKeys("router intent", root, TOP_LEVEL_KEYS);
    String schema = requireString(root, "schema", "router intent", false);
    if (!SCHEMA.equals(schema)) {
      throw new IllegalArgumentException("unsupported router intent schema: " + schema);
    }
    requireInteger(root, "schema_version", "router intent", SCHEMA_VERSION, SCHEMA_VERSION, false);
    requireString(root, "profile", "router intent", false);

    JsonObject settingsObject = requireObject(root, "settings", "router intent");
    validateExactObjectKeys("router intent settings", settingsObject, SETTINGS_KEYS);
    requireInteger(settingsObject, "deterministic_seed", "router intent settings", 0, Integer.MAX_VALUE, false);
    requireBoolean(settingsObject, "single_thread_default", "router intent settings");
    requireEnum(settingsObject, "route_order", "router intent settings", ROUTE_ORDER_VALUES);
    requireEnum(
        settingsObject,
        "ripup_protection",
        "router intent settings",
        SETTINGS_RIPUP_PROTECTION_VALUES);

    Set<String> netNames = new HashSet<>();
    Set<String> objectiveIds = new HashSet<>();
    Set<String> pairNets = new HashSet<>();
    for (JsonObject entry : requireObjectArray(root, "net_intents", "router intent")) {
      validateExactObjectKeys("router intent net_intents entry", entry, NET_INTENT_KEYS);
      String net = requireString(entry, "net", "router intent net_intents entry", false);
      requireUnique(netNames, net, "router intent net_intents net");
      requireEnum(entry, "net_type", "router intent net_intents entry", NET_TYPE_VALUES);
      requireEnum(entry, "priority", "router intent net_intents entry", PRIORITY_VALUES);
      requireEnum(entry, "scope", "router intent net_intents entry", SCOPE_VALUES);
      requireInteger(entry, "route_order_rank", "router intent net_intents entry", 0, Integer.MAX_VALUE, false);
      requireStringArray(entry, "preferred_layers", "router intent net_intents entry", 0, Integer.MAX_VALUE);
      requireStringArray(entry, "plane_layers", "router intent net_intents entry", 0, Integer.MAX_VALUE);
      requireString(entry, "route_class", "router intent net_intents entry", true);
      requireFiniteNumber(entry, "track_width_mm", "router intent net_intents entry", 0.0, false, null, true);
      requireFiniteNumber(entry, "clearance_mm", "router intent net_intents entry", 0.0, false, null, true);
      requireFiniteNumber(entry, "via_diameter_mm", "router intent net_intents entry", 0.0, false, null, true);
      requireFiniteNumber(entry, "via_drill_mm", "router intent net_intents entry", 0.0, false, null, true);
      requireEnum(entry, "ripup_protection", "router intent net_intents entry", RIPUP_PROTECTION_VALUES);
      requireStringArray(entry, "critical_path_ids", "router intent net_intents entry", 0, Integer.MAX_VALUE);
      requireStringArray(entry, "differential_pair_ids", "router intent net_intents entry", 0, Integer.MAX_VALUE);
      requireStringArray(entry, "route_length_match_ids", "router intent net_intents entry", 0, Integer.MAX_VALUE);
      requireStringArray(entry, "local_support_ids", "router intent net_intents entry", 0, Integer.MAX_VALUE);
      requireStringArray(entry, "block_port_ids", "router intent net_intents entry", 0, Integer.MAX_VALUE);
      requireStringArray(entry, "source_copper_ids", "router intent net_intents entry", 0, Integer.MAX_VALUE);
    }

    for (JsonObject entry : requireObjectArray(root, "critical_paths", "router intent")) {
      validateExactObjectKeys("router intent critical_paths entry", entry, CRITICAL_PATH_KEYS);
      requireUnique(
          objectiveIds,
          requireString(entry, "id", "router intent critical_paths entry", false),
          "router intent objective id");
      requireString(entry, "net", "router intent critical_paths entry", false);
      requireEnum(entry, "priority", "router intent critical_paths entry", PRIORITY_VALUES);
      requireString(entry, "from", "router intent critical_paths entry", false);
      requireString(entry, "to", "router intent critical_paths entry", false);
      requireStringArray(entry, "preferred_layers", "router intent critical_paths entry", 0, Integer.MAX_VALUE);
      requireFiniteNumber(entry, "max_length_mm", "router intent critical_paths entry", 0.0, false, null, true);
      requireFiniteNumber(entry, "max_extra_mm", "router intent critical_paths entry", 0.0, true, null, true);
      requireFiniteNumber(entry, "max_ratio", "router intent critical_paths entry", 1.0, true, null, true);
    }

    for (JsonObject entry : requireObjectArray(root, "differential_pairs", "router intent")) {
      validateExactObjectKeys("router intent differential_pairs entry", entry, DIFFERENTIAL_PAIR_KEYS);
      requireUnique(
          objectiveIds,
          requireString(entry, "id", "router intent differential_pairs entry", false),
          "router intent objective id");
      String positiveNet = requireString(entry, "positive_net", "router intent differential_pairs entry", false);
      String negativeNet = requireString(entry, "negative_net", "router intent differential_pairs entry", false);
      if (positiveNet.equals(negativeNet)) {
        throw new IllegalArgumentException("router intent differential pair nets must differ");
      }
      requireUnique(pairNets, positiveNet, "router intent differential pair member net");
      requireUnique(pairNets, negativeNet, "router intent differential pair member net");
      requireEnum(entry, "priority", "router intent differential_pairs entry", PRIORITY_VALUES);
      String positiveFrom = requireString(entry, "positive_from", "router intent differential_pairs entry", true);
      String positiveTo = requireString(entry, "positive_to", "router intent differential_pairs entry", true);
      String negativeFrom = requireString(entry, "negative_from", "router intent differential_pairs entry", true);
      String negativeTo = requireString(entry, "negative_to", "router intent differential_pairs entry", true);
      boolean anyEndpoint = positiveFrom != null || positiveTo != null || negativeFrom != null || negativeTo != null;
      boolean allEndpoints = positiveFrom != null && positiveTo != null && negativeFrom != null && negativeTo != null;
      if (anyEndpoint != allEndpoints) {
        throw new IllegalArgumentException("router intent differential pair endpoints must be all present or all null");
      }
      requireStringArray(
          entry, "positive_preferred_layers", "router intent differential_pairs entry", 0, Integer.MAX_VALUE);
      requireStringArray(
          entry, "negative_preferred_layers", "router intent differential_pairs entry", 0, Integer.MAX_VALUE);
      requireStringArray(entry, "allowed_layers", "router intent differential_pairs entry", 0, Integer.MAX_VALUE);
      requireBoolean(entry, "same_layer_required", "router intent differential_pairs entry");
      requireInteger(entry, "max_vias_per_net", "router intent differential_pairs entry", 0, Integer.MAX_VALUE, true);
      requireBoolean(entry, "matched_via_transitions_required", "router intent differential_pairs entry");
      requireBoolean(entry, "route_as_coupled_pair", "router intent differential_pairs entry");
      requireFiniteNumber(entry, "target_width_mm", "router intent differential_pairs entry", 0.0, false, null, true);
      Double targetGap = requireFiniteNumber(
          entry, "target_gap_mm", "router intent differential_pairs entry", 0.0, false, null, true);
      Double gapTolerance = requireFiniteNumber(
          entry, "gap_tolerance_mm", "router intent differential_pairs entry", 0.0, true, null, true);
      if (gapTolerance != null && targetGap == null) {
        throw new IllegalArgumentException("router intent differential pair gap tolerance requires a target gap");
      }
      Double escapeWidth = requireFiniteNumber(
          entry, "endpoint_escape_width_mm", "router intent differential_pairs entry", 0.0, false, null, true);
      Double escapeLength = requireFiniteNumber(
          entry, "endpoint_escape_length_mm", "router intent differential_pairs entry", 0.0, false, null, true);
      if ((escapeWidth == null) != (escapeLength == null)) {
        throw new IllegalArgumentException("router intent differential pair escape width and length must both be present");
      }
      if (escapeWidth != null && !allEndpoints) {
        throw new IllegalArgumentException("router intent differential pair escape requires all endpoints");
      }
      requireFiniteNumber(entry, "max_skew_mm", "router intent differential_pairs entry", 0.0, true, null, true);
      requireFiniteNumber(entry, "max_stub_mm", "router intent differential_pairs entry", 0.0, true, null, true);
      requireFiniteNumber(
          entry, "min_parallel_length_ratio", "router intent differential_pairs entry", 0.0, true, 1.0, true);
      requireFiniteNumber(
          entry, "max_uncoupled_length_mm", "router intent differential_pairs entry", 0.0, true, null, true);
      requireBoolean(entry, "require_parallel_evidence", "router intent differential_pairs entry");
    }

    for (JsonObject entry : requireObjectArray(root, "route_length_matches", "router intent")) {
      validateExactObjectKeys("router intent route_length_matches entry", entry, ROUTE_LENGTH_MATCH_KEYS);
      requireUnique(
          objectiveIds,
          requireString(entry, "id", "router intent route_length_matches entry", false),
          "router intent objective id");
      requireStringArray(entry, "nets", "router intent route_length_matches entry", 2, 32);
      requireEnum(entry, "priority", "router intent route_length_matches entry", PRIORITY_VALUES);
      requireFiniteNumber(entry, "max_skew_mm", "router intent route_length_matches entry", 0.0, true, null, false);
    }

    for (JsonObject entry : requireObjectArray(root, "local_support", "router intent")) {
      validateExactObjectKeys("router intent local_support entry", entry, LOCAL_SUPPORT_KEYS);
      requireUnique(
          objectiveIds,
          requireString(entry, "id", "router intent local_support entry", false),
          "router intent objective id");
      requireEnum(entry, "kind", "router intent local_support entry", LOCAL_SUPPORT_KIND_VALUES);
      requireStringArray(entry, "nets", "router intent local_support entry", 1, Integer.MAX_VALUE);
      requireStringArray(entry, "pad_refs", "router intent local_support entry", 0, Integer.MAX_VALUE);
      requireEnum(entry, "priority", "router intent local_support entry", PRIORITY_VALUES);
      requireStringArray(entry, "preferred_layers", "router intent local_support entry", 0, Integer.MAX_VALUE);
      requireFiniteNumber(entry, "max_distance_mm", "router intent local_support entry", 0.0, true, null, true);
      requireFiniteNumber(
          entry, "max_return_distance_mm", "router intent local_support entry", 0.0, true, null, true);
    }

    for (JsonObject entry : requireObjectArray(root, "block_ports", "router intent")) {
      validateExactObjectKeys("router intent block_ports entry", entry, BLOCK_PORT_KEYS);
      requireUnique(
          objectiveIds,
          requireString(entry, "id", "router intent block_ports entry", false),
          "router intent objective id");
      requireString(entry, "block", "router intent block_ports entry", false);
      requireString(entry, "port", "router intent block_ports entry", false);
      requireEnum(entry, "kind", "router intent block_ports entry", BLOCK_PORT_KIND_VALUES);
      requireString(entry, "net", "router intent block_ports entry", false);
      String padRef = requireString(entry, "pad_ref", "router intent block_ports entry", true);
      Double x = requireFiniteNumber(entry, "x_mm", "router intent block_ports entry", null, true, null, true);
      Double y = requireFiniteNumber(entry, "y_mm", "router intent block_ports entry", null, true, null, true);
      if ((padRef != null) == (x != null || y != null) || (x == null) != (y == null)) {
        throw new IllegalArgumentException("router intent block port must use exactly one complete point form");
      }
      String boundaryName = requireString(entry, "boundary_name", "router intent block_ports entry", true);
      Double boundaryX = requireFiniteNumber(
          entry, "boundary_center_x_mm", "router intent block_ports entry", null, true, null, true);
      Double boundaryY = requireFiniteNumber(
          entry, "boundary_center_y_mm", "router intent block_ports entry", null, true, null, true);
      Double boundaryWidth = requireFiniteNumber(
          entry, "boundary_width_mm", "router intent block_ports entry", 0.0, false, null, true);
      Double boundaryHeight = requireFiniteNumber(
          entry, "boundary_height_mm", "router intent block_ports entry", 0.0, false, null, true);
      int boundaryParts = (boundaryName == null ? 0 : 1)
          + (boundaryX == null ? 0 : 1)
          + (boundaryY == null ? 0 : 1)
          + (boundaryWidth == null ? 0 : 1)
          + (boundaryHeight == null ? 0 : 1);
      if (boundaryParts != 0 && boundaryParts != 5) {
        throw new IllegalArgumentException("router intent block port boundary must be complete or null");
      }
    }
  }

  private static void validateExactObjectKeys(String label, JsonObject object, Set<String> expectedKeys) {
    for (String key : object.keySet()) {
      if (!expectedKeys.contains(key)) {
        throw new IllegalArgumentException(label + " has unsupported field: " + key);
      }
    }
    for (String key : expectedKeys) {
      if (!object.has(key)) {
        throw new IllegalArgumentException(label + " is missing required field: " + key);
      }
    }
  }

  private static JsonObject requireObject(JsonObject object, String field, String label) {
    JsonElement value = requiredValue(object, field, label);
    if (!value.isJsonObject()) {
      throw new IllegalArgumentException(label + " field " + field + " must be an object");
    }
    return value.getAsJsonObject();
  }

  private static List<JsonObject> requireObjectArray(JsonObject object, String field, String label) {
    JsonElement value = requiredValue(object, field, label);
    if (!value.isJsonArray()) {
      throw new IllegalArgumentException(label + " field " + field + " must be an array");
    }
    List<JsonObject> result = new ArrayList<>();
    for (JsonElement entry : value.getAsJsonArray()) {
      if (!entry.isJsonObject()) {
        throw new IllegalArgumentException(label + " field " + field + " entries must be objects");
      }
      result.add(entry.getAsJsonObject());
    }
    return result;
  }

  private static String requireString(JsonObject object, String field, String label, boolean nullable) {
    JsonElement value = requiredValue(object, field, label);
    if (value.isJsonNull()) {
      if (nullable) {
        return null;
      }
      throw new IllegalArgumentException(label + " field " + field + " must not be null");
    }
    if (!value.isJsonPrimitive() || !value.getAsJsonPrimitive().isString()) {
      throw new IllegalArgumentException(label + " field " + field + " must be a string");
    }
    String result = value.getAsString();
    if (result.isBlank()) {
      throw new IllegalArgumentException(label + " field " + field + " must not be blank");
    }
    return result;
  }

  private static void requireBoolean(JsonObject object, String field, String label) {
    JsonElement value = requiredValue(object, field, label);
    if (!value.isJsonPrimitive() || !value.getAsJsonPrimitive().isBoolean()) {
      throw new IllegalArgumentException(label + " field " + field + " must be a boolean");
    }
  }

  private static Integer requireInteger(
      JsonObject object,
      String field,
      String label,
      int minimum,
      int maximum,
      boolean nullable) {
    JsonElement value = requiredValue(object, field, label);
    if (value.isJsonNull()) {
      if (nullable) {
        return null;
      }
      throw new IllegalArgumentException(label + " field " + field + " must not be null");
    }
    if (!value.isJsonPrimitive() || !value.getAsJsonPrimitive().isNumber()) {
      throw new IllegalArgumentException(label + " field " + field + " must be an integer");
    }
    try {
      BigDecimal decimal = value.getAsBigDecimal();
      int result = decimal.intValueExact();
      if (result < minimum || result > maximum) {
        throw new IllegalArgumentException(
            label + " field " + field + " must be between " + minimum + " and " + maximum);
      }
      return result;
    } catch (ArithmeticException | NumberFormatException e) {
      throw new IllegalArgumentException(label + " field " + field + " must be an integer", e);
    }
  }

  private static Double requireFiniteNumber(
      JsonObject object,
      String field,
      String label,
      Double minimum,
      boolean minimumInclusive,
      Double maximum,
      boolean nullable) {
    JsonElement value = requiredValue(object, field, label);
    if (value.isJsonNull()) {
      if (nullable) {
        return null;
      }
      throw new IllegalArgumentException(label + " field " + field + " must not be null");
    }
    if (!value.isJsonPrimitive() || !value.getAsJsonPrimitive().isNumber()) {
      throw new IllegalArgumentException(label + " field " + field + " must be a number");
    }
    double result;
    try {
      result = value.getAsDouble();
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException(label + " field " + field + " must be a number", e);
    }
    if (!Double.isFinite(result)) {
      throw new IllegalArgumentException(label + " field " + field + " must be finite");
    }
    if (minimum != null && (result < minimum || (!minimumInclusive && result == minimum))) {
      String comparison = minimumInclusive ? "at least " : "greater than ";
      throw new IllegalArgumentException(label + " field " + field + " must be " + comparison + minimum);
    }
    if (maximum != null && result > maximum) {
      throw new IllegalArgumentException(label + " field " + field + " must be at most " + maximum);
    }
    return result;
  }

  private static String[] requireStringArray(
      JsonObject object,
      String field,
      String label,
      int minimumSize,
      int maximumSize) {
    JsonElement value = requiredValue(object, field, label);
    if (!value.isJsonArray()) {
      throw new IllegalArgumentException(label + " field " + field + " must be an array");
    }
    int size = value.getAsJsonArray().size();
    if (size < minimumSize || size > maximumSize) {
      throw new IllegalArgumentException(
          label + " field " + field + " must contain between " + minimumSize + " and " + maximumSize + " entries");
    }
    String[] result = new String[size];
    Set<String> uniqueValues = new HashSet<>();
    for (int index = 0; index < size; index++) {
      JsonElement entry = value.getAsJsonArray().get(index);
      if (!entry.isJsonPrimitive() || !entry.getAsJsonPrimitive().isString() || entry.getAsString().isBlank()) {
        throw new IllegalArgumentException(label + " field " + field + " entries must be non-blank strings");
      }
      result[index] = entry.getAsString();
      requireUnique(uniqueValues, result[index], label + " field " + field + " entry");
    }
    return result;
  }

  private static void requireEnum(
      JsonObject object,
      String field,
      String label,
      Set<String> allowedValues) {
    String value = requireString(object, field, label, false);
    if (!allowedValues.contains(value)) {
      throw new IllegalArgumentException(label + " field " + field + " has unsupported value: " + value);
    }
  }

  private static JsonElement requiredValue(JsonObject object, String field, String label) {
    JsonElement result = object.get(field);
    if (result == null) {
      throw new IllegalArgumentException(label + " is missing required field: " + field);
    }
    return result;
  }

  private static void requireUnique(Set<String> values, String value, String label) {
    if (!values.add(value)) {
      throw new IllegalArgumentException(label + " must be unique: " + value);
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

  public int routeOrderRankForNet(String netName) {
    NetIntent intent = netIntent(netName);
    return intent == null ? 0 : intent.routeOrderRank();
  }

  public NetType netTypeForNet(String netName) {
    NetIntent intent = netIntent(netName);
    return intent == null ? NetType.NORMAL : intent.netType;
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

  public Double differentialPairTargetGapMmForNet(String netName) {
    DifferentialPairIntent pair = differentialPairForNet(netName);
    return pair == null ? null : pair.targetGapMm;
  }

  public Double differentialPairGapToleranceMmForNet(String netName) {
    DifferentialPairIntent pair = differentialPairForNet(netName);
    return pair == null ? null : pair.gapToleranceMm;
  }

  public Double differentialPairTargetWidthMmForNet(String netName) {
    DifferentialPairIntent pair = differentialPairForNet(netName);
    return pair == null ? null : pair.targetWidthMm;
  }

  public boolean isHardDifferentialPairLayerForNet(String netName, String layerName) {
    DifferentialPairIntent pair = differentialPairForNet(netName);
    if (pair == null || pair.allowedLayers == null || pair.allowedLayers.length == 0) {
      return true;
    }
    for (String allowedLayer : pair.allowedLayers) {
      if (layerName != null && layerName.equals(allowedLayer)) {
        return true;
      }
    }
    return false;
  }

  public boolean forbidsViasForDifferentialPairNet(String netName) {
    DifferentialPairIntent pair = differentialPairForNet(netName);
    return pair != null && pair.maxViasPerNet != null && pair.maxViasPerNet <= 0;
  }

  public boolean requiresCoupledDifferentialPairRoute(String netName) {
    DifferentialPairIntent pair = differentialPairForNet(netName);
    return pair != null && Boolean.TRUE.equals(pair.routeAsCoupledPair);
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
    @SerializedName("net_type")
    public NetType netType;
    @SerializedName("priority")
    public Priority priority;
    @SerializedName("scope")
    public Scope scope;
    @SerializedName("route_order_rank")
    public Integer routeOrderRank;
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

    public int routeOrderRank() {
      return routeOrderRank == null ? 0 : routeOrderRank;
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
    @SerializedName("allowed_layers")
    public String[] allowedLayers;
    @SerializedName("same_layer_required")
    public Boolean sameLayerRequired;
    @SerializedName("max_vias_per_net")
    public Integer maxViasPerNet;
    @SerializedName("matched_via_transitions_required")
    public Boolean matchedViaTransitionsRequired;
    @SerializedName("route_as_coupled_pair")
    public Boolean routeAsCoupledPair;
    @SerializedName("target_width_mm")
    public Double targetWidthMm;
    @SerializedName("target_gap_mm")
    public Double targetGapMm;
    @SerializedName("gap_tolerance_mm")
    public Double gapToleranceMm;
    @SerializedName("endpoint_escape_width_mm")
    public Double endpointEscapeWidthMm;
    @SerializedName("endpoint_escape_length_mm")
    public Double endpointEscapeLengthMm;
    @SerializedName("max_skew_mm")
    public Double maxSkewMm;
    @SerializedName("max_stub_mm")
    public Double maxStubMm;
    @SerializedName("min_parallel_length_ratio")
    public Double minParallelLengthRatio;
    @SerializedName("max_uncoupled_length_mm")
    public Double maxUncoupledLengthMm;
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

  public enum NetType {
    @SerializedName("normal")
    NORMAL,
    @SerializedName("critical_net")
    CRITICAL_NET,
    @SerializedName("differential_pair_member")
    DIFFERENTIAL_PAIR_MEMBER,
    @SerializedName("route_length_match_member")
    ROUTE_LENGTH_MATCH_MEMBER,
    @SerializedName("local_support")
    LOCAL_SUPPORT,
    @SerializedName("block_port")
    BLOCK_PORT,
    @SerializedName("source_copper")
    SOURCE_COPPER
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
