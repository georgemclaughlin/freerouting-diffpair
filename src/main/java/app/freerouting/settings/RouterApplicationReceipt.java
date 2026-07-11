package app.freerouting.settings;

import com.google.gson.JsonObject;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/** Machine-readable proof of the router artifact and intent semantics used by a CLI run. */
public final class RouterApplicationReceipt {
  public static final String SCHEMA = "kicad-agent-router-application-receipt";
  public static final int SCHEMA_VERSION = 1;

  private static final Map<String, String> FIELD_SUPPORT = buildFieldSupport();

  private RouterApplicationReceipt() {
  }

  public static JsonObject create(RouterIntentSettings intent, String routerArtifactSha256) {
    if (intent == null) {
      throw new IllegalArgumentException("router intent is required for an application receipt");
    }
    intent.validateForApplicationReceipt();
    requireSha256("router artifact", routerArtifactSha256);
    requireSha256("router intent", intent.intentSha256());

    JsonObject receipt = new JsonObject();
    receipt.addProperty("schema", SCHEMA);
    receipt.addProperty("schema_version", SCHEMA_VERSION);
    receipt.addProperty("router_artifact_sha256", routerArtifactSha256);
    receipt.addProperty("router_intent_sha256", intent.intentSha256());
    receipt.addProperty("router_intent_schema", intent.schema);
    receipt.addProperty("router_intent_schema_version", intent.schemaVersion);

    JsonObject fieldSupport = new JsonObject();
    FIELD_SUPPORT.forEach(fieldSupport::addProperty);
    receipt.add("field_support", fieldSupport);
    return receipt;
  }

  public static Map<String, String> fieldSupport() {
    return FIELD_SUPPORT;
  }

  public static String sha256(Path path) throws IOException {
    MessageDigest digest = sha256Digest();
    try (InputStream stream = Files.newInputStream(path)) {
      byte[] buffer = new byte[8192];
      int count;
      while ((count = stream.read(buffer)) >= 0) {
        if (count > 0) {
          digest.update(buffer, 0, count);
        }
      }
    }
    return toHex(digest.digest());
  }

  static String sha256(byte[] bytes) {
    return toHex(sha256Digest().digest(bytes));
  }

  private static MessageDigest sha256Digest() {
    try {
      return MessageDigest.getInstance("SHA-256");
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException("JVM does not provide SHA-256", e);
    }
  }

  private static String toHex(byte[] bytes) {
    StringBuilder result = new StringBuilder(bytes.length * 2);
    for (byte value : bytes) {
      result.append(String.format("%02x", value & 0xff));
    }
    return result.toString();
  }

  private static void requireSha256(String label, String value) {
    if (value == null || !value.matches("[0-9a-f]{64}")) {
      throw new IllegalStateException(label + " SHA-256 is unavailable or invalid");
    }
  }

  private static Map<String, String> buildFieldSupport() {
    Map<String, String> result = new LinkedHashMap<>();

    add(result, "post_route_only",
        "schema",
        "schema_version",
        "profile",
        "settings.deterministic_seed",
        "settings.single_thread_default",
        "settings.route_order",
        "settings.ripup_protection");

    add(result, "search_hard",
        "net_intents[].net",
        "differential_pairs[].positive_net",
        "differential_pairs[].negative_net",
        "differential_pairs[].positive_from",
        "differential_pairs[].positive_to",
        "differential_pairs[].negative_from",
        "differential_pairs[].negative_to",
        "differential_pairs[].allowed_layers");

    add(result, "search_soft",
        "net_intents[].priority",
        "net_intents[].scope",
        "net_intents[].route_order_rank",
        "net_intents[].preferred_layers",
        "net_intents[].ripup_protection",
        "differential_pairs[].route_as_coupled_pair",
        "differential_pairs[].target_gap_mm",
        "differential_pairs[].gap_tolerance_mm",
        "differential_pairs[].max_skew_mm",
        "differential_pairs[].max_vias_per_net",
        "differential_pairs[].target_width_mm",
        "route_length_matches[].nets",
        "route_length_matches[].max_skew_mm",
        "local_support[].nets",
        "local_support[].pad_refs",
        "local_support[].max_distance_mm",
        "local_support[].max_return_distance_mm",
        "block_ports[].net",
        "block_ports[].boundary_center_x_mm",
        "block_ports[].boundary_center_y_mm",
        "block_ports[].boundary_width_mm",
        "block_ports[].boundary_height_mm");

    add(result, "scoring",
        "net_intents[].plane_layers",
        "critical_paths[].net",
        "critical_paths[].priority",
        "critical_paths[].max_length_mm",
        "differential_pairs[].priority",
        "route_length_matches[].id",
        "route_length_matches[].priority");

    add(result, "post_route_only",
        "net_intents[].net_type",
        "net_intents[].route_class",
        "net_intents[].track_width_mm",
        "net_intents[].clearance_mm",
        "net_intents[].via_diameter_mm",
        "net_intents[].via_drill_mm",
        "net_intents[].critical_path_ids",
        "net_intents[].differential_pair_ids",
        "net_intents[].route_length_match_ids",
        "net_intents[].local_support_ids",
        "net_intents[].block_port_ids",
        "net_intents[].source_copper_ids",
        "critical_paths[].id",
        "critical_paths[].from",
        "critical_paths[].to",
        "critical_paths[].preferred_layers",
        "critical_paths[].max_extra_mm",
        "critical_paths[].max_ratio",
        "differential_pairs[].id",
        "differential_pairs[].positive_preferred_layers",
        "differential_pairs[].negative_preferred_layers",
        "differential_pairs[].same_layer_required",
        "differential_pairs[].matched_via_transitions_required",
        "differential_pairs[].endpoint_escape_width_mm",
        "differential_pairs[].endpoint_escape_length_mm",
        "differential_pairs[].max_stub_mm",
        "differential_pairs[].min_parallel_length_ratio",
        "differential_pairs[].max_uncoupled_length_mm",
        "differential_pairs[].require_parallel_evidence",
        "local_support[].id",
        "local_support[].kind",
        "local_support[].priority",
        "local_support[].preferred_layers",
        "block_ports[].id",
        "block_ports[].block",
        "block_ports[].port",
        "block_ports[].kind",
        "block_ports[].pad_ref",
        "block_ports[].x_mm",
        "block_ports[].y_mm",
        "block_ports[].boundary_name");

    return Collections.unmodifiableMap(result);
  }

  private static void add(Map<String, String> result, String support, String... fields) {
    for (String field : fields) {
      if (result.put(field, support) != null) {
        throw new IllegalStateException("duplicate router intent field support entry: " + field);
      }
    }
  }
}
