package app.freerouting.settings;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.settings.sources.CliSettings;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class RouterIntentSettingsTest {

  @TempDir
  Path tempDir;

  @Test
  void loadsClosedToolkitRouterIntentPayload() throws IOException {
    RouterIntentSettings intent = RouterIntentSettings.load(writePayload("router-intent.json", validPayload()));

    assertEquals("kicad-agent-router-intent", intent.schema);
    assertEquals(1, intent.schemaVersion);
    assertEquals("esp32_air_quality_node", intent.profile);
    assertEquals(RouterIntentSettings.RouteOrder.PRIORITY_THEN_LOCAL_SCOPE, intent.settings.routeOrder);
    assertEquals(
        RouterIntentSettings.RuntimeRipupProtection.SOURCE_COPPER_AND_CRITICAL,
        intent.settings.ripupProtection);
    assertEquals(RouterIntentSettings.Priority.CRITICAL, intent.netIntents[0].priority);
    assertEquals(RouterIntentSettings.Scope.GLOBAL, intent.netIntents[0].scope);
    assertEquals(RouterIntentSettings.RipupProtection.CRITICAL, intent.netIntents[0].ripupProtection);
    assertEquals(RouterIntentSettings.LocalSupportKind.SAME_NET_PAD_TIE, intent.localSupport[0].kind);
    assertEquals(3, intent.priorityRankForNet("3V3"));
    assertEquals(2, intent.scopeRankForNet("ESP_BOOT_LOCAL"));
    assertEquals(2, intent.ripupProtectionRankForNet("ESP_BOOT_LOCAL"));
    assertTrue(intent.hasPreferredLayerIntent("3V3"));
    assertTrue(intent.isPreferredLayerForNet("3V3", "F.Cu"));
    assertFalse(intent.isPreferredLayerForNet("3V3", "B.Cu"));
    assertEquals(0, intent.priorityRankForNet("UNMENTIONED"));
  }

  @Test
  void rejectsUnsupportedPayloadFields() throws IOException {
    Path payload = writePayload("bad-router-intent.json", validPayload().replace(
        "\"profile\": \"esp32_air_quality_node\",",
        "\"profile\": \"esp32_air_quality_node\",\n  \"freeform\": true,"));

    assertThrows(IllegalArgumentException.class, () -> RouterIntentSettings.load(payload));
  }

  @Test
  void rejectsUnsupportedPayloadEnumValues() throws IOException {
    Path payload = writePayload("bad-router-intent-value.json", validPayload().replace(
        "\"priority\": \"critical\",",
        "\"priority\": \"urgent\","));

    assertThrows(IllegalArgumentException.class, () -> RouterIntentSettings.load(payload));
  }

  @Test
  void cliRouterIntentFilePopulatesRouterSettingsIntent() throws IOException {
    Path payload = writePayload("router-intent.json", validPayload());

    RouterSettings settings = new CliSettings(new String[] {
        "--router.intent_file=" + payload
    }).getSettings();

    assertNotNull(settings.intent);
    assertEquals(3, settings.intent.priorityRankForNet("3V3"));
  }

  @Test
  void mergedRouterSettingsPreserveIntentRanks() throws IOException {
    RouterSettings source = new RouterSettings();
    source.intent = RouterIntentSettings.load(writePayload("router-intent.json", validPayload()));
    RouterSettings target = new RouterSettings();

    target.applyNewValuesFrom(source);

    assertNotNull(target.intent);
    assertEquals(3, target.intent.priorityRankForNet("3V3"));
    assertEquals(2, target.intent.scopeRankForNet("ESP_BOOT_LOCAL"));
  }

  private Path writePayload(String filename, String payload) throws IOException {
    Path path = tempDir.resolve(filename);
    Files.writeString(path, payload, StandardCharsets.UTF_8);
    return path;
  }

  private String validPayload() {
    return """
        {
          "schema": "kicad-agent-router-intent",
          "schema_version": 1,
          "profile": "esp32_air_quality_node",
          "settings": {
            "deterministic_seed": 123,
            "single_thread_default": true,
            "route_order": "priority_then_local_scope",
            "ripup_protection": "source_copper_and_critical"
          },
          "net_intents": [
            {
              "net": "3V3",
              "priority": "critical",
              "scope": "global",
              "preferred_layers": ["F.Cu", "In1.Cu"],
              "plane_layers": [],
              "route_class": "Power",
              "track_width_mm": 0.25,
              "clearance_mm": 0.15,
              "via_diameter_mm": null,
              "via_drill_mm": null,
              "ripup_protection": "critical",
              "critical_path_ids": ["esp32_boot_pullup_supply_tie"],
              "differential_pair_ids": [],
              "local_support_ids": [],
              "source_copper_ids": []
            },
            {
              "net": "ESP_BOOT_LOCAL",
              "priority": "normal",
              "scope": "local",
              "preferred_layers": ["F.Cu"],
              "plane_layers": [],
              "route_class": "Control",
              "track_width_mm": 0.2,
              "clearance_mm": 0.15,
              "via_diameter_mm": null,
              "via_drill_mm": null,
              "ripup_protection": "local_support",
              "critical_path_ids": [],
              "differential_pair_ids": [],
              "local_support_ids": ["local_support_1_same_net_pad_tie"],
              "source_copper_ids": []
            }
          ],
          "critical_paths": [
            {
              "id": "esp32_boot_pullup_supply_tie",
              "net": "3V3",
              "priority": "critical",
              "from": "R15.1",
              "to": "U3.2",
              "preferred_layers": ["F.Cu"],
              "max_length_mm": 2.0,
              "max_extra_mm": null,
              "max_ratio": null
            }
          ],
          "differential_pairs": [],
          "local_support": [
            {
              "id": "local_support_1_same_net_pad_tie",
              "kind": "same_net_pad_tie",
              "nets": ["ESP_BOOT_LOCAL"],
              "pad_refs": ["U3.27", "R15.1"],
              "priority": "normal",
              "preferred_layers": ["F.Cu"],
              "max_distance_mm": 1.0,
              "max_return_distance_mm": null
            }
          ]
        }
        """;
  }
}
