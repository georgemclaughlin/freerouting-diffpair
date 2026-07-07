package app.freerouting.settings;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
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
    assertEquals(5, intent.schemaVersion);
    assertEquals("esp32_air_quality_node", intent.profile);
    assertEquals(RouterIntentSettings.RouteOrder.PRIORITY_THEN_LOCAL_SCOPE, intent.settings.routeOrder);
    assertEquals(
        RouterIntentSettings.RuntimeRipupProtection.SOURCE_COPPER_AND_CRITICAL,
        intent.settings.ripupProtection);
    assertEquals(RouterIntentSettings.Priority.CRITICAL, intent.netIntents[0].priority);
    assertEquals(RouterIntentSettings.Scope.GLOBAL, intent.netIntents[0].scope);
    assertEquals(RouterIntentSettings.RipupProtection.CRITICAL, intent.netIntents[0].ripupProtection);
    assertArrayEquals(new String[] { "matched_controls" }, intent.netIntents[0].routeLengthMatchIds);
    assertEquals(RouterIntentSettings.LocalSupportKind.SAME_NET_PAD_TIE, intent.localSupport[0].kind);
    assertEquals(3, intent.priorityRankForNet("3V3"));
    assertEquals(2, intent.scopeRankForNet("ESP_BOOT_LOCAL"));
    assertEquals(2, intent.ripupProtectionRankForNet("ESP_BOOT_LOCAL"));
    assertTrue(intent.hasLocalScopeIntent("ESP_BOOT_LOCAL"));
    assertFalse(intent.hasLocalScopeIntent("3V3"));
    assertTrue(intent.hasBlockPortIntent("3V3"));
    assertTrue(intent.hasLocalConfinementIntent("3V3"));
    assertTrue(intent.hasPreferredLayerIntent("3V3"));
    assertTrue(intent.isPreferredLayerForNet("3V3", "F.Cu"));
    assertFalse(intent.isPreferredLayerForNet("3V3", "B.Cu"));
    assertTrue(intent.hasRouteLengthMatchIntents());
    assertEquals("matched_controls", intent.routeLengthMatchGroupForNet("3V3"));
    assertArrayEquals(new String[] { "ESP_BOOT_LOCAL" }, intent.routeLengthMatchSiblingNetsForNet("3V3"));
    assertEquals(2.0, intent.routeLengthMatchMaxSkewMmForNet("3V3"));
    assertEquals(RouterIntentSettings.Priority.HIGH, intent.routeLengthMatchPriorityForNet("3V3"));
    assertEquals(1, intent.routeLengthMatchesForNet("3V3").length);
    assertEquals(0, intent.priorityRankForNet("UNMENTIONED"));
  }

  @Test
  void differentialPairAccessorsExposeClosedPairMembership() {
    RouterIntentSettings intent = new RouterIntentSettings();
    RouterIntentSettings.DifferentialPairIntent pair = new RouterIntentSettings.DifferentialPairIntent();
    pair.id = "usb2_data";
    pair.positiveNet = "USB_D_PLUS";
    pair.negativeNet = "USB_D_MINUS";
    pair.priority = RouterIntentSettings.Priority.CRITICAL;
    pair.maxSkewMm = 0.75;
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] { pair };

    assertTrue(intent.hasDifferentialPairIntents());
    assertEquals("usb2_data", intent.differentialPairGroupForNet("USB_D_PLUS"));
    assertEquals("usb2_data", intent.differentialPairGroupForNet("USB_D_MINUS"));
    assertEquals(0, intent.differentialPairMemberRankForNet("USB_D_PLUS"));
    assertEquals(1, intent.differentialPairMemberRankForNet("USB_D_MINUS"));
    assertEquals(2, intent.differentialPairMemberRankForNet("VBUS"));
    assertEquals("USB_D_MINUS", intent.differentialPairSiblingNetForNet("USB_D_PLUS"));
    assertEquals("USB_D_PLUS", intent.differentialPairSiblingNetForNet("USB_D_MINUS"));
    assertEquals(0.75, intent.differentialPairMaxSkewMmForNet("USB_D_PLUS"));
    assertEquals(0.75, intent.differentialPairMaxSkewMmForNet("USB_D_MINUS"));
    assertTrue(intent.areDifferentialPairMembers("USB_D_PLUS", "USB_D_MINUS"));
    assertFalse(intent.areDifferentialPairMembers("USB_D_PLUS", "VBUS"));
  }

  @Test
  void differentialPairAccessorsExposeHardUsbPolicy() {
    RouterIntentSettings intent = new RouterIntentSettings();
    RouterIntentSettings.DifferentialPairIntent pair = new RouterIntentSettings.DifferentialPairIntent();
    pair.id = "usb2_data";
    pair.positiveNet = "USB_D_PLUS";
    pair.negativeNet = "USB_D_MINUS";
    pair.allowedLayers = new String[] { "F.Cu" };
    pair.sameLayerRequired = true;
    pair.maxViasPerNet = 0;
    pair.routeAsCoupledPair = true;
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] { pair };

    assertTrue(intent.isHardDifferentialPairLayerForNet("USB_D_PLUS", "F.Cu"));
    assertFalse(intent.isHardDifferentialPairLayerForNet("USB_D_PLUS", "In1.Cu"));
    assertTrue(intent.forbidsViasForDifferentialPairNet("USB_D_MINUS"));
    assertTrue(intent.requiresCoupledDifferentialPairRoute("USB_D_PLUS"));
    assertTrue(intent.isHardDifferentialPairLayerForNet("VBUS", "In1.Cu"));
    assertFalse(intent.forbidsViasForDifferentialPairNet("VBUS"));
  }

  @Test
  void loadsDifferentialPairMaxUncoupledLength() throws IOException {
    RouterIntentSettings intent = RouterIntentSettings.load(writePayload(
        "router-intent.json",
        validPayload().replace(
            "\"differential_pairs\": []",
            """
            "differential_pairs": [
              {
                "id": "usb2_data",
                "positive_net": "USB_D+",
                "negative_net": "USB_D-",
                "priority": "critical",
                "positive_from": "TP1.1",
                "positive_to": "TP3.1",
                "negative_from": "TP2.1",
                "negative_to": "TP4.1",
                "positive_preferred_layers": ["F.Cu"],
                "negative_preferred_layers": ["F.Cu"],
                "allowed_layers": ["F.Cu"],
                "same_layer_required": true,
                "max_vias_per_net": 0,
                "matched_via_transitions_required": false,
                "route_as_coupled_pair": true,
                "target_width_mm": 0.2,
                "target_gap_mm": 1.0,
                "gap_tolerance_mm": 0.12,
                "endpoint_escape_width_mm": null,
                "endpoint_escape_length_mm": null,
                "max_skew_mm": 0.2,
                "max_stub_mm": 0.6,
                "min_parallel_length_ratio": null,
                "max_uncoupled_length_mm": 24.0,
                "require_parallel_evidence": true
              }
            ]""")));

    assertEquals(1, intent.differentialPairs.length);
    assertEquals(24.0, intent.differentialPairs[0].maxUncoupledLengthMm);
    assertTrue(intent.requiresCoupledDifferentialPairRoute("USB_D+"));
  }

  @Test
  void localSupportForNetReturnsOnlyMatchingClosedEntries() throws IOException {
    RouterIntentSettings intent = RouterIntentSettings.load(writePayload("router-intent.json", validPayload()));

    RouterIntentSettings.LocalSupportIntent[] bootSupport = intent.localSupportForNet("ESP_BOOT_LOCAL");
    RouterIntentSettings.LocalSupportIntent[] powerSupport = intent.localSupportForNet("3V3");
    RouterIntentSettings.BlockPortIntent[] powerPorts = intent.blockPortsForNet("3V3");

    assertEquals(1, bootSupport.length);
    assertEquals("local_support_1_same_net_pad_tie", bootSupport[0].id);
    assertEquals(0, powerSupport.length);
    assertEquals(1, powerPorts.length);
    assertEquals("main_power", powerPorts[0].block);
    assertEquals(RouterIntentSettings.BlockPortKind.POWER_OUTPUT, powerPorts[0].kind);
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
  void rejectsUnsupportedRouteLengthMatchFieldsAndEnums() throws IOException {
    Path fieldPayload = writePayload("bad-route-length-field.json", validPayload().replace(
        "\"max_skew_mm\": 2.0",
        "\"max_skew_mm\": 2.0,\n      \"freeform\": true"));
    Path enumPayload = writePayload("bad-route-length-enum.json", validPayload().replace(
        "\"priority\": \"high\",\n      \"max_skew_mm\": 2.0",
        "\"priority\": \"urgent\",\n      \"max_skew_mm\": 2.0"));

    assertThrows(IllegalArgumentException.class, () -> RouterIntentSettings.load(fieldPayload));
    assertThrows(IllegalArgumentException.class, () -> RouterIntentSettings.load(enumPayload));
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
          "schema_version": 5,
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
              "route_length_match_ids": ["matched_controls"],
              "local_support_ids": [],
              "block_port_ids": ["block_port_1_main_power_output"],
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
              "route_length_match_ids": ["matched_controls"],
              "local_support_ids": ["local_support_1_same_net_pad_tie"],
              "block_port_ids": [],
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
          "route_length_matches": [
            {
              "id": "matched_controls",
              "nets": ["3V3", "ESP_BOOT_LOCAL"],
              "priority": "high",
              "max_skew_mm": 2.0
            }
          ],
          "block_ports": [
            {
              "id": "block_port_1_main_power_output",
              "block": "main_power",
              "port": "output",
              "kind": "power_output",
              "net": "3V3",
              "pad_ref": "C3.1",
              "x_mm": null,
              "y_mm": null,
              "boundary_name": "regulator_cluster",
              "boundary_center_x_mm": 50.0,
              "boundary_center_y_mm": -35.0,
              "boundary_width_mm": 110.0,
              "boundary_height_mm": 30.0
            }
          ],
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
