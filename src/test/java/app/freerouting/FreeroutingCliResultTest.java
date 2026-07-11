package app.freerouting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.core.RoutingJob;
import app.freerouting.core.RoutingJobState;
import app.freerouting.logger.AllowErrorLogs;
import app.freerouting.settings.RouterApplicationReceipt;
import app.freerouting.settings.RouterIntentSettings;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class FreeroutingCliResultTest {

  @TempDir
  Path tempDir;

  @Test
  void incompleteResultIsMachineReadable() {
    RoutingJob job = new RoutingJob();
    job.state = RoutingJobState.INCOMPLETE;
    job.incompleteConnectionCount = 3;

    String record = Freerouting.formatCliRoutingResult(job);
    JsonObject result = JsonParser.parseString(
        record.substring(Freerouting.CLI_ROUTING_RESULT_PREFIX.length())).getAsJsonObject();

    assertEquals("INCOMPLETE", result.get("state").getAsString());
    assertEquals(3, result.get("incomplete_connection_count").getAsInt());
  }

  @Test
  void failedResultReportsUnknownIncompleteCountAsNull() {
    RoutingJob job = new RoutingJob();
    job.state = RoutingJobState.TERMINATED;

    String record = Freerouting.formatCliRoutingResult(job);
    JsonObject result = JsonParser.parseString(
        record.substring(Freerouting.CLI_ROUTING_RESULT_PREFIX.length())).getAsJsonObject();

    assertEquals("TERMINATED", result.get("state").getAsString());
    assertTrue(result.get("incomplete_connection_count").isJsonNull());
  }

  @Test
  void intentRunReportsArtifactIntentAndFieldSupportReceipt() throws IOException {
    Path intentPath = tempDir.resolve("router-intent.json");
    Files.writeString(intentPath, """
        {
          "schema": "kicad-agent-router-intent",
          "schema_version": 6,
          "profile": "receipt_test",
          "settings": {
            "deterministic_seed": 1,
            "single_thread_default": true,
            "route_order": "priority_then_local_scope",
            "ripup_protection": "source_copper_and_critical"
          },
          "net_intents": [],
          "critical_paths": [],
          "differential_pairs": [],
          "route_length_matches": [],
          "local_support": [],
          "block_ports": []
        }
        """, StandardCharsets.UTF_8);
    RoutingJob job = new RoutingJob();
    job.state = RoutingJobState.COMPLETED;
    job.incompleteConnectionCount = 0;
    job.routerSettings.intent = RouterIntentSettings.load(intentPath);
    String artifactSha256 = "a".repeat(64);

    String record = Freerouting.formatCliRoutingResult(job, artifactSha256);
    JsonObject result = JsonParser.parseString(
        record.substring(Freerouting.CLI_ROUTING_RESULT_PREFIX.length())).getAsJsonObject();
    JsonObject receipt = result.getAsJsonObject("router_application_receipt");

    assertEquals(RouterApplicationReceipt.SCHEMA, receipt.get("schema").getAsString());
    assertEquals(RouterApplicationReceipt.SCHEMA_VERSION, receipt.get("schema_version").getAsInt());
    assertEquals(artifactSha256, receipt.get("router_artifact_sha256").getAsString());
    assertEquals(job.routerSettings.intent.intentSha256(), receipt.get("router_intent_sha256").getAsString());
    assertEquals("search_hard", receipt.getAsJsonObject("field_support")
        .get("differential_pairs[].allowed_layers").getAsString());
    assertEquals("post_route_only", receipt.getAsJsonObject("field_support")
        .get("differential_pairs[].same_layer_required").getAsString());
    assertEquals("search_soft", receipt.getAsJsonObject("field_support")
        .get("differential_pairs[].max_vias_per_net").getAsString());
    assertEquals("post_route_only", receipt.getAsJsonObject("field_support")
        .get("differential_pairs[].endpoint_escape_length_mm").getAsString());
    assertEquals("search_soft", receipt.getAsJsonObject("field_support")
        .get("route_length_matches[].max_skew_mm").getAsString());
    assertEquals(
        RouterApplicationReceipt.fieldSupport().size(),
        receipt.getAsJsonObject("field_support").size());
  }

  @Test
  @AllowErrorLogs("intentionally exercises the bounded CLI scheduler-start failure")
  void cliWaitTerminatesWhenSchedulerDoesNotPickUpReadyJob() {
    RoutingJob job = new RoutingJob();
    job.state = RoutingJobState.READY_TO_START;

    Freerouting.awaitCliRoutingJob(job, 5, 1);

    assertEquals(RoutingJobState.TERMINATED, job.state);
  }
}
