package app.freerouting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.core.RoutingJob;
import app.freerouting.core.RoutingJobState;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

class FreeroutingCliResultTest {

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
}
