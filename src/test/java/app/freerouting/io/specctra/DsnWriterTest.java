package app.freerouting.io.specctra;

import app.freerouting.Freerouting;
import app.freerouting.board.RoutingBoard;
import app.freerouting.rules.DifferentialPair;
import app.freerouting.settings.GlobalSettings;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
import java.nio.charset.StandardCharsets;

import static org.junit.jupiter.api.Assertions.*;

class DsnWriterTest {

  @BeforeEach
  void setUp() {
    Freerouting.globalSettings = new GlobalSettings();
  }

  @Test
  void writesValidDsnHeader() throws Exception {
    RoutingBoard board = DsnTestFixtures.loadBoard("Issue143-rpi_splitter.dsn");
    ByteArrayOutputStream out = new ByteArrayOutputStream();
    DsnWriter.write(board, out, "test", false);
    String content = out.toString(StandardCharsets.UTF_8);
    assertTrue(content.startsWith("(pcb"), "DSN output must start with (pcb");
    assertTrue(content.contains("(structure"), "DSN output must contain (structure scope");
  }

  @Test
  void roundtripPreservesLayerCount() throws Exception {
    RoutingBoard original = DsnTestFixtures.loadBoard("Issue143-rpi_splitter.dsn");
    int originalLayers = original.get_layer_count();
    ByteArrayOutputStream out = new ByteArrayOutputStream();
    DsnWriter.write(original, out, "roundtrip", false);
    RoutingBoard reloaded = DsnTestFixtures.loadBoard(out.toByteArray());
    assertEquals(originalLayers, reloaded.get_layer_count());
  }

  @Test
  void roundtripPreservesDifferentialPairMetadata() throws Exception {
    RoutingBoard original = DsnTestFixtures.loadBoardFromContent(DsnTestFixtures.DSN_WITH_DIFFERENTIAL_PAIR);
    ByteArrayOutputStream out = new ByteArrayOutputStream();
    DsnWriter.write(original, out, "roundtrip-diff-pair", false);

    String content = out.toString(StandardCharsets.UTF_8);
    assertTrue(content.contains("(pair"), "DSN output must contain pair metadata");

    RoutingBoard reloaded = DsnTestFixtures.loadBoard(out.toByteArray());
    assertEquals(1, reloaded.rules.differential_pairs.count());
    DifferentialPair pair = reloaded.rules.differential_pairs.get(0);
    assertEquals("USB_D+", reloaded.rules.nets.get(pair.first_net_no()).name);
    assertEquals("USB_D-", reloaded.rules.nets.get(pair.second_net_no()).name);
  }

  @Test
  void roundtripPreservesScopedDifferentialPairPins() throws Exception {
    RoutingBoard original = DsnTestFixtures.loadBoardFromContent(DsnTestFixtures.DSN_WITH_SCOPED_DIFFERENTIAL_PAIR);
    ByteArrayOutputStream out = new ByteArrayOutputStream();
    DsnWriter.write(original, out, "roundtrip-scoped-diff-pair", false);

    String content = out.toString(StandardCharsets.UTF_8);
    assertTrue(content.contains("(pins"), "DSN output must contain scoped pair pins");

    RoutingBoard reloaded = DsnTestFixtures.loadBoard(out.toByteArray());
    DifferentialPair pair = reloaded.rules.differential_pairs.get(0);
    assertTrue(pair.has_scoped_pins());
    assertEquals("U1-14", pair.first_from_pin());
    assertEquals("R6-2", pair.first_to_pin());
    assertEquals("U1-13", pair.second_from_pin());
    assertEquals("R7-2", pair.second_to_pin());
  }

  @Test
  void compatModeProducesOutput() throws Exception {
    RoutingBoard board = DsnTestFixtures.loadBoard("Issue143-rpi_splitter.dsn");
    ByteArrayOutputStream out = new ByteArrayOutputStream();
    DsnWriter.write(board, out, "compat-test", true);
    String content = out.toString(StandardCharsets.UTF_8);
    assertTrue(content.startsWith("(pcb"), "Compat-mode DSN output must start with (pcb");
  }

  @Test
  void outputStreamContainsDataAfterWrite() throws Exception {
    RoutingBoard board = DsnTestFixtures.loadBoard("Issue143-rpi_splitter.dsn");
    ByteArrayOutputStream out = new ByteArrayOutputStream();
    DsnWriter.write(board, out, "flush-test", false);
    assertTrue(out.size() > 0, "Output stream must contain data after write (flush must have occurred)");
  }
}
