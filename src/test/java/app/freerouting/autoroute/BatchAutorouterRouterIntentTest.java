package app.freerouting.autoroute;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import app.freerouting.Freerouting;
import app.freerouting.board.FixedState;
import app.freerouting.board.RoutingBoard;
import app.freerouting.geometry.planar.IntPoint;
import app.freerouting.geometry.planar.Point;
import app.freerouting.io.BoardReadResult;
import app.freerouting.io.specctra.DsnReader;
import app.freerouting.rules.Net;
import app.freerouting.settings.GlobalSettings;
import app.freerouting.settings.RouterIntentSettings;
import app.freerouting.settings.RouterSettings;
import java.io.ByteArrayInputStream;
import java.nio.charset.StandardCharsets;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BatchAutorouterRouterIntentTest {

  private static final String USB_D_PLUS = "USB_D+";
  private static final String USB_D_MINUS = "USB_D-";
  private static final String TWO_LAYER_PAIR_DSN =
      "(pcb diff-pair-sibling-layer-test\n"
          + "  (parser (string_quote \"))\n"
          + "  (resolution um 10)\n"
          + "  (unit um)\n"
          + "  (structure\n"
          + "    (layer F.Cu (type signal) (property (index 0)))\n"
          + "    (layer B.Cu (type signal) (property (index 1)))\n"
          + "    (boundary (path pcb 0  0 0  100000 0  100000 -60000  0 -60000  0 0))\n"
          + "    (via \"Via[0-1]_600:300_um\")\n"
          + "    (rule (width 500) (clearance 500) (clearance 100 (type smd_smd)))\n"
          + "  )\n"
          + "  (library\n"
          + "    (padstack Rect[T]Pad_1000x1000_um\n"
          + "      (shape (rect F.Cu -500 -500 500 500))\n"
          + "      (shape (rect B.Cu -500 -500 500 500))\n"
          + "      (attach off)\n"
          + "    )\n"
          + "    (padstack \"Via[0-1]_600:300_um\"\n"
          + "      (shape (circle F.Cu 600))\n"
          + "      (shape (circle B.Cu 600))\n"
          + "      (attach off)\n"
          + "    )\n"
          + "    (image Pad_1x1mm\n"
          + "      (pin Rect[T]Pad_1000x1000_um 1 0 0)\n"
          + "    )\n"
          + "  )\n"
          + "  (placement\n"
          + "    (component Pad_1x1mm\n"
          + "      (place P1 10000 -20000 front 0.000000)\n"
          + "      (place P2 90000 -20000 front 0.000000)\n"
          + "      (place N1 10000 -30000 front 0.000000)\n"
          + "      (place N2 90000 -30000 front 0.000000)\n"
          + "    )\n"
          + "  )\n"
          + "  (network\n"
          + "    (net USB_D+ (pins P1-1 P2-1))\n"
          + "    (net USB_D- (pins N1-1 N2-1))\n"
          + "    (class default \"\" USB_D+ USB_D-\n"
          + "      (rule (width 500) (clearance 500))\n"
          + "      (circuit (use_via \"Via[0-1]_600:300_um\") (use_layer F.Cu B.Cu))\n"
          + "    )\n"
          + "  )\n"
          + "  (wiring)\n"
          + ")\n";

  @BeforeEach
  void setUp() {
    Freerouting.globalSettings = new GlobalSettings();
  }

  @Test
  void traceCostsPreferLayerUsedByAlreadyRoutedDifferentialPairSibling() throws Exception {
    RoutingBoard board = loadBoard(TWO_LAYER_PAIR_DSN);
    Net positive = board.rules.nets.get(USB_D_PLUS, 1);
    Net negative = board.rules.nets.get(USB_D_MINUS, 1);
    assertNotNull(positive);
    assertNotNull(negative);
    insertSiblingTrace(board, positive, 1);

    RouterSettings settings = new RouterSettings(board);
    settings.intent = differentialPairIntent();
    BatchAutorouter router = new BatchAutorouter(
        null,
        board,
        settings,
        true,
        false,
        settings.get_start_ripup_costs(),
        500);

    AutorouteControl.ExpansionCostFactor[] costs = router.traceCostsForRouterIntent(negative.net_number);

    assertEquals(2, costs.length);
    assertTrue(costs[0].horizontal > costs[1].horizontal);
    assertTrue(costs[0].vertical > costs[1].vertical);
    assertEquals(3.0, costs[0].horizontal / costs[1].horizontal, 0.01);
    assertEquals(3.0, costs[0].vertical / costs[1].vertical, 0.01);
  }

  private void insertSiblingTrace(RoutingBoard board, Net net, int layer) {
    int halfWidth = board.rules.get_trace_half_width(net.net_number, layer);
    int clearanceClass = net.get_class().get_trace_clearance_class();
    board.insert_trace(
        new Point[] {
            new IntPoint(10_000, -20_000),
            new IntPoint(90_000, -20_000)
        },
        layer,
        halfWidth,
        new int[] { net.net_number },
        clearanceClass,
        FixedState.UNFIXED);
  }

  private RouterIntentSettings differentialPairIntent() {
    RouterIntentSettings.DifferentialPairIntent pair = new RouterIntentSettings.DifferentialPairIntent();
    pair.id = "usb2_data";
    pair.positiveNet = USB_D_PLUS;
    pair.negativeNet = USB_D_MINUS;
    pair.priority = RouterIntentSettings.Priority.CRITICAL;

    RouterIntentSettings intent = new RouterIntentSettings();
    intent.differentialPairs = new RouterIntentSettings.DifferentialPairIntent[] { pair };
    return intent;
  }

  private static RoutingBoard loadBoard(String dsn) throws Exception {
    BoardReadResult result = DsnReader.readBoard(
        new ByteArrayInputStream(dsn.getBytes(StandardCharsets.UTF_8)), null, null);
    return switch (result) {
      case BoardReadResult.Success success -> (RoutingBoard) success.board();
      case BoardReadResult.OutlineMissing outlineMissing -> (RoutingBoard) outlineMissing.board();
      case BoardReadResult.ParseError parseError ->
          throw new IllegalStateException(parseError.location() + ": " + parseError.detail());
      case BoardReadResult.IoError ioError -> throw ioError.cause();
    };
  }
}
