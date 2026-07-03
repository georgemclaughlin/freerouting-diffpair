package app.freerouting.autoroute;

import app.freerouting.board.Component;
import app.freerouting.board.Pin;
import app.freerouting.board.RoutingBoard;
import app.freerouting.board.Unit;
import app.freerouting.geometry.planar.FloatPoint;
import app.freerouting.geometry.planar.IntBox;
import app.freerouting.rules.Net;
import app.freerouting.settings.RouterIntentSettings;

final class RouterIntentLocalScope {

  private RouterIntentLocalScope() {
  }

  static IntBox localRegion(RoutingBoard board, RouterIntentSettings intent, Net net) {
    if (board == null || intent == null || net == null || !intent.hasLocalScopeIntent(net.name)) {
      return null;
    }

    RouterIntentSettings.LocalSupportIntent[] supports = intent.localSupportForNet(net.name);
    if (supports.length == 0) {
      return null;
    }

    int minX = Integer.MAX_VALUE;
    int minY = Integer.MAX_VALUE;
    int maxX = Integer.MIN_VALUE;
    int maxY = Integer.MIN_VALUE;
    int padCount = 0;
    double maxDistanceMm = 0.0;
    for (RouterIntentSettings.LocalSupportIntent support : supports) {
      if (support.maxDistanceMm != null) {
        maxDistanceMm = Math.max(maxDistanceMm, support.maxDistanceMm);
      }
      if (support.maxReturnDistanceMm != null) {
        maxDistanceMm = Math.max(maxDistanceMm, support.maxReturnDistanceMm);
      }
      if (support.padRefs == null) {
        continue;
      }
      for (String padRef : support.padRefs) {
        Pin pin = findPinByPadRef(board, padRef);
        if (pin == null) {
          continue;
        }
        FloatPoint center = pin.get_center().to_float();
        minX = Math.min(minX, (int) Math.floor(center.x));
        minY = Math.min(minY, (int) Math.floor(center.y));
        maxX = Math.max(maxX, (int) Math.ceil(center.x));
        maxY = Math.max(maxY, (int) Math.ceil(center.y));
        padCount++;
      }
    }

    if (padCount == 0 || maxDistanceMm <= 0.0) {
      return null;
    }

    int margin = Math.max(1, (int) Math.ceil(maxDistanceMm * board.communication.get_resolution(Unit.MM)));
    return new IntBox(minX - margin, minY - margin, maxX + margin, maxY + margin);
  }

  static boolean pointInside(IntBox box, FloatPoint point) {
    if (box == null || point == null) {
      return false;
    }
    return point.x >= box.ll.x
        && point.x <= box.ur.x
        && point.y >= box.ll.y
        && point.y <= box.ur.y;
  }

  static double distanceOutside(IntBox box, FloatPoint point) {
    if (box == null || point == null) {
      return 0.0;
    }
    if (pointInside(box, point)) {
      return 0.0;
    }
    return point.distance(box.nearest_point(point));
  }

  private static Pin findPinByPadRef(RoutingBoard board, String padRefValue) {
    if (board == null) {
      return null;
    }
    PadRef padRef = PadRef.parse(padRefValue);
    if (padRef == null) {
      return null;
    }

    Component component = board.components.get(padRef.ref());
    if (component == null) {
      return null;
    }
    for (Pin pin : board.get_component_pins(component.no)) {
      if (padRef.pad().equals(pin.name())) {
        return pin;
      }
    }
    return null;
  }

  private record PadRef(String ref, String pad) {
    static PadRef parse(String value) {
      if (value == null || value.isBlank()) {
        return null;
      }
      int separator = value.lastIndexOf('.');
      if (separator < 0) {
        separator = value.lastIndexOf('-');
      }
      if (separator <= 0 || separator >= value.length() - 1) {
        return null;
      }
      return new PadRef(value.substring(0, separator), value.substring(separator + 1));
    }
  }
}
