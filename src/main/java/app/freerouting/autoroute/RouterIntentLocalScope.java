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
    if (board == null || intent == null || net == null || !intent.hasLocalConfinementIntent(net.name)) {
      return null;
    }

    double mmResolution = board.communication.get_resolution(Unit.MM);
    if (mmResolution <= 0) {
      return null;
    }

    RegionBounds bounds = new RegionBounds();
    includeLocalSupportBounds(board, intent, net, mmResolution, bounds);
    includeBlockPortBounds(intent, net, mmResolution, bounds);
    return bounds.toBox();
  }

  private static void includeLocalSupportBounds(
      RoutingBoard board,
      RouterIntentSettings intent,
      Net net,
      double mmResolution,
      RegionBounds bounds) {
    if (!intent.hasLocalScopeIntent(net.name)) {
      return;
    }

    RouterIntentSettings.LocalSupportIntent[] supports = intent.localSupportForNet(net.name);
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
        bounds.includePoint(center);
        padCount++;
      }
    }

    if (padCount == 0 || maxDistanceMm <= 0.0) {
      return;
    }

    int margin = Math.max(1, (int) Math.ceil(maxDistanceMm * mmResolution));
    bounds.expand(margin);
  }

  private static void includeBlockPortBounds(
      RouterIntentSettings intent,
      Net net,
      double mmResolution,
      RegionBounds bounds) {
    for (RouterIntentSettings.BlockPortIntent blockPort : intent.blockPortsForNet(net.name)) {
      if (blockPort.boundaryCenterXMm == null
          || blockPort.boundaryCenterYMm == null
          || blockPort.boundaryWidthMm == null
          || blockPort.boundaryHeightMm == null
          || blockPort.boundaryWidthMm <= 0
          || blockPort.boundaryHeightMm <= 0) {
        continue;
      }

      double halfWidth = blockPort.boundaryWidthMm / 2.0;
      double halfHeight = blockPort.boundaryHeightMm / 2.0;
      int minX = (int) Math.floor((blockPort.boundaryCenterXMm - halfWidth) * mmResolution);
      int minY = (int) Math.floor((blockPort.boundaryCenterYMm - halfHeight) * mmResolution);
      int maxX = (int) Math.ceil((blockPort.boundaryCenterXMm + halfWidth) * mmResolution);
      int maxY = (int) Math.ceil((blockPort.boundaryCenterYMm + halfHeight) * mmResolution);
      bounds.includeBox(minX, minY, maxX, maxY);
    }
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

  private static final class RegionBounds {
    private int minX = Integer.MAX_VALUE;
    private int minY = Integer.MAX_VALUE;
    private int maxX = Integer.MIN_VALUE;
    private int maxY = Integer.MIN_VALUE;

    void includePoint(FloatPoint point) {
      includeBox(
          (int) Math.floor(point.x),
          (int) Math.floor(point.y),
          (int) Math.ceil(point.x),
          (int) Math.ceil(point.y));
    }

    void includeBox(int boxMinX, int boxMinY, int boxMaxX, int boxMaxY) {
      this.minX = Math.min(this.minX, boxMinX);
      this.minY = Math.min(this.minY, boxMinY);
      this.maxX = Math.max(this.maxX, boxMaxX);
      this.maxY = Math.max(this.maxY, boxMaxY);
    }

    void expand(int margin) {
      if (!hasBounds()) {
        return;
      }
      this.minX -= margin;
      this.minY -= margin;
      this.maxX += margin;
      this.maxY += margin;
    }

    IntBox toBox() {
      if (!hasBounds()) {
        return null;
      }
      return new IntBox(this.minX, this.minY, this.maxX, this.maxY);
    }

    private boolean hasBounds() {
      return this.minX <= this.maxX && this.minY <= this.maxY;
    }
  }
}
