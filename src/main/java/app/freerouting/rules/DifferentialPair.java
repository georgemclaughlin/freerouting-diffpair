package app.freerouting.rules;

import java.io.Serializable;

/**
 * Metadata describing two nets that should be treated as a differential pair.
 *
 * <p>Pin names are optional and use SPECCTRA component-pin identifiers, for example {@code R6-1}.
 * When present, they identify the routed path that should be length matched on each pair member.
 */
public record DifferentialPair(
    int first_net_no,
    int second_net_no,
    String first_from_pin,
    String first_to_pin,
    String second_from_pin,
    String second_to_pin) implements Serializable {

  public DifferentialPair(int p_first_net_no, int p_second_net_no) {
    this(p_first_net_no, p_second_net_no, null, null, null, null);
  }

  public DifferentialPair {
    if (first_net_no <= 0 || second_net_no <= 0) {
      throw new IllegalArgumentException("Differential pair net numbers must be positive");
    }
    if (first_net_no == second_net_no) {
      throw new IllegalArgumentException("Differential pair nets must be distinct");
    }
    boolean hasAnyPin = first_from_pin != null || first_to_pin != null || second_from_pin != null || second_to_pin != null;
    boolean hasAllPins = first_from_pin != null && first_to_pin != null && second_from_pin != null && second_to_pin != null;
    if (hasAnyPin && !hasAllPins) {
      throw new IllegalArgumentException("Differential pair scoped pins must be provided as a complete set");
    }
  }

  public boolean has_scoped_pins() {
    return first_from_pin != null;
  }
}
