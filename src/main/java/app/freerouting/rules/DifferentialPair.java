package app.freerouting.rules;

import java.io.Serializable;

/**
 * Metadata describing two nets that should be treated as a differential pair.
 *
 * <p>This is currently a parsed and serialized constraint record. Pair-aware routing and length
 * matching are implemented separately from this data model.
 */
public record DifferentialPair(int first_net_no, int second_net_no) implements Serializable {

  public DifferentialPair {
    if (first_net_no <= 0 || second_net_no <= 0) {
      throw new IllegalArgumentException("Differential pair net numbers must be positive");
    }
    if (first_net_no == second_net_no) {
      throw new IllegalArgumentException("Differential pair nets must be distinct");
    }
  }
}
