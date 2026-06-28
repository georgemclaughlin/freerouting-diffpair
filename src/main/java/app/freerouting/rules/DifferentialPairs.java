package app.freerouting.rules;

import java.io.Serializable;
import java.util.Vector;

/**
 * Collection of differential-pair metadata attached to board rules.
 */
public class DifferentialPairs implements Serializable {

  private final Vector<DifferentialPair> pair_arr = new Vector<>();

  public int count() {
    return pair_arr.size();
  }

  public DifferentialPair get(int p_index) {
    if (p_index < 0 || p_index >= pair_arr.size()) {
      return null;
    }
    return pair_arr.get(p_index);
  }

  public boolean contains(int p_first_net_no, int p_second_net_no) {
    for (DifferentialPair curr_pair : pair_arr) {
      if (curr_pair.first_net_no() == p_first_net_no && curr_pair.second_net_no() == p_second_net_no) {
        return true;
      }
      if (curr_pair.first_net_no() == p_second_net_no && curr_pair.second_net_no() == p_first_net_no) {
        return true;
      }
    }
    return false;
  }

  public DifferentialPair add(int p_first_net_no, int p_second_net_no) {
    if (contains(p_first_net_no, p_second_net_no)) {
      return null;
    }
    DifferentialPair result = new DifferentialPair(p_first_net_no, p_second_net_no);
    pair_arr.add(result);
    return result;
  }
}
