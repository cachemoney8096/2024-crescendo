import frc.robot.utils.CRTUtil;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class CRTUtilTest {
  @Test
  void test(){
    Optional<CRTUtil> crtUtilO = CRTUtil.init(42.0, 44.0, Math.PI, 22, 0.2);
    if(crtUtilO.isEmpty()){
      System.out.println("Empty Optional");
      assert false;
      return;
    }
    CRTUtil crtUtil = crtUtilO.get();
    System.out.println(crtUtil.getAbsolutePosition(0.8333333333, 0.9318181818));
  }
}
