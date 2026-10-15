/*
 * Adds the result plots to the two contact-heating models (ContactHeating.java saves them with the
 * solution but no plot groups, so the COMSOL Desktop showed only the bare geometry) and exports PNGs.
 *   comsol compile AddPlots.java
 *   comsol batch -inputfile AddPlots.class -nosave
 * PNG export is off: in batch mode on macOS it hangs at 'Generating image' (no display).
 */
import com.comsol.model.*;
import com.comsol.model.util.*;

public class AddPlots {
  static boolean EXPORT_PNG = false;
  static String D = System.getProperty("user.dir");     // run comsol batch from this folder

  public static Model run() throws Exception {
    Model last = null;
    for (String name : new String[]{"realistic", "validation"}) {
      Model m = ModelUtil.load("M_" + name, D + "/ContactHeating_" + name + ".mph");
      for (String t : new String[]{"pg1", "pg2", "pg3"}) {
        try { m.result().remove(t); } catch (Exception e) { }
      }
      try { m.result().dataset().remove("rev1"); } catch (Exception e) { }
      m.result().dataset().create("rev1", "Revolve2D");
      m.result().dataset("rev1").set("data", "dset1");
      m.result().dataset("rev1").set("revangle", "270");

      m.result().create("pg1", "PlotGroup2D");
      m.result("pg1").label("Temperature (r-z section), I = 1 A, Rc = 30 mOhm");
      m.result("pg1").set("data", "dset1");
      m.result("pg1").create("surf1", "Surface");
      m.result("pg1").feature("surf1").set("expr", "T");
      m.result("pg1").feature("surf1").set("unit", "degC");
      m.result("pg1").feature("surf1").set("colortable", "HeatCameraLight");
      m.result("pg1").create("con1", "Contour");
      m.result("pg1").feature("con1").set("expr", "T");
      m.result("pg1").feature("con1").set("unit", "degC");
      m.result("pg1").feature("con1").set("number", "12");
      m.result("pg1").feature("con1").set("coloring", "uniform");
      m.result("pg1").feature("con1").set("color", "black");

      m.result().create("pg2", "PlotGroup3D");
      m.result("pg2").label("Temperature (3D, revolved)");
      m.result("pg2").set("data", "rev1");
      m.result("pg2").create("surf1", "Surface");
      m.result("pg2").feature("surf1").set("expr", "T");
      m.result("pg2").feature("surf1").set("unit", "degC");
      m.result("pg2").feature("surf1").set("colortable", "HeatCameraLight");

      m.result().create("pg3", "PlotGroup2D");
      m.result("pg3").label("Current density |J|");
      m.result("pg3").set("data", "dset1");
      m.result("pg3").create("surf1", "Surface");
      m.result("pg3").feature("surf1").set("expr", "ec.normJ");
      m.result("pg3").feature("surf1").set("unit", "A/mm^2");
      m.result("pg3").create("str1", "Streamline");
      m.result("pg3").feature("str1").set("expr", new String[]{"ec.Jr", "ec.Jz"});
      m.result("pg3").feature("str1").set("posmethod", "uniform");
      m.result("pg3").feature("str1").set("udist", "0.03");

      for (String pg : new String[]{"pg1", "pg2", "pg3"}) {
        m.result(pg).run();
        if (!EXPORT_PNG) continue;            // batch-mode image export hangs without a display (macOS)
        String tag = "img_" + pg;
        m.result().export().create(tag, "Image");
        try { m.result().export(tag).set("sourceobject", pg); } catch (Exception e) { m.result().export(tag).set("plotgroup", pg); }
        m.result().export(tag).set("pngfilename", D + "/ContactHeating_" + name + "_" + pg + ".png");
        m.result().export(tag).set("size", "manualweb");
        m.result().export(tag).set("width", "1400");
        m.result().export(tag).set("height", "900");
        m.result().export(tag).run();
      }
      m.save(D + "/ContactHeating_" + name + ".mph");
      last = m;
    }
    return last;
  }

  public static void main(String[] args) throws Exception { run(); }
}
