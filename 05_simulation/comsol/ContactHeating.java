/*
 * ATLAS dock contact Joule heating — COMSOL 6.4 (Electric Currents + Heat Transfer,
 * Electromagnetic Heating coupling), 2D axisymmetric.
 *
 * Two components:
 *   comp1  "validation"  pin + carrier PCB only, contact layer at the tip. Same assumptions
 *          as 05_simulation/matlab/contact_heating.m (all I^2(Rc+Rpin) at the tip, tip
 *          insulated, board edge adiabatic). Must reproduce theta ~= 156 K/W.
 *   comp2  "realistic"   adds the agent pad/PCB on top of the contact, so heat can also
 *          leave through the agent side, and the 20 mOhm pin resistance is distributed
 *          along the pin instead of lumped at the tip.
 *
 * Build/run (no server needed):
 *   comsol compile ContactHeating.java
 *   comsol batch -inputfile ContactHeating.class
 * Writes contact_heating_comsol.csv and ContactHeating.mph next to this file.
 */
import com.comsol.model.*;
import com.comsol.model.util.*;
import java.io.*;

public class ContactHeating {

  static String OUT = System.getProperty("user.dir");   // run comsol batch from this folder

  public static Model run() throws Exception {
    Model mv = ModelUtil.create("Mval");
    Model mr = ModelUtil.create("Mreal");
    params(mv); params(mr);
    build(mv, "comp1", false);
    build(mr, "comp1", true);

    double[] Is  = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
    double[] Rcs = {0.010, 0.030, 0.100, 0.200};

    // COMSOL's security settings block plain Java file I/O, so results go into a
    // COMSOL table and are written with a Table export node.
    double[][] rows = new double[2 * Is.length * Rcs.length][7];
    int k = 0;
    for (Model model : new Model[]{mv, mr}) {
      String name = (model == mv) ? "validation" : "realistic";
      for (double I : Is) {
        for (double Rc : Rcs) {
          model.param().set("I0", I + "[A]");
          model.param().set("Rc", Rc + "[ohm]");
          model.study("std1").run();
          model.result().numerical("gev_comp1").set("data", "dset1");
          double[][] v = model.result().numerical("gev_comp1").getReal();
          double P = v[0][0], Tmax = v[1][0] - 273.15;
          double Pexp = I * I * (Rc + 0.020);
          rows[k++] = new double[]{model == mv ? 0 : 1, I, Rc, P, Pexp, Tmax, (Tmax - 40.0) / Pexp};
        }
      }
      model.param().set("I0", "1[A]");
      model.param().set("Rc", "30[mohm]");
      model.study("std1").run();
      model.save(OUT + "/ContactHeating_" + name + ".mph");
    }
    mv.result().table().create("tbl1", "Table");
    mv.result().table("tbl1").setColumnHeaders(new String[]{"model_0val_1real", "I_A", "Rc_ohm", "P_joule_W", "P_expected_W", "Tmax_C", "theta_K_per_W"});
    mv.result().table("tbl1").setTableData(rows);
    mv.result().export().create("exp1", "Table");
    mv.result().export("exp1").set("table", "tbl1");
    mv.result().export("exp1").set("filename", OUT + "/contact_heating_comsol.txt");
    mv.result().export("exp1").run();
    mv.save(OUT + "/ContactHeating_validation.mph");
    return mv;
  }

  static void params(Model model) {
    model.label("ATLAS_contact_heating");
    model.param().set("I0", "1[A]", "contact current");
    model.param().set("Rc", "30[mohm]", "contact interface resistance");
    model.param().set("Rpin", "20[mohm]", "pin barrel + spring resistance");
    model.param().set("rp", "1[mm]", "pin tip radius");
    model.param().set("Lp", "8[mm]", "pin length above board");
    model.param().set("tl", "50[um]", "contact layer thickness");
    model.param().set("rb", "12[mm]", "board radius");
    model.param().set("tb", "1.6[mm]", "board thickness");
    model.param().set("kb", "(385*35e-6+0.3*(1.6e-3-35e-6))/1.6e-3[W/(m*K)]", "homogenised FR4 + 1 oz Cu");
    model.param().set("h0", "10[W/(m^2*K)]", "natural convection");
    model.param().set("Tamb", "40[degC]", "enclosure ambient");
    model.param().set("sig_brass", "1.5e7[S/m]");
    model.param().set("sig_pin_eff", "Lp/(Rpin*pi*rp^2)", "pin with Rpin distributed");
    model.param().set("sig_lay_val", "tl/((Rc+Rpin)*pi*rp^2)", "validation: Rc+Rpin lumped at tip");
    model.param().set("sig_lay_real", "tl/(Rc*pi*rp^2)", "realistic: only Rc at the interface");

  }

  static void build(Model model, String c, boolean agentSide) {
    ModelNode comp = model.component().create(c, true);
    comp.geom().create("g_" + c, 2);
    comp.geom("g_" + c).axisymmetric(true);
    comp.geom("g_" + c).lengthUnit("mm");
    comp.geom("g_" + c).create("board", "Rectangle").set("pos", new String[]{"0", "-tb"}).set("size", new String[]{"rb", "tb"});
    comp.geom("g_" + c).create("pin", "Rectangle").set("pos", new String[]{"0", "0"}).set("size", new String[]{"rp", "Lp"});
    comp.geom("g_" + c).create("lay", "Rectangle").set("pos", new String[]{"0", "Lp"}).set("size", new String[]{"rp", "tl"});
    if (agentSide) {
      comp.geom("g_" + c).create("abrd", "Rectangle").set("pos", new String[]{"0", "Lp+tl"}).set("size", new String[]{"rb", "tb"});
    }
    comp.geom("g_" + c).run();

    // ---- selections by coordinates (r = x, z = y), units mm; tags suffixed per component
    cur = c;
    domBox(comp, S("s_board"), -1, 13, -1.7, 0.01);
    domBox(comp, S("s_pin"),   -1, 1.01, -0.01, 8.01);
    domBox(comp, S("s_lay"),   -1, 1.01, 7.99, 8.06);
    domBox(comp, S("s_ec"),    -1, 1.01, -0.01, 8.06);
    if (agentSide) domBox(comp, S("s_abrd"), -1, 13, 8.04, 9.7);
    bndBox(comp, S("b_pinbase"), -0.01, 1.01, -0.01, 0.01);   // pin/board interface (r < rp, z = 0)
    bndBox(comp, S("b_tip"),     -0.01, 1.01, 8.04, 8.06);    // top of contact layer
    bndBox(comp, S("b_pinside"), 0.99, 1.01, -0.01, 8.01);
    bndBox(comp, S("b_laysid"),  0.99, 1.01, 7.99, 8.06);
    bndBox(comp, S("b_btop"),    0.99, 12.5, -0.01, 0.01);
    bndBox(comp, S("b_bbot"),    -0.01, 12.5, -1.61, -1.59);
    String[] conv = agentSide
        ? new String[]{S("b_pinside"), S("b_laysid"), S("b_btop"), S("b_bbot"), S("b_abot"), S("b_atop")}
        : new String[]{S("b_pinside"), S("b_laysid"), S("b_btop"), S("b_bbot")};
    if (agentSide) {
      bndBox(comp, S("b_abot"), 0.99, 12.5, 8.04, 8.06);
      bndBox(comp, S("b_atop"), -0.01, 12.5, 9.64, 9.66);
    }
    comp.selection().create(S("b_conv"), "Union").set("entitydim", 1).set("input", conv);

    // ---- materials
    mat(comp, S("m_board"), S("s_board"), "kb", "1e-6[S/m]");
    mat(comp, S("m_pin"), S("s_pin"), "110[W/(m*K)]", agentSide ? "sig_pin_eff" : "sig_brass");
    mat(comp, S("m_lay"), S("s_lay"), "110[W/(m*K)]", agentSide ? "sig_lay_real" : "sig_lay_val");
    if (agentSide) mat(comp, S("m_abrd"), S("s_abrd"), "kb", "1e-6[S/m]");

    // ---- physics
    comp.physics().create("ec", "ConductiveMedia", "g_" + c);
    String EC = "ec", HT = "ht";
    comp.physics(EC).selection().named(S("s_ec"));
    comp.physics(EC).create("term1", "Terminal", 1).selection().named(S("b_pinbase"));
    comp.physics(EC).feature("term1").set("TerminalType", "Current");
    comp.physics(EC).feature("term1").set("I0", "I0");
    comp.physics(EC).create("gnd1", "Ground", 1).selection().named(S("b_tip"));

    comp.physics().create("ht", "HeatTransfer", "g_" + c);
    comp.physics(HT).feature("init1").set("Tinit", "Tamb");
    comp.physics(HT).create("conv1", "HeatFluxBoundary", 1).selection().named(S("b_conv"));
    comp.physics(HT).feature("conv1").set("HeatFluxType", "ConvectiveHeatFlux");
    comp.physics(HT).feature("conv1").set("h", "h0");
    comp.physics(HT).feature("conv1").set("Text", "Tamb");

    comp.multiphysics().create("emh1", "ElectromagneticHeating", 2);
    comp.multiphysics("emh1").selection().named(S("s_ec"));
    comp.multiphysics("emh1").set("EMHeat_physics", "ec");
    comp.multiphysics("emh1").set("Heat_physics", "ht");

    // ---- probes
    comp.cpl().create(S("maxop"), "Maximum").selection().all();
    comp.cpl().create(S("intop"), "Integration").selection().named(S("s_ec"));

    // ---- mesh: fine in the thin contact layer
    comp.mesh().create("mesh_" + c);
    comp.mesh("mesh_" + c).create("sz", "Size");
    comp.mesh("mesh_" + c).feature("sz").selection().geom("g_" + c, 2);
    comp.mesh("mesh_" + c).feature("sz").selection().named(S("s_lay"));
    comp.mesh("mesh_" + c).feature("sz").set("custom", "on");
    comp.mesh("mesh_" + c).feature("sz").set("hmaxactive", true);
    comp.mesh("mesh_" + c).feature("sz").set("hmax", "0.02[mm]");
    comp.mesh("mesh_" + c).create("ftri", "FreeTri");
    comp.mesh("mesh_" + c).feature("size").set("hauto", 2);
    comp.mesh("mesh_" + c).run();

    model.study().create("std1");
    model.study("std1").create("stat", "Stationary");

    // global evaluation: Joule power, Tmax
    model.result().numerical().create("gev_" + c, "EvalGlobal");
    model.result().numerical("gev_" + c).set("expr", new String[]{"I0*ec.V0_1", c + "." + S("maxop") + "(T)"});
    model.result().numerical("gev_" + c).set("unit", new String[]{"W", "K"});
  }

  static String cur = "";
  static String S(String t) { return t + "_" + cur; }

  static void domBox(ModelNode comp, String tag, double x0, double x1, double y0, double y1) {
    comp.selection().create(tag, "Box").set("entitydim", 2)
        .set("xmin", x0).set("xmax", x1).set("ymin", y0).set("ymax", y1).set("condition", "inside");
  }

  static void bndBox(ModelNode comp, String tag, double x0, double x1, double y0, double y1) {
    comp.selection().create(tag, "Box").set("entitydim", 1)
        .set("xmin", x0).set("xmax", x1).set("ymin", y0).set("ymax", y1).set("condition", "inside");
  }

  static void mat(ModelNode comp, String tag, String sel, String k, String sigma) {
    comp.material().create(tag, "Common");
    comp.material(tag).selection().named(sel);
    comp.material(tag).propertyGroup("def").set("thermalconductivity", new String[]{k});
    comp.material(tag).propertyGroup("def").set("electricconductivity", new String[]{sigma});
    comp.material(tag).propertyGroup("def").set("relpermittivity", new String[]{"1"});
    comp.material(tag).propertyGroup("def").set("density", new String[]{"8500"});
    comp.material(tag).propertyGroup("def").set("heatcapacity", new String[]{"380"});
  }

  public static void main(String[] args) throws Exception {
    run();
  }
}
