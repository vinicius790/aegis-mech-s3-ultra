// cad/trava_cremalheira.scad — AEGIS-MECH v3.0 (Engrenamento Real M1.25 + Pinhao Z=16 + AS5600)
$fn = 64;

mod_m      = 1.25;
passo      = PI * mod_m;       // 3.927 mm
alt_dente  = 2.25 * mod_m;     // 2.8125 mm
curso_util = 28.0;
Z_pinhao   = 16;
D_prim     = mod_m * Z_pinhao; // 20.0 mm -> 0.17453 mm/deg no AS5600!

larg_crem  = 12.0;
alt_crem   = 14.0;
comp_crem  = 78.0;

module dente_crem(h_extrude) {
    linear_extrude(height = h_extrude)
        polygon(points = [
            [-0.38*passo, 0], [-0.16*passo, alt_dente],
            [ 0.16*passo, alt_dente], [ 0.38*passo, 0]
        ]);
}

module trava_cremalheira_v3() {
    difference() {
        union() {
            cube([comp_crem, larg_crem, alt_crem]);
            for (x = [10 : passo : 62]) {
                translate([x, larg_crem, alt_crem])
                    rotate([90, 0, 0]) dente_crem(larg_crem);
            }
            translate([6, -2.5, 4]) cube([6, 2.5, 6]); // Came microswitches SW1
        }
        translate([16.0, larg_crem/2, -0.5]) cylinder(r = 2.6, h = 9.5); // Entalhe SOL1
        translate([26.0, larg_crem/2 - 1.7, -0.5]) cube([curso_util + 6.0, 3.4, 3.8]);
    }
}

module pinhao_e_ima_as5600() {
    r_raiz = (D_prim / 2) - 1.25 * mod_m;
    difference() {
        union() {
            cylinder(r = r_raiz, h = larg_crem);
            for (k = [0 : Z_pinhao - 1]) {
                rotate([0, 0, k * (360 / Z_pinhao)])
                    translate([r_raiz - 0.2, 0, 0])
                        rotate([0, 0, -90]) dente_crem(larg_crem);
            }
            translate([0, 0, larg_crem]) cylinder(r = 6.5, h = 6.0);
        }
        // D-shaft 6mm (chanfro 5.5mm) + bolso ima diametral Ø6.0 x 2.5mm
        difference() {
            translate([0, 0, -1]) cylinder(r = 3.1, h = larg_crem + 4.0);
            translate([2.55, -4, -1]) cube([3, 8, larg_crem + 4.0]);
        }
        translate([0, 0, larg_crem + 3.5]) cylinder(r = 3.12, h = 3.0);
    }
}

module caixa_mancal_v3() {
    difference() {
        union() {
            cube([84, 46, 40]);
            translate([38, 46, 8]) cube([26, 12, 26]); // Torre coaxial AS5600 @ 1.5mm gap
        }
        translate([-1, (46 - larg_crem - 0.6)/2, 10]) cube([88, larg_crem + 0.6, alt_crem + 3.6]);
        translate([38, 23, 27.5]) rotate([-90, 0, 0]) cylinder(r = 12.7, h = 25);
        translate([16, 23, -1]) cylinder(r = 6.2, h = 12);
        translate([4,  8, 12]) cube([14, 7, 10]);
        translate([32, 8, 12]) cube([14, 7, 10]);
        translate([38, 52, 27.5]) rotate([-90, 0, 0]) cylinder(r = 1.65, h = 8);
    }
}

translate([0, 65, 0])  trava_cremalheira_v3();
translate([95, 25, 0]) pinhao_e_ima_as5600();
caixa_mancal_v3();
