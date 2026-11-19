// cad/painel_sensores.scad — AEGIS-MECH v3.0 (Imprimivel Sem Colisoes, Parede=6.85mm)
$fn = 64;

bloco_w = 56.0;    // Corrige ruptura anterior (45.6 > 45.0): parede lateral = 6.85 mm!
bloco_d = 26.0;
bloco_h = 36.0;
us_dist = 26.0;    // Espacamento oficial centro-a-centro do HC-SR04 (igual arena_base)
us_r    = 8.15;    // Diametro 16.0 mm + folga 0.30 mm FDM
us_z    = 12.5;
tof_z   = 27.0;
x_c1    = (bloco_w - us_dist) / 2; // 15.0 mm
x_c2    = x_c1 + us_dist;          // 41.0 mm

module painel_sensores_v3() {
    difference() {
        union() {
            cube([bloco_w, bloco_d, bloco_h]);
            translate([-9, 0, 0])      cube([9, bloco_d, 5.5]);
            translate([bloco_w, 0, 0]) cube([9, bloco_d, 5.5]);
        }
        // 1. Par HC-SR04
        translate([x_c1, -1, us_z]) rotate([-90, 0, 0]) cylinder(r = us_r, h = bloco_d + 2);
        translate([x_c2, -1, us_z]) rotate([-90, 0, 0]) cylinder(r = us_r, h = bloco_d + 2);
        translate([(bloco_w - 46.5)/2, 10.0, us_z - 10.8])
            cube([46.5, bloco_d, 21.6]);

        // 2. Cavidade VL53L1X + abertura conica livre > 27 deg FOV anti-crosstalk IR 940nm
        translate([(bloco_w - 18.0)/2, 7.0, tof_z - 6.5])
            cube([18.0, bloco_d, 13.0]);
        translate([bloco_w/2, 7.1, tof_z])
            rotate([90, 0, 0])
                cylinder(r1 = 3.8, r2 = 8.4, h = 8.2);

        // 3. Passagem traseira p/ chicote 26AWG e furos M3 nas abas
        translate([(bloco_w - 10.0)/2, bloco_d - 6.0, -1]) cube([10.0, 7.0, 8.0]);
        translate([-4.5, bloco_d/2, -1])        cylinder(r = 1.65, h = 8.0);
        translate([bloco_w + 4.5, bloco_d/2, -1]) cylinder(r = 1.65, h = 8.0);
    }
}

painel_sensores_v3();
