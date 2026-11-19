// cad/arena_base.scad — AEGIS-MECH v3.0 (Arena [120, 320] mm c/ VL53L1X subtraido + dutos)
$fn = 60;

arena_l     = 200.0;
arena_w     = 200.0;
piso_h      = 5.0;
par_t       = 8.0;
alt_borda   = 38.0;
us_r        = 8.15;
us_dist     = 26.0;
offset_cent = 95.0;

module janela_sensores_parede() {
    translate([-us_dist/2, 0, 14]) rotate([-90, 0, 0]) cylinder(r=us_r, h=par_t*3, center=true);
    translate([ us_dist/2, 0, 14]) rotate([-90, 0, 0]) cylinder(r=us_r, h=par_t*3, center=true);
    // Janela conica real do VL53L1X (corrige omission do v2.0 onde variaveis nunca eram subtraidas!)
    translate([0, -par_t/2 - 0.1, 28])
        rotate([-90, 0, 0])
            cylinder(r1 = 8.0, r2 = 4.0, h = par_t + 0.5);
    translate([-9.0, 2.5, 21.5]) cube([18.0, par_t, 13.0]);
}

module arena_base_v3() {
    ext_x = arena_l + 2*par_t;
    ext_y = arena_w + 2*par_t;
    difference() {
        cube([ext_x, ext_y, alt_borda]);
        translate([par_t, par_t, piso_h]) cube([arena_l, arena_w, alt_borda]);
        translate([par_t + offset_cent, par_t/2, 0]) janela_sensores_parede();
        translate([par_t/2, par_t + offset_cent, 0]) rotate([0, 0, -90]) janela_sensores_parede();

        // Dutos inferiores p/ passagem de chicote + recesso de espuma anti-multipath 5mm
        translate([-1, par_t/2 - 3, -0.5]) cube([ext_x + 2, 6.0, 4.0]);
        translate([par_t/2 - 3, -1, -0.5]) cube([6.0, ext_y + 2, 4.0]);
        translate([par_t + 10, ext_y - par_t - 0.1, piso_h + 2]) cube([arena_l - 20, 2.0, 26.0]);
        translate([ext_x - par_t - 0.1, par_t + 10, piso_h + 2]) cube([2.0, arena_w - 20, 26.0]);
    }
}

arena_base_v3();
