// cad/caixa_cofre.scad — AEGIS-MECH v3.0 (PCB 100x80 + 2x18650 + E-Stop Ø22 + MPU6050 Rigido)
$fn = 50;

par   = 5.0;
int_l = 200.0;
int_p = 160.0;
int_h = 115.0;

module caixa_cofre_v3() {
    difference() {
        cube([int_l + 2*par, int_p + 2*par, int_h + par]);
        translate([par, par, par]) cube([int_l, int_p, int_h + 2]);

        // Frontal: OLED (28x16), Anel WS2812B (R=22.5), Botao Modo (Ø12.2)
        translate([55, -1, 68])  cube([28, par + 2, 16]);
        translate([125, -1, 76]) rotate([-90, 0, 0]) cylinder(r = 22.5, h = par + 2);
        translate([170, -1, 76]) rotate([-90, 0, 0]) cylinder(r = 6.1, h = par + 2);

        // Saida do ferrolho lateral direito
        translate([int_l + par - 1, 72, 25]) cube([par + 2, 13.5, 16.0]);

        // Traseiro: Cogumelo E-STOP Ø22.3mm + PG7 + DC Barrel + USB-C
        translate([45,  int_p + par - 1, 65]) rotate([-90, 0, 0]) cylinder(r = 11.15, h = par + 2);
        translate([95,  int_p + par - 1, 30]) rotate([-90, 0, 0]) cylinder(r = 6.25, h = par + 2);
        translate([130, int_p + par - 1, 30]) rotate([-90, 0, 0]) cylinder(r = 4.0,  h = par + 2);
        translate([160, int_p + par - 1, 30]) cube([12.5, par + 2, 7.5]);

        // Venezianas laterais de conveccao DRV8871 / MP1584EN
        for (z_v = [25 : 10 : 55]) translate([-1, 40, z_v]) cube([par + 2, 45, 4]);
    }

    // 4 Torres M3 PCB 100x80mm (espacamento 92x72mm, furo Ø4.0 p/ inserto latao)
    for (px = [30, 122], py = [35, 107]) {
        translate([px, py, par]) difference() {
            cylinder(r = 4.5, h = 8.0);
            translate([0, 0, 1.5]) cylinder(r = 2.0, h = 7.0);
        }
    }
    // Berco rigido solidario ao chassi p/ MPU6050 100Hz
    translate([int_l + par - 26, par + 5, par]) cube([24, 20, 6]);
}

caixa_cofre_v3();
