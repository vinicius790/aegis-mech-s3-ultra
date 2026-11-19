// cad/mounts_opcionais/wand_enclosure_esp32c3.scad — Varinha Ergonomica COMET-AIR-MOUSE v3.0
$fn = 48;

module wand_case_v3() {
    difference() {
        // Corpo cilindrico ergonomico p/ ESP32-C3 SuperMini + MPU6050 + LiPo 1S
        cylinder(r = 16.5, h = 114);
        translate([0, 0, 3]) cylinder(r = 13.8, h = 112);
        // Janela USB-C na base inferior
        translate([-5.5, -17.5, 4]) cube([11.0, 6.0, 6.5]);
        // Furos para 2 microswitches (GPIO3 Left / GPIO2 Right)
        translate([0, 12, 72]) rotate([-90, 0, 0]) cylinder(r = 3.2, h = 8);
        translate([0, 12, 92]) rotate([-90, 0, 0]) cylinder(r = 3.2, h = 8);
    }
}

wand_case_v3();
