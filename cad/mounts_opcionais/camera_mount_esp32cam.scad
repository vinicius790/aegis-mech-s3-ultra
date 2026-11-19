// cad/mounts_opcionais/camera_mount_esp32cam.scad — Pan-Tilt Indexado 15 deg (Surveillance Robot)
$fn = 48;

module camera_mount_pan_tilt() {
    difference() {
        union() {
            cube([34, 48, 16]);
            translate([17, 24, -10]) cylinder(r = 14, h = 10); // Colar Pan
        }
        // Cavidade PCB ESP32-CAM (27.2 x 41.0 mm) + Ventilacao traseira
        translate([3.4, 3.5, 4]) cube([27.2, 41.0, 13]);
        translate([8, 8, -1]) cube([18, 22, 6]);
        // Lente OV2640 frontal (quando tampado/invertido) + indexacao 15 deg
        for (a = [0 : 15 : 165]) {
            rotate([0, 0, a]) translate([11, 0, -11]) cylinder(r = 1.2, h = 6);
        }
    }
}

camera_mount_pan_tilt();
