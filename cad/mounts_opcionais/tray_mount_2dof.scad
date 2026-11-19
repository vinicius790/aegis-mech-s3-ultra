// cad/mounts_opcionais/tray_mount_2dof.scad — Braco Planar 2-DOF (L1=120mm, L2=100mm SO-ARM100)
$fn = 48;

l1 = 120.0;
l2 = 100.0;

module servo_pocket() {
    difference() {
        cube([26, 32, 22]);
        translate([2.9, 3.0, 2]) cube([20.2, 26.0, 21]);
        translate([13, 16, -1]) cylinder(r = 3.1, h = 5);
    }
}

module link_arm(len_mm) {
    difference() {
        hull() {
            cylinder(r = 12, h = 6);
            translate([len_mm, 0, 0]) cylinder(r = 10, h = 6);
        }
        translate([0, 0, -1]) cylinder(r = 3.1, h = 8);
        translate([len_mm, 0, -1]) cylinder(r = 3.1, h = 8);
    }
}

servo_pocket();
translate([40, 0, 0]) link_arm(l1);
translate([40, 35, 0]) link_arm(l2);
