include <BOSL2/std.scad>

// Adjust this angle as desired!
face_elevation = 30; // (degrees)

aruco_size = 15;
shaft_diameter = 8.6;
screw_diameter = 2.5;

height = shaft_diameter+4;
size = aruco_size * 5/3;

depth = size * cos(face_elevation);
h = sin(face_elevation) * size;

$fn = 50;

union() {

// Main body + set screw hole
diff("remove") {

    // Cube
    cuboid([size, depth, height], rounding=1, edges="Z")
    // Minus tube with rounded ends for the instrument shaft
    tag("remove") prism_connector(circle(r=shaft_diameter/2, $fn=100), parent(), FRONT, parent(), BACK, fillet=1);
    // Minus cylinder for set screw
    tag("remove") rotate([0, 90, 0]) cylinder(h=size/2+1, r=screw_diameter/2);

}

// Add elevated face
difference() {

// Large block of material
translate([0, 0, (h+height)/2])
cuboid([size, depth, h], rounding=1, edges="Z");

// Cut off above desired face
translate([0, -(depth)/2, height/2])
rotate([face_elevation, 0, 0])
translate([0, size/2, h/2])
cuboid([size+2, size+2, h]);

}

}
